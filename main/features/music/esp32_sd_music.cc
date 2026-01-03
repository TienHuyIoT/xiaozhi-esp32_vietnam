#include "esp32_sd_music.h"
#include "board.h"
#include "display.h"
#include "audio_codec.h"
#include "application.h"
#include "sd_card.h"
#include <sys/stat.h>
#include <dirent.h>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <unordered_set>
#include <cstdlib>  // atoi, strtoull

#include <esp_log.h>
#include <esp_heap_caps.h>
#include "esp_audio_dec.h"
#include "esp_audio_simple_dec_default.h"
#include <esp_pthread.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

static const char* TAG = "Esp32SdMusic";

// ================================================================
//  UTILITY HÀM TỰ DO (UTF-8, tên, thời gian, gợi ý)
// ================================================================

// Chuẩn hóa về lowercase nhưng chỉ động tới ASCII (giữ nguyên UTF-8 đa byte)
static std::string ToLowerAscii(const std::string& s)
{
    std::string out = s;
    for (char& c : out) {
        unsigned char uc = static_cast<unsigned char>(c);
        if (uc < 128) {
            c = static_cast<char>(std::tolower(uc));
        }
    }
    return out;
}

static std::string ExtractDirectory(const std::string& full_path)
{
    size_t pos = full_path.find_last_of('/');
    if (pos == std::string::npos) return std::string();
    return full_path.substr(0, pos);
}

static std::string ExtractBaseNameNoExt(const std::string& name_or_path)
{
    size_t slash = name_or_path.find_last_of('/');
    size_t start = (slash == std::string::npos) ? 0 : slash + 1;
    size_t dot = name_or_path.find_last_of('.');
    size_t end = (dot == std::string::npos || dot < start) ? name_or_path.size() : dot;
    return name_or_path.substr(start, end - start);
}

static std::string MsToTimeString(int64_t ms)
{
    if (ms <= 0) {
        return "00:00";
    }

    int64_t total_sec = ms / 1000;
    int sec  = static_cast<int>(total_sec % 60);
    int min  = static_cast<int>((total_sec / 60) % 60);
    int hour = static_cast<int>(total_sec / 3600);

    char buf[32];

    if (hour > 0) {
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hour, min, sec);
    } else {
        snprintf(buf, sizeof(buf), "%02d:%02d", min, sec);
    }

    return std::string(buf);
}

// Chuẩn hóa chuỗi cho tìm kiếm:
// - lower ASCII
// - gom ' ', '_', '-', '.', '/', '\\' thành 1 khoảng trắng
// - giữ nguyên byte UTF-8 đa byte
static std::string NormalizeForSearch(const std::string& s)
{
    std::string lower = ToLowerAscii(s);
    std::string out;
    out.reserve(lower.size());

    bool last_space = false;

    for (char ch : lower) {
        unsigned char c = static_cast<unsigned char>(ch);

        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
            out.push_back(ch);
            last_space = false;
        } else if (c < 128 && (ch == ' ' || ch == '_' || ch == '-' ||
                               ch == '.' || ch == '/' || ch == '\\')) {
            if (!last_space) {
                out.push_back(' ');
                last_space = true;
            }
        } else {
            out.push_back(ch);
            last_space = false;
        }
    }

    while (!out.empty() && out.front() == ' ') out.erase(out.begin());
    while (!out.empty() && out.back() == ' ')  out.pop_back();

    return out;
}

// Kiểm tra đuôi file (ví dụ ".mp3", ".wav", ...)
static bool HasExtension(const std::string& path, const char* ext)
{
    if (!ext) return false;
    std::string low = ToLowerAscii(path);
    size_t len_ext  = strlen(ext);
    if (low.size() < len_ext) return false;
    return memcmp(low.data() + low.size() - len_ext, ext, len_ext) == 0;
}

// Định dạng audio hỗ trợ trên SD
enum class SdAudioFormat : uint8_t {
    Unknown = 0,
    Mp3,
    Wav,
    Aac,
    Flac,
    Ogg,
    Opus
};

static SdAudioFormat DetectAudioFormat(const std::string& path)
{
    if (HasExtension(path, ".mp3"))  return SdAudioFormat::Mp3;
    if (HasExtension(path, ".wav"))  return SdAudioFormat::Wav;
    if (HasExtension(path, ".aac") ||
        HasExtension(path, ".m4a")) return SdAudioFormat::Aac;
    if (HasExtension(path, ".flac")) return SdAudioFormat::Flac;
    if (HasExtension(path, ".ogg"))  return SdAudioFormat::Ogg;
    if (HasExtension(path, ".opus")) return SdAudioFormat::Opus;
    return SdAudioFormat::Unknown;
}

// Parse header WAV PCM 16-bit đơn giản (RIFF/WAVE/fmt/data)
static bool ParseWavHeader(FILE* fp,
                           int& sample_rate,
                           int& channels,
                           size_t& data_offset,
                           size_t& data_size)
{
    if (!fp) return false;

    uint8_t header[44];
    if (fread(header, 1, sizeof(header), fp) != sizeof(header)) {
        return false;
    }

    if (memcmp(header, "RIFF", 4) != 0 ||
        memcmp(header + 8, "WAVE", 4) != 0) {
        return false;
    }

    if (memcmp(header + 12, "fmt ", 4) != 0) {
        return false;
    }

    uint16_t audio_format = header[20] | (header[21] << 8);
    uint16_t num_channels = header[22] | (header[23] << 8);
    uint32_t sampleRate   = header[24] |
                             (header[25] << 8) |
                             (header[26] << 16) |
                             (header[27] << 24);
    uint16_t bitsPerSample = header[34] | (header[35] << 8);

    if (audio_format != 1 || bitsPerSample != 16 ||
        num_channels == 0 || sampleRate == 0) {
        // Chỉ hỗ trợ PCM 16-bit
        return false;
    }

    if (memcmp(header + 36, "data", 4) != 0) {
        // Layout phức tạp hơn (chunk khác) không hỗ trợ ở đây
        return false;
    }

    uint32_t dataSize = header[40] |
                        (header[41] << 8) |
                        (header[42] << 16) |
                        (header[43] << 24);

    sample_rate = (int)sampleRate;
    channels    = (int)num_channels;
    data_offset = sizeof(header);
    data_size   = dataSize;

    return true;
}

// Score cho chế độ gợi ý (tên tương tự + cùng thư mục + tần suất phát)
static int ComputeTrackScoreForBase(const Esp32SdMusic::TrackInfo& base,
                                    const Esp32SdMusic::TrackInfo& cand,
                                    uint32_t cand_play_count)
{
    int score = 0;

    std::string base_dir = ExtractDirectory(base.path);
    std::string cand_dir = ExtractDirectory(cand.path);
    if (!base_dir.empty() && base_dir == cand_dir) {
        score += 3;
    }

    std::string base_name = ExtractBaseNameNoExt(base.name.empty() ? base.path : base.name);
    std::string cand_name = ExtractBaseNameNoExt(cand.name.empty() ? cand.path : cand.name);

    base_name = ToLowerAscii(base_name);
    cand_name = ToLowerAscii(cand_name);

    if (!base_name.empty() && !cand_name.empty()) {
        if (cand_name.find(base_name) != std::string::npos ||
            base_name.find(cand_name) != std::string::npos)
        {
            score += 3;
        } else {
            auto b_space = base_name.find(' ');
            auto c_space = cand_name.find(' ');
            std::string b_first = base_name.substr(0, b_space);
            std::string c_first = cand_name.substr(0, c_space);
            if (!b_first.empty() && b_first == c_first) {
                score += 1;
            }
        }
    }

    if (cand_play_count > 0) {
        score += static_cast<int>(cand_play_count);
    }

    return score;
}

// ================================================================
//  BẢNG TRA GENRE ID3v1
// ================================================================
static const char* kId3v1Genres[] = {
    "Blues", "Classic Rock", "Country", "Dance", "Disco", "Funk", "Grunge",
    "Hip-Hop", "Jazz", "Metal", "New Age", "Oldies", "Other", "Pop", "R&B",
    "Rap", "Reggae", "Rock", "Techno", "Industrial", "Alternative", "Ska",
    "Death Metal", "Pranks", "Soundtrack", "Euro-Techno", "Ambient",
    "Trip-Hop", "Vocal", "Jazz+Funk", "Fusion", "Trance", "Classical",
    "Instrumental", "Acid", "House", "Game", "Sound Clip", "Gospel",
    "Noise", "AlternRock", "Bass", "Soul", "Punk", "Space", "Meditative",
    "Instrumental Pop", "Instrumental Rock", "Ethnic", "Gothic", "Darkwave",
    "Techno-Industrial", "Electronic", "Pop-Folk", "Eurodance", "Dream",
    "Southern Rock", "Comedy", "Cult", "Gangsta", "Top 40", "Christian Rap",
    "Pop/Funk", "Jungle", "Native American", "Cabaret", "New Wave",
    "Psychadelic", "Rave", "Showtunes", "Trailer", "Lo-Fi", "Tribal",
    "Acid Punk", "Acid Jazz", "Polka", "Retro", "Musical", "Rock & Roll",
    "Hard Rock"
};

static const char* Id3v1GenreNameFromIndex(int idx)
{
    if (idx < 0) return nullptr;
    int count = sizeof(kId3v1Genres) / sizeof(kId3v1Genres[0]);
    if (idx >= count) return nullptr;
    return kId3v1Genres[idx];
}

// ================================================================
//  Đọc ID3v1 (cuối file) — rất nhẹ, không dính tới ID3v2
// ================================================================
static void ReadId3v1(const std::string& path,
                      Esp32SdMusic::TrackInfo& info)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return;

    if (fseek(f, -128, SEEK_END) != 0) {
        fclose(f);
        return;
    }

    uint8_t tag[128];
    if (fread(tag, 1, 128, f) != 128) {
        fclose(f);
        return;
    }
    fclose(f);

    if (memcmp(tag, "TAG", 3) != 0) {
        return;
    }

    auto trim = [](const char* p, size_t n) -> std::string {
        std::string s(p, n);
        while (!s.empty() && (s.back() == ' ' || s.back() == '\0')) s.pop_back();
        return s;
    };

    std::string title   = trim((char*)tag + 3, 30);
    std::string artist  = trim((char*)tag + 33, 30);
    std::string album   = trim((char*)tag + 63, 30);
    std::string year    = trim((char*)tag + 93, 4);
    std::string comment = trim((char*)tag + 97, 28);
    uint8_t     track   = tag[126]; // ID3v1.1

    if (!title.empty()   && info.title.empty())   info.title   = title;
    if (!artist.empty()  && info.artist.empty())  info.artist  = artist;
    if (!album.empty()   && info.album.empty())   info.album   = album;
    if (!year.empty()    && info.year.empty())    info.year    = year;
    if (!comment.empty() && info.comment.empty()) info.comment = comment;

    if (info.track_number == 0 && track != 0) {
        info.track_number = track;
    }

    uint8_t genre_idx = tag[127];
    if (info.genre.empty() && genre_idx != 0xFF) {
        const char* gname = Id3v1GenreNameFromIndex((int)genre_idx);
        if (gname) {
            info.genre = gname;  // Pop, Rock,...
        } else {
            info.genre = std::to_string((int)genre_idx); // fallback
        }
    }
}

// ================================================================
//   Đọc ID3v2 SAFETY MODE (không load toàn bộ header vào RAM)
//   Chỉ đọc TIT2 (title), TPE1 (artist), TALB (album), TYER (year), TCON (genre)
// ================================================================
static std::string Utf16ToUtf8(const uint8_t* data, size_t len, bool big_endian)
{
    std::string out;
    out.reserve(len);

    for (size_t i = 0; i + 1 < len; i += 2) {
        uint16_t ch;
        if (!big_endian)
            ch = data[i] | (data[i + 1] << 8);
        else
            ch = (data[i] << 8) | data[i + 1];

        // Basic UTF-16 (không xử lý surrogate vì ID3 ít dùng)
        if (ch < 0x80) {
            out.push_back((char)ch);
        } else if (ch < 0x800) {
            out.push_back((char)(0xC0 | (ch >> 6)));
            out.push_back((char)(0x80 | (ch & 0x3F)));
        } else {
            out.push_back((char)(0xE0 | (ch >> 12)));
            out.push_back((char)(0x80 | ((ch >> 6) & 0x3F)));
            out.push_back((char)(0x80 | (ch & 0x3F)));
        }
    }

    return out;
}

static std::string TrimNull(const std::string& s)
{
    size_t end = s.find('\0');
    if (end == std::string::npos) return s;
    return s.substr(0, end);
}

// Chuẩn hóa giá trị TCON (genre ID3v2)
static std::string NormalizeTcon(const std::string& raw)
{
    std::string s = TrimNull(raw);

    // Trim space đầu/cuối
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();

    // Dạng "(13)" hoặc "(13)Pop" → map 13 → tên nếu có
    if (!s.empty() && s.front() == '(') {
        size_t close = s.find(')');
        if (close != std::string::npos && close > 1) {
            std::string num = s.substr(1, close - 1);
            int idx = atoi(num.c_str());
            const char* gname = Id3v1GenreNameFromIndex(idx);
            if (gname) {
                return std::string(gname);
            }
        }
    }

    return s;
}

static void ReadId3v2_Safe(const std::string& path,
                           Esp32SdMusic::TrackInfo& info)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return;

    uint8_t hdr[10];
    if (fread(hdr, 1, 10, f) != 10) {
        fclose(f);
        return;
    }

    if (memcmp(hdr, "ID3", 3) != 0) {
        fclose(f);
        return;
    }

    uint32_t tag_size =
        ((hdr[6] & 0x7F) << 21) |
        ((hdr[7] & 0x7F) << 14) |
        ((hdr[8] & 0x7F) << 7)  |
         (hdr[9] & 0x7F);

    uint32_t pos = 10;
    uint32_t end = 10 + tag_size;

    auto readFrame = [&](const char* id) -> std::string {
        uint32_t cur = pos;

        while (cur + 10 <= end) {
            fseek(f, cur, SEEK_SET);

            uint8_t frame_hdr[10];
            if (fread(frame_hdr, 1, 10, f) != 10) break;

            if (frame_hdr[0] == 0) break;

            uint32_t frame_size =
                (frame_hdr[4] << 24) |
                (frame_hdr[5] << 16) |
                (frame_hdr[6] << 8)  |
                 frame_hdr[7];

            if (frame_size == 0) break;

            if (memcmp(frame_hdr, id, 4) == 0) {
                if (frame_size < 2 || frame_size > 2048) return "";

                std::vector<uint8_t> buf(frame_size);
                if (fread(buf.data(), 1, frame_size, f) != frame_size)
                    return "";

                uint8_t enc = buf[0];
                const uint8_t* p = &buf[1];
                size_t plen = frame_size - 1;

                std::string raw;

                switch (enc) {
                    case 0: // ISO-8859-1
                        raw = std::string((char*)p, plen);
                        break;

                    case 3: // UTF-8
                        raw = std::string((char*)p, plen);
                        break;

                    case 1: { // UTF-16 with BOM
                        if (plen < 2) return "";
                        bool big_endian = !(p[0] == 0xFF && p[1] == 0xFE);
                        size_t start = 2;
                        raw = Utf16ToUtf8(p + start, plen - start, big_endian);
                        break;
                    }

                    case 2: // UTF-16BE no BOM
                        raw = Utf16ToUtf8(p, plen, true);
                        break;

                    default:
                        return "";
                }

                return TrimNull(raw);
            }

            cur += 10 + frame_size;
            if (cur >= end) break;
        }
        return "";
    };

    std::string t, a, b, y, g;

    t = readFrame("TIT2");
    if (!t.empty()) info.title = t;

    a = readFrame("TPE1");
    if (!a.empty()) info.artist = a;

    b = readFrame("TALB");
    if (!b.empty()) info.album = b;

    y = readFrame("TYER");
    if (!y.empty()) info.year = y;

    g = readFrame("TCON");
    if (!g.empty()) {
        info.genre = NormalizeTcon(g);
    }

    fclose(f);
}

// Escape chuỗi sang JSON string
static std::string JsonEscape(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (unsigned char c : in) {
        switch (c) {
        case '\"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b";  break;
        case '\f': out += "\\f";  break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:
            if (c < 0x20) {
                char buf[7];
                snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out.push_back((char)c);
            }
        }
    }
    return out;
}

// ============================================================================
//                         PART 1 / 3
//      CTOR / DTOR / PLAYLIST / THƯ MỤC / ĐẾM BÀI / CHIA TRANG
// ============================================================================

Esp32SdMusic::Esp32SdMusic()
    : root_directory_(),
      playlist_(),
      playlist_mutex_(),
      current_index_(-1),
      play_count_(),
      playback_thread_(),
      stop_requested_(false),
      pause_requested_(false),
      state_(PlayerState::Stopped),
      state_mutex_(),
      state_cv_(),
      shuffle_enabled_(false),
      repeat_mode_(RepeatMode::None),
      current_play_time_ms_(0),
      total_duration_ms_(0),
      final_pcm_data_fft_(nullptr),
      mp3_decoder_(nullptr),
      mp3_decoder_initialized_(false),
      mp3_frame_info_{},
      genre_playlist_(),
      genre_current_pos_(-1),
      genre_current_key_(),
      history_mutex_(),
      play_history_indices_()
{
}

Esp32SdMusic::~Esp32SdMusic()
{
    ESP_LOGI(TAG, "Destroying SD music module");

    stop();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_requested_  = true;
        pause_requested_ = false;
        state_cv_.notify_all();
    }

    joinPlaybackThreadWithTimeout();
    cleanupMp3Decoder();

    auto display = Board::GetInstance().GetDisplay();
    if (display && final_pcm_data_fft_) {
        display->ReleaseAudioBuffFFT(final_pcm_data_fft_);
        final_pcm_data_fft_ = nullptr;
    }

    ESP_LOGI(TAG, "SD music module destroyed");
}

void Esp32SdMusic::Initialize(class SdCard* sd_card) {
    sd_card_ = sd_card;
    if (sd_card_ && sd_card_->IsMounted()) {
        root_directory_ = sd_card_->GetMountPoint();
    } else {
        ESP_LOGW(TAG, "SD card not mounted yet — will retry later");
    }
}

void Esp32SdMusic::joinPlaybackThreadWithTimeout()
{
    if (!playback_thread_.joinable()) return;

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(120);

    while (playback_thread_.joinable() &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (playback_thread_.joinable()) {
        ESP_LOGE(TAG, "Thread stuck → force detach()");
        playback_thread_.detach();
    }
}

// Playlist loading — sử dụng playlist.json
// Nếu chưa có / hỏng / rỗng → quét lại và lưu playlist.json
// Nếu file đã có nội dung hợp lệ → chỉ đọc, không quét
bool Esp32SdMusic::loadTrackList()
{
    std::vector<TrackInfo> list;

    if (root_directory_.empty()) {
        if (sd_card_ == nullptr || !sd_card_->IsMounted()) {
            ESP_LOGE(TAG, "loadTrackList: SD not mounted");
            return false;
        }
        root_directory_ = sd_card_->GetMountPoint();
    }

    // Playlist theo thư mục gốc hiện tại
    // Ví dụ: /sdcard/playlist.json hoặc /sdcard/Music/playlist.json
    std::string playlist_path = root_directory_ + "/playlist.json";

    bool loaded = loadPlaylistFromFile(playlist_path, list);
    if (!loaded || list.empty()) {
        ESP_LOGW(TAG,
                 "playlist.json missing/empty/invalid, scanning SD to rebuild: %s",
                 root_directory_.c_str());

        ESP_LOGI(TAG, "Scanning SD card: %s", root_directory_.c_str());
        scanDirectoryRecursive(root_directory_, list);

        // Lưu playlist.json (kể cả khi list rỗng, coi như playlist trống)
        if (!savePlaylistToFile(playlist_path, list)) {
            ESP_LOGE(TAG, "Failed to save playlist.json: %s", playlist_path.c_str());
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        playlist_.swap(list);
        current_index_ = playlist_.empty() ? -1 : 0;
        play_count_.assign(playlist_.size(), 0);
    }

    {
        std::lock_guard<std::mutex> hlock(history_mutex_);
        play_history_indices_.clear();
    }

    ESP_LOGI(TAG, "Track list ready from playlist.json: %u tracks",
             (unsigned)playlist_.size());
    return !playlist_.empty();
}

size_t Esp32SdMusic::getTotalTracks() const
{
    std::lock_guard<std::mutex> lock(playlist_mutex_);
    return playlist_.size();
}

std::vector<Esp32SdMusic::TrackInfo> Esp32SdMusic::listTracks() const
{
    std::lock_guard<std::mutex> lock(playlist_mutex_);
    return playlist_;
}

Esp32SdMusic::TrackInfo Esp32SdMusic::getTrackInfo(int index) const
{
    std::lock_guard<std::mutex> lock(playlist_mutex_);
    if (index < 0 || index >= (int)playlist_.size()) return {};
    return playlist_[index];
}

// Gom code build path + resolve FAT short / case-insensitive
bool Esp32SdMusic::resolveDirectoryRelative(const std::string& relative_dir,
                                            std::string& out_full)
{
    if (sd_card_ == nullptr || !sd_card_->IsMounted()) {
        ESP_LOGE(TAG, "resolveDirectoryRelative: SD not mounted");
        return false;
    }

    std::string mount = sd_card_->GetMountPoint();
    std::string full;

    if (relative_dir.empty() || relative_dir == "/") {
        full = mount;
    } else if (relative_dir[0] == '/') {
        full = mount + relative_dir;
    } else {
        full = mount + "/" + relative_dir;
    }

    full = resolveLongName(full);
    full = resolveCaseInsensitiveDir(full);

    struct stat st{};
    if (stat(full.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        ESP_LOGE(TAG, "Invalid directory: %s", full.c_str());
        return false;
    }

    out_full = full;
    return true;
}

bool Esp32SdMusic::setDirectory(const std::string& relative_dir)
{
    std::string full;
    if (!resolveDirectoryRelative(relative_dir, full)) {
        ESP_LOGE(TAG, "setDirectory: cannot resolve %s", relative_dir.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        root_directory_ = full;
    }

    ESP_LOGI(TAG, "Directory selected: %s", full.c_str());
    return loadTrackList();
}

bool Esp32SdMusic::playDirectory(const std::string& relative_dir)
{
    ESP_LOGI(TAG, "Request to play directory: %s", relative_dir.c_str());
    if (!setDirectory(relative_dir)) {
        ESP_LOGE(TAG, "playDirectory: cannot set directory: %s", relative_dir.c_str());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (playlist_.empty()) {
            ESP_LOGE(TAG, "playDirectory: directory is empty: %s", relative_dir.c_str());
            return false;
        }
        current_index_ = 0;
        ESP_LOGI(TAG, "playDirectory: start track #0: %s",
                 playlist_[0].name.c_str());
    }
    return play();
}

// Tìm index theo keyword (tên hoặc path)
int Esp32SdMusic::findTrackIndexByKeyword(const std::string& keyword) const
{
    if (keyword.empty()) return -1;

    std::string kw = NormalizeForSearch(keyword);
    if (kw.empty()) return -1;

    std::lock_guard<std::mutex> lock(playlist_mutex_);

    for (int i = 0; i < (int)playlist_.size(); ++i) {
        std::string name_norm = NormalizeForSearch(playlist_[i].name);
        std::string path_norm = NormalizeForSearch(playlist_[i].path);

        if ((!name_norm.empty() && name_norm.find(kw) != std::string::npos) ||
            (!path_norm.empty() && path_norm.find(kw) != std::string::npos)) {
            return i;
        }
    }
    return -1;
}

bool Esp32SdMusic::playByName(const std::string& keyword)
{
    if (keyword.empty()) {
        ESP_LOGW(TAG, "playByName(): empty keyword");
        return false;
    }

    bool need_reload = false;
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (playlist_.empty()) {
            ESP_LOGW(TAG, "playByName(): playlist empty — reloading");
            need_reload = true;
        }
    }

    if (need_reload) {
        if (!loadTrackList()) {
            ESP_LOGE(TAG, "playByName(): Cannot load playlist");
            return false;
        }
    }

    int found_index = findTrackIndexByKeyword(keyword);
    if (found_index < 0) {
        ESP_LOGW(TAG, "playByName(): no match for '%s'", keyword.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (found_index < 0 || found_index >= (int)playlist_.size()) {
            return false;
        }
        current_index_ = found_index;
        ESP_LOGI(TAG, "playByName(): matched track #%d → %s",
                 found_index, playlist_[found_index].name.c_str());
    }

    return play();
}

std::string Esp32SdMusic::getCurrentTrack() const
{
    std::lock_guard<std::mutex> lock(playlist_mutex_);
    if (current_index_ < 0 || current_index_ >= (int)playlist_.size()) return "";
    return playlist_[current_index_].name;
}

std::string Esp32SdMusic::getCurrentTrackPath() const
{
    std::lock_guard<std::mutex> lock(playlist_mutex_);
    if (current_index_ < 0 || current_index_ >= (int)playlist_.size()) return "";
    return playlist_[current_index_].path;
}

std::vector<std::string> Esp32SdMusic::listDirectories() const
{
    std::vector<std::string> dirs;

    DIR* d = opendir(root_directory_.c_str());
    if (!d) {
        ESP_LOGE(TAG, "Cannot open directory: %s", root_directory_.c_str());
        return dirs;
    }

    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        std::string name = ent->d_name;

        if (name == "." || name == "..")
            continue;

        std::string full = root_directory_ + "/" + name;

        struct stat st{};
        if (stat(full.c_str(), &st) != 0)
            continue;

        if (S_ISDIR(st.st_mode)) {
            dirs.push_back(name);
        }
    }

    closedir(d);
    return dirs;
}

std::vector<Esp32SdMusic::TrackInfo>
Esp32SdMusic::searchTracks(const std::string& keyword) const
{
    std::vector<TrackInfo> results;
    if (keyword.empty()) return results;

    std::string kw = NormalizeForSearch(keyword);
    if (kw.empty()) return results;

    std::lock_guard<std::mutex> lock(playlist_mutex_);
    for (const auto& t : playlist_) {
        std::string name_norm = NormalizeForSearch(t.name);
        std::string path_norm = NormalizeForSearch(t.path);

        if ((!name_norm.empty() && name_norm.find(kw) != std::string::npos) ||
            (!path_norm.empty() && path_norm.find(kw) != std::string::npos)) {
            results.push_back(t);
        }
    }
    return results;
}

std::vector<std::string> Esp32SdMusic::listGenres() const
{
    std::vector<std::string> genres;
    std::unordered_set<std::string> uniq;

    std::lock_guard<std::mutex> lock(playlist_mutex_);
    for (const auto &t : playlist_) {
        if (t.genre.empty()) continue;
        if (uniq.insert(t.genre).second) {
            genres.push_back(t.genre);
        }
    }

    std::sort(genres.begin(), genres.end(),
              [](const std::string& a, const std::string& b) {
                  return ToLowerAscii(a) < ToLowerAscii(b);
              });

    return genres;
}

std::string Esp32SdMusic::resolveCaseInsensitiveDir(const std::string& path)
{
    size_t pos = path.find_last_of('/');
    if (pos == std::string::npos)
        return path;

    std::string parent = path.substr(0, pos);
    std::string name   = path.substr(pos + 1);

    DIR* d = opendir(parent.c_str());
    if (!d) {
        return path;
    }

    std::string lowerName = ToLowerAscii(name);

    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        std::string entry = ent->d_name;

        if (entry == "." || entry == "..")
            continue;

        std::string lowerEntry = ToLowerAscii(entry);

        if (lowerEntry == lowerName) {
            std::string full = parent + "/" + entry;

            struct stat st{};
            if (stat(full.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
                closedir(d);
                return full;
            }
        }
    }

    closedir(d);
    return path;
}

bool Esp32SdMusic::setTrack(int index)
{
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (index < 0 || index >= (int)playlist_.size()) {
            ESP_LOGE(TAG, "setTrack: index %d out of range", index);
            return false;
        }
        current_index_ = index;
        ESP_LOGI(TAG, "Switching to track #%d: %s",
                 index, playlist_[index].name.c_str());
    }
    return play();
}

void Esp32SdMusic::scanDirectoryRecursive(
    const std::string& dir,
    std::vector<TrackInfo>& out)
{
    DIR* d = opendir(dir.c_str());
    if (!d) {
        ESP_LOGE(TAG, "Cannot open directory: %s", dir.c_str());
        return;
    }

    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        std::string name_utf8 = ent->d_name;

        if (name_utf8 == "." || name_utf8 == "..")
            continue;

        std::string full = dir + "/" + name_utf8;

        struct stat st{};
        if (stat(full.c_str(), &st) != 0) {
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            scanDirectoryRecursive(full, out);
            continue;
        }

        SdAudioFormat fmt = DetectAudioFormat(full);
        if (fmt == SdAudioFormat::Unknown)
            continue;

        TrackInfo t;
        t.path      = full;
        t.file_size = st.st_size;

        // ID3v2 (ưu tiên) → nếu thiếu fallback ID3v1
        ReadId3v2_Safe(full, t);
        ReadId3v1(full, t);

        // Tên hiển thị: ưu tiên title, fallback tên file (không extension)
        if (!t.title.empty())
            t.name = t.title;
        else
            t.name = ExtractBaseNameNoExt(name_utf8);

        out.push_back(std::move(t));
    }

    closedir(d);
}

// Rebuild playlist theo yêu cầu người dùng (MCP gọi hàm này)
// Bỏ qua nội dung playlist.json hiện tại, luôn quét lại và ghi đè
bool Esp32SdMusic::rebuildPlaylistFromSd()
{
    std::vector<TrackInfo> list;

    if (root_directory_.empty()) {
        if (sd_card_ == nullptr || !sd_card_->IsMounted()) {
            ESP_LOGE(TAG, "rebuildPlaylistFromSd: SD not mounted");
            return false;
        }
        root_directory_ = sd_card_->GetMountPoint();
    }

    ESP_LOGI(TAG, "Rebuilding playlist by scanning directory: %s",
             root_directory_.c_str());

    scanDirectoryRecursive(root_directory_, list);

    std::string playlist_path = root_directory_ + "/playlist.json";
    if (!savePlaylistToFile(playlist_path, list)) {
        ESP_LOGE(TAG, "Failed to save playlist.json after rebuild");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        playlist_.swap(list);
        current_index_ = playlist_.empty() ? -1 : 0;
        play_count_.assign(playlist_.size(), 0);
    }

    {
        std::lock_guard<std::mutex> hlock(history_mutex_);
        play_history_indices_.clear();
    }

    ESP_LOGI(TAG, "Playlist rebuilt: %u tracks",
             (unsigned)playlist_.size());
    return !playlist_.empty();
}

// Playlist JSON: lưu đủ metadata cần thiết để search/tìm bài (không lưu ảnh bìa)
bool Esp32SdMusic::savePlaylistToFile(const std::string& playlist_path,
                                      const std::vector<TrackInfo>& list) const
{
    FILE* fp = fopen(playlist_path.c_str(), "wb");
    if (!fp) {
        ESP_LOGE(TAG, "Cannot create playlist file: %s", playlist_path.c_str());
        return false;
    }

    // Header JSON
    fprintf(fp, "{\n");
    fprintf(fp, "  \"version\": 1,\n");
    fprintf(fp, "  \"tracks\": [\n");

    size_t written = 0;

    for (size_t i = 0; i < list.size(); ++i) {
        const auto& t = list[i];

        // Chuẩn bị chuỗi đã escape
        std::string name    = JsonEscape(t.name.empty()
                                         ? ExtractBaseNameNoExt(t.path)
                                         : t.name);
        std::string path    = JsonEscape(t.path);
        std::string title   = JsonEscape(t.title);
        std::string artist  = JsonEscape(t.artist);
        std::string album   = JsonEscape(t.album);
        std::string genre   = JsonEscape(t.genre);
        std::string comment = JsonEscape(t.comment);
        std::string year    = JsonEscape(t.year);

        int track_number = t.track_number;
        int duration_ms  = t.duration_ms;
        int bitrate      = t.bitrate_kbps;
        unsigned long long file_size = (unsigned long long)t.file_size;

        // Bắt đầu object
        fprintf(fp, "    {\n");
        fprintf(fp, "      \"name\": \"%s\",\n",    name.c_str());
        fprintf(fp, "      \"path\": \"%s\",\n",    path.c_str());
        fprintf(fp, "      \"title\": \"%s\",\n",   title.c_str());
        fprintf(fp, "      \"artist\": \"%s\",\n",  artist.c_str());
        fprintf(fp, "      \"album\": \"%s\",\n",   album.c_str());
        fprintf(fp, "      \"genre\": \"%s\",\n",   genre.c_str());
        fprintf(fp, "      \"comment\": \"%s\",\n", comment.c_str());
        fprintf(fp, "      \"year\": \"%s\",\n",    year.c_str());
        fprintf(fp, "      \"track_number\": %d,\n",   track_number);
        fprintf(fp, "      \"duration_ms\": %d,\n",    duration_ms);
        fprintf(fp, "      \"bitrate_kbps\": %d,\n",   bitrate);
        fprintf(fp, "      \"file_size\": %llu\n",     file_size);
        fprintf(fp, "    }%s\n", (i + 1 < list.size()) ? "," : "");

        ++written;
    }

    fprintf(fp, "  ]\n");
    fprintf(fp, "}\n");
    fclose(fp);

    ESP_LOGI(TAG, "Playlist file (JSON) saved: %s (%u tracks)",
             playlist_path.c_str(), (unsigned)written);
    // Cho phép playlist rỗng (0 track) vẫn coi là thành công
    return true;
}

bool Esp32SdMusic::loadPlaylistFromFile(const std::string& playlist_path,
                                        std::vector<TrackInfo>& out) const
{
    FILE* fp = fopen(playlist_path.c_str(), "rb");
    if (!fp) {
        ESP_LOGW(TAG, "Playlist file not found: %s", playlist_path.c_str());
        return false;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return false;
    }
    long sz = ftell(fp);
    if (sz <= 0) {
        fclose(fp);
        ESP_LOGW(TAG, "Playlist file is empty: %s", playlist_path.c_str());
        return false;
    }
    fseek(fp, 0, SEEK_SET);

    char* buf = (char*) heap_caps_malloc(sz + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "Not enough memory to read playlist file (%ld bytes)", sz);
        fclose(fp);
        return false;
    }

    size_t read_bytes = fread(buf, 1, sz, fp);
    fclose(fp);
    buf[read_bytes] = '\0';

    cJSON* root = cJSON_Parse(buf);
    heap_caps_free(buf);

    if (!root) {
        ESP_LOGE(TAG, "Failed to parse playlist.json as JSON");
        return false;
    }

    cJSON* tracks = cJSON_GetObjectItem(root, "tracks");
    if (!cJSON_IsArray(tracks)) {
        ESP_LOGE(TAG, "playlist.json has no 'tracks' array");
        cJSON_Delete(root);
        return false;
    }

    out.clear();
    cJSON* item = nullptr;
    cJSON_ArrayForEach(item, tracks) {
        if (!cJSON_IsObject(item)) {
            continue;
        }

        auto getStr = [&](const char* key) -> std::string {
            cJSON* v = cJSON_GetObjectItem(item, key);
            if (cJSON_IsString(v) && v->valuestring) {
                return std::string(v->valuestring);
            }
            return std::string();
        };

        auto getInt = [&](const char* key) -> int {
            cJSON* v = cJSON_GetObjectItem(item, key);
            if (cJSON_IsNumber(v)) {
                return v->valueint;
            }
            return 0;
        };

        auto getSizeT = [&](const char* key) -> size_t {
            cJSON* v = cJSON_GetObjectItem(item, key);
            if (cJSON_IsNumber(v)) {
                return (size_t)v->valuedouble;
            }
            return 0;
        };

        TrackInfo t;

        t.name    = getStr("name");
        t.path    = getStr("path");
        t.title   = getStr("title");
        t.artist  = getStr("artist");
        t.album   = getStr("album");
        t.genre   = getStr("genre");
        t.comment = getStr("comment");
        t.year    = getStr("year");

        if (t.path.empty()) {
            continue;
        }

        if (t.name.empty()) {
            t.name = ExtractBaseNameNoExt(t.path);
        }

        t.track_number = getInt("track_number");
        t.duration_ms  = getInt("duration_ms");
        t.bitrate_kbps = getInt("bitrate_kbps");
        t.file_size    = getSizeT("file_size");

        // Không dùng cover_* trong playlist (luôn 0 / rỗng)
        t.cover_size = 0;
        t.cover_mime.clear();

        out.push_back(std::move(t));
    }

    cJSON_Delete(root);

    if (out.empty()) {
        ESP_LOGW(TAG, "Playlist JSON has no valid tracks: %s",
                 playlist_path.c_str());
        return false;
    }

    ESP_LOGI(TAG, "Loaded %u tracks from playlist.json: %s",
             (unsigned)out.size(), playlist_path.c_str());
    return true;
}

std::string Esp32SdMusic::resolveLongName(const std::string& path)
{
    // Không xử lý short-name 8.3 → trả nguyên đường dẫn
    return path;
}

int Esp32SdMusic::findNextTrackIndex(int start, int direction)
{
    if (playlist_.empty()) return -1;
    int count = static_cast<int>(playlist_.size());
    if (start < 0 || start >= count)
        return 0;
    int result = (start + direction + count) % count;
    return result;
}

size_t Esp32SdMusic::countTracksInDirectory(const std::string& relative_dir)
{
    std::string full;
    if (!resolveDirectoryRelative(relative_dir, full)) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(playlist_mutex_);
    if (playlist_.empty()) {
        return 0;
    }

    std::string prefix = full;
    if (!prefix.empty() && prefix.back() != '/') {
        prefix += '/';
    }

    size_t count = 0;
    for (const auto& t : playlist_) {
        if (t.path.compare(0, prefix.size(), prefix) == 0) {
            ++count;
        }
    }
    return count;
}

size_t Esp32SdMusic::countTracksInCurrentDirectory() const
{
    std::lock_guard<std::mutex> lock(playlist_mutex_);
    return playlist_.size();
}

std::vector<Esp32SdMusic::TrackInfo>
Esp32SdMusic::listTracksPage(size_t page_index, size_t page_size) const
{
    std::vector<TrackInfo> result;
    if (page_size == 0) return result;

    std::lock_guard<std::mutex> lock(playlist_mutex_);
    if (playlist_.empty()) return result;

    size_t start = page_index * page_size;
    if (start >= playlist_.size()) return result;

    size_t end = std::min(start + page_size, playlist_.size());
    result.reserve(end - start);
    for (size_t i = start; i < end; ++i) {
        result.push_back(playlist_[i]);
    }
    return result;
}

// ============================================================================
//                         PART 2 / 3
//      SHUFFLE / REPEAT / PLAY-PAUSE-STOP / THREAD / DECODE
// ============================================================================

void Esp32SdMusic::shuffle(bool enabled)
{
    shuffle_enabled_ = enabled;
    ESP_LOGI(TAG, "Shuffle: %s", enabled ? "ON" : "OFF");
}

void Esp32SdMusic::repeat(RepeatMode mode)
{
    repeat_mode_ = mode;
    const char* mode_str =
        (mode == RepeatMode::None)      ? "None" :
        (mode == RepeatMode::RepeatOne) ? "RepeatOne" : "RepeatAll";
    ESP_LOGI(TAG, "Repeat mode = %s", mode_str);
}

bool Esp32SdMusic::IsPlaying() const
{
    PlayerState st = state_.load();
    return (st != PlayerState::Stopped &&
            st != PlayerState::Error);
}

bool Esp32SdMusic::play()
{
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);

        if (playlist_.empty()) {
            ESP_LOGW(TAG, "Playlist empty — reloading");
            loadTrackList();
            if (playlist_.empty()) {
                ESP_LOGE(TAG, "No audio files found on SD");
                return false;
            }
        }

        if (current_index_ < 0)
            current_index_ = 0;
    }

    if (state_.load() == PlayerState::Paused) {
        ESP_LOGI(TAG, "Resuming playback");
        pause_requested_ = false;
        state_.store(PlayerState::Playing);
        state_cv_.notify_all();
        return true;
    }

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        stop_requested_  = true;
        pause_requested_ = false;
        state_cv_.notify_all();
    }

    joinPlaybackThreadWithTimeout();

    auto& app = Application::GetInstance();
    app.StopListening();
    app.GetAudioService().EnableWakeWordDetection(false);
    app.SetDeviceState(kDeviceStateSpeaking);

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        stop_requested_  = false;
        pause_requested_ = false;
        state_.store(PlayerState::Preparing);
    }

    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size  = 8192;
    cfg.prio        = 5;
    cfg.thread_name = (char*)"sd_music_play";
    esp_pthread_set_cfg(&cfg);

    ESP_LOGI(TAG, "Starting playback thread");
    playback_thread_ = std::thread(&Esp32SdMusic::playbackThreadFunc, this);

    return true;
}

void Esp32SdMusic::pause()
{
    if (state_.load() == PlayerState::Playing) {
        ESP_LOGI(TAG, "Pausing playback");
        pause_requested_ = true;
    }
}

void Esp32SdMusic::stop()
{
    PlayerState st = state_.load();

    if (st == PlayerState::Stopped ||
        st == PlayerState::Error ||
        st == PlayerState::Preparing) {
        ESP_LOGW(TAG, "stop(): No SD music in progress to stop");
        return;
    }

    ESP_LOGI(TAG, "Stopping SD music playback");

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_requested_  = true;
        pause_requested_ = false;
        state_cv_.notify_all();
    }

    joinPlaybackThreadWithTimeout();

    state_.store(PlayerState::Stopped);
    current_play_time_ms_ = 0;

    ESP_LOGI(TAG, "SD music stopped successfully");
}

bool Esp32SdMusic::next()
{
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (playlist_.empty()) return false;
        if (shuffle_enabled_) {
            if (playlist_.size() > 1) {
                int new_i;
                do {
                    new_i = rand() % playlist_.size();
                } while (new_i == current_index_);
                current_index_ = new_i;
            }
        } else {
            current_index_ = findNextTrackIndex(current_index_, +1);
        }
        ESP_LOGI(TAG, "Next track → #%d: %s",
                 current_index_, playlist_[current_index_].name.c_str());
    }
    return play();
}

bool Esp32SdMusic::prev()
{
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (playlist_.empty()) return false;
        if (shuffle_enabled_) {
            if (playlist_.size() > 1) {
                int new_i;
                do {
                    new_i = rand() % playlist_.size();
                } while (new_i == current_index_);
                current_index_ = new_i;
            }
        } else {
            current_index_ = findNextTrackIndex(current_index_, -1);
        }
        ESP_LOGI(TAG, "Previous track → #%d: %s",
                 current_index_, playlist_[current_index_].name.c_str());
    }
    return play();
}

void Esp32SdMusic::recordPlayHistory(int index)
{
    if (index < 0) return;

    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        play_history_indices_.push_back(index);
        const size_t kMaxHistory = 200;
        if (play_history_indices_.size() > kMaxHistory) {
            size_t remove_count = play_history_indices_.size() - kMaxHistory;
            play_history_indices_.erase(
                play_history_indices_.begin(),
                play_history_indices_.begin() + remove_count);
        }
    }

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (index >= 0 && index < (int)play_count_.size()) {
            ++play_count_[index];
        }
    }
}

void Esp32SdMusic::playbackThreadFunc()
{
    TrackInfo track;
    int play_index = -1;
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);

        if (current_index_ < 0 || current_index_ >= (int)playlist_.size()) {
            ESP_LOGE(TAG, "Invalid current track index");
            state_.store(PlayerState::Error);
            return;
        }

        track      = playlist_[current_index_];
        play_index = current_index_;
    }

    recordPlayHistory(play_index);

    state_.store(PlayerState::Playing);
    ESP_LOGI(TAG, "Playback thread start: %s", track.path.c_str());
    current_play_time_ms_ = 0;
    total_duration_ms_    = 0;

    auto display = Board::GetInstance().GetDisplay();
    if (display) {
        std::string title  = !track.title.empty() ? track.title : track.name;
        std::string artist = track.artist;

        std::string line;
        if (!artist.empty()) {
            line = artist + " - " + title;
        } else {
            line = title;
        }

        display->SetMusicInfo(line.c_str());
        display->StartFFT();
    }

    initializeMp3Decoder();
    mp3_frame_info_ = {};

    bool ok = decodeAndPlayFile(track);
    cleanupMp3Decoder();

    if (display) {
        display->StopFFT();
        if (final_pcm_data_fft_) {
            display->ReleaseAudioBuffFFT(final_pcm_data_fft_);
            final_pcm_data_fft_ = nullptr;
        }
    }

    resetSampleRate();

    if (stop_requested_) {
        state_.store(PlayerState::Stopped);
        return;
    }

    if (!ok) {
        ESP_LOGW(TAG, "Playback error, stopping");
        state_.store(PlayerState::Error);
        return;
    }

    ESP_LOGI(TAG, "Playback finished normally: %s", track.name.c_str());

    // Nếu đang phát theo genre → ưu tiên chuyển bài tiếp theo trong genre
    if (!genre_playlist_.empty()) {
        if (playNextGenre()) return;
    }

    int next_index = -1;

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);

        if (playlist_.empty()) {
            state_.store(PlayerState::Stopped);
            return;
        }

        switch (repeat_mode_) {
            case RepeatMode::RepeatOne:
                ESP_LOGI(TAG, "[RepeatOne] → replay same track");
                next_index = current_index_;
                break;

            case RepeatMode::RepeatAll:
                ESP_LOGI(TAG, "[RepeatAll] → next");
                if (shuffle_enabled_ && playlist_.size() > 1) {
                    int new_i;
                    do {
                        new_i = rand() % playlist_.size();
                    } while (new_i == current_index_);
                    next_index = new_i;
                } else {
                    next_index = findNextTrackIndex(current_index_, +1);
                }
                break;

            case RepeatMode::None:
            default:
                ESP_LOGI(TAG, "[No repeat] → stop");

                if (current_index_ == (int)playlist_.size() - 1) {
                    state_.store(PlayerState::Stopped);
                    return;
                }

                next_index = findNextTrackIndex(current_index_, +1);
                break;
        }
    }

    if (next_index >= 0) {
        current_index_ = next_index;
        play();
    }
}

bool Esp32SdMusic::decodeAndPlayFile(const TrackInfo& track)
{
    SdAudioFormat fmt = DetectAudioFormat(track.path);

    auto display = Board::GetInstance().GetDisplay();
    auto& app    = Application::GetInstance();
    auto codec   = Board::GetInstance().GetAudioCodec();
    if (!codec) {
        ESP_LOGE(TAG, "No audio codec available");
        state_.store(PlayerState::Error);
        return false;
    }

    // Ensure audio output is enabled
    if (!codec->output_enabled()) {
        codec->EnableOutput(true);
    }

    // ==============================
    //  NHÁNH WAV (PCM 16-bit .wav)
    // ==============================
    if (fmt == SdAudioFormat::Wav) {
        FILE* fp = fopen(track.path.c_str(), "rb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot open WAV file: %s", track.path.c_str());
            state_.store(PlayerState::Error);
            return false;
        }

        int wav_sample_rate = 0;
        int wav_channels    = 0;
        size_t data_offset  = 0;
        size_t data_size    = 0;

        if (!ParseWavHeader(fp, wav_sample_rate, wav_channels, data_offset, data_size)) {
            ESP_LOGE(TAG, "Unsupported WAV format (only PCM 16-bit): %s", track.path.c_str());
            fclose(fp);
            state_.store(PlayerState::Error);
            return false;
        }

        if (codec->output_sample_rate() != wav_sample_rate) {
            ESP_LOGI(TAG, "Switch sample rate (WAV) → %d Hz", wav_sample_rate);
            codec->SetOutputSampleRate(wav_sample_rate);
        }

        if (fseek(fp, (long)data_offset, SEEK_SET) != 0) {
            ESP_LOGE(TAG, "Failed to seek to WAV data");
            fclose(fp);
            state_.store(PlayerState::Error);
            return false;
        }

        const size_t kBlockSamples = 1152 * 2; // tương đương MP3 buffer
        std::vector<int16_t> pcm_block(kBlockSamples);
        std::vector<int16_t> mono_block(kBlockSamples);

        current_play_time_ms_ = 0;

        if (wav_sample_rate > 0 && wav_channels > 0) {
            int64_t total_samples_per_chan =
                (int64_t)data_size / (wav_channels * (int)sizeof(int16_t));
            total_duration_ms_ =
                (total_samples_per_chan * 1000) / wav_sample_rate;
        } else {
            total_duration_ms_ = 0;
        }

        state_.store(PlayerState::Playing);

        size_t bytes_consumed = 0;

        while (bytes_consumed < data_size) {
            if (stop_requested_) break;

            if (pause_requested_) {
                {
                    std::unique_lock<std::mutex> lk(state_mutex_);
                    state_.store(PlayerState::Paused);
                    state_cv_.wait(lk, [this]() {
                        return (!pause_requested_) || stop_requested_;
                    });
                }

                if (stop_requested_) break;
                state_.store(PlayerState::Playing);
            }

            {
                DeviceState current_state = app.GetDeviceState();

                if (current_state == kDeviceStateListening ||
                    current_state == kDeviceStateSpeaking) {
                    app.ToggleChatState();
                    vTaskDelay(pdMS_TO_TICKS(300));
                    continue;
                } else if (current_state != kDeviceStateIdle) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }
            }

            size_t remain = data_size - bytes_consumed;
            size_t to_read_bytes =
                std::min(remain, kBlockSamples * sizeof(int16_t));

            size_t read_bytes =
                fread(pcm_block.data(), 1, to_read_bytes, fp);

            if (read_bytes == 0) {
                ESP_LOGI(TAG, "EOF reached for WAV");
                break;
            }

            bytes_consumed += read_bytes;

            size_t total_samples = read_bytes / sizeof(int16_t);
            if (total_samples == 0) {
                continue;
            }

            int16_t* input = pcm_block.data();
            int16_t* final_pcm = nullptr;
            int final_samples = 0;

            if (wav_channels == 2) {
                int samples_per_chan = (int)(total_samples / 2);
                for (int i = 0; i < samples_per_chan; ++i) {
                    int L = input[2 * i];
                    int R = input[2 * i + 1];
                    mono_block[i] = (int16_t)((L + R) / 2);
                }
                final_pcm     = mono_block.data();
                final_samples = samples_per_chan;
            } else {
                // 1 kênh hoặc kênh khác: xử lý như mono
                final_pcm     = input;
                final_samples = (int)total_samples;
            }

            int frame_ms =
                (final_samples * 1000) / wav_sample_rate;
            current_play_time_ms_ += frame_ms;

            AudioStreamPacket pkt;
            pkt.sample_rate    = wav_sample_rate;
            pkt.frame_duration = frame_ms;
            pkt.timestamp      = 0;

            size_t pcm_bytes = final_samples * sizeof(int16_t);
            pkt.payload.resize(pcm_bytes);
            memcpy(pkt.payload.data(), final_pcm, pcm_bytes);

            app.AddAudioData(std::move(pkt));

            if (display) {
                final_pcm_data_fft_ = display->MakeAudioBuffFFT(pcm_bytes);
                display->FeedAudioDataFFT(final_pcm, pcm_bytes);
            }
        }

        fclose(fp);
        return !stop_requested_;
    }

    // ==============================
    //  NHÁNH AAC / FLAC / OGG / OPUS
    //  dùng esp_audio_simple_dec
    // ==============================
    if (fmt == SdAudioFormat::Aac ||
        fmt == SdAudioFormat::Flac ||
        fmt == SdAudioFormat::Ogg  ||
        fmt == SdAudioFormat::Opus) {

        FILE* fp = fopen(track.path.c_str(), "rb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot open file (simple-dec): %s", track.path.c_str());
            state_.store(PlayerState::Error);
            return false;
        }

        esp_audio_simple_dec_cfg_t cfg = {};
        switch (fmt) {
            case SdAudioFormat::Aac:
                cfg.dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_AAC;
                break;
            case SdAudioFormat::Flac:
                cfg.dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_FLAC;
                break;
            case SdAudioFormat::Ogg:
            case SdAudioFormat::Opus:
                ESP_LOGE(TAG, "OGG/OPUS not supported by Simple-Decoder");
                fclose(fp);
                state_.store(PlayerState::Error);
                return false;
            default:
                ESP_LOGE(TAG, "Unsupported simple-dec format");
                fclose(fp);
                state_.store(PlayerState::Error);
                return false;
        }

        cfg.dec_cfg  = nullptr;
        cfg.cfg_size = 0;

        esp_audio_simple_dec_handle_t dec = nullptr;

        esp_audio_dec_register_default();
        esp_audio_simple_dec_register_default();

        esp_audio_err_t dec_ret = esp_audio_simple_dec_open(&cfg, &dec);
        if (dec_ret != ESP_AUDIO_ERR_OK || !dec) {
            ESP_LOGE(TAG, "Failed to open simple decoder, err=%d", (int)dec_ret);
            esp_audio_simple_dec_unregister_default();
            esp_audio_dec_unregister_default();
            fclose(fp);
            state_.store(PlayerState::Error);
            return false;
        }

        std::vector<uint8_t>  in_buf(4096);
        std::vector<int16_t>  mono_buf;
        std::vector<int16_t>  tmp_out(4096 * 4); // bytes -> 16-bit

        bool info_ready = false;
        esp_audio_simple_dec_info_t info{};
        current_play_time_ms_ = 0;
        total_duration_ms_    = 0;
        state_.store(PlayerState::Playing);

        while (true) {
            if (stop_requested_) break;

            if (pause_requested_) {
                {
                    std::unique_lock<std::mutex> lk(state_mutex_);
                    state_.store(PlayerState::Paused);
                    state_cv_.wait(lk, [this]() {
                        return (!pause_requested_) || stop_requested_;
                    });
                }

                if (stop_requested_) break;
                state_.store(PlayerState::Playing);
            }

            {
                DeviceState current_state = app.GetDeviceState();

                if (current_state == kDeviceStateListening ||
                    current_state == kDeviceStateSpeaking) {
                    app.ToggleChatState();
                    vTaskDelay(pdMS_TO_TICKS(300));
                    continue;
                } else if (current_state != kDeviceStateIdle) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }
            }

            size_t read_bytes = fread(in_buf.data(), 1, in_buf.size(), fp);
            bool input_eos = (read_bytes == 0);

            if (read_bytes == 0 && !input_eos) {
                ESP_LOGI(TAG, "EOF reached (simple-dec)");
                break;
            }

            esp_audio_simple_dec_raw_t raw = {};
            raw.buffer = in_buf.data();
            raw.len    = read_bytes;
            raw.eos    = input_eos;

            while ((raw.len > 0 || input_eos) && !stop_requested_) {
                esp_audio_simple_dec_out_t out = {};
                out.buffer = reinterpret_cast<uint8_t*>(tmp_out.data());
                out.len    = tmp_out.size() * sizeof(int16_t);

                dec_ret = esp_audio_simple_dec_process(dec, &raw, &out);
                if (dec_ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
                    // Mở rộng buffer và thử lại
                    tmp_out.resize(out.needed_size / (int)sizeof(int16_t) + 1);
                    continue;
                }
                if (dec_ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGE(TAG, "Decode error (simple-dec): %d", (int)dec_ret);
                    input_eos = true;
                    break;
                }

                if (out.decoded_size == 0) {
                    // Không có PCM lúc này
                    if (input_eos && raw.len == 0) {
                        break;
                    }
                    if (raw.len == 0) {
                        break;
                    }
                    continue;
                }

                if (!info_ready) {
                    esp_audio_simple_dec_get_info(dec, &info);
                    info_ready = true;
                    ESP_LOGI(TAG, "Stream info: %d Hz, %d bit, %d ch",
                             info.sample_rate,
                             info.bits_per_sample,
                             info.channel);

                    if (codec->output_sample_rate() != info.sample_rate) {
                        ESP_LOGI(TAG, "Switch sample rate (simple-dec) → %d Hz", info.sample_rate);
                        codec->SetOutputSampleRate(info.sample_rate);
                    }
                }

                int bits_per_sample = (info.bits_per_sample > 0)
                                      ? info.bits_per_sample
                                      : 16;
                int bytes_per_sample = bits_per_sample / 8;
                if (bytes_per_sample <= 0) bytes_per_sample = 2;

                int channels = (info.channel > 0) ? info.channel : 2;

                int total_samples = out.decoded_size / bytes_per_sample;
                if (total_samples <= 0) {
                    continue;
                }

                int16_t* pcm_in = reinterpret_cast<int16_t*>(out.buffer);
                int16_t* final_pcm = nullptr;
                int final_samples  = 0;

                if (channels == 2) {
                    int samples_per_chan = total_samples / 2;
                    mono_buf.resize(samples_per_chan);
                    for (int i = 0; i < samples_per_chan; ++i) {
                        int L = pcm_in[2 * i];
                        int R = pcm_in[2 * i + 1];
                        mono_buf[i] = (int16_t)((L + R) / 2);
                    }
                    final_pcm    = mono_buf.data();
                    final_samples = samples_per_chan;
                } else {
                    mono_buf.assign(pcm_in, pcm_in + total_samples);
                    final_pcm    = mono_buf.data();
                    final_samples = total_samples;
                }

                int frame_ms =
                    (info.sample_rate > 0)
                        ? (final_samples * 1000) / info.sample_rate
                        : 0;
                current_play_time_ms_ += frame_ms;

                AudioStreamPacket pkt;
                pkt.sample_rate    = info.sample_rate;
                pkt.frame_duration = frame_ms;
                pkt.timestamp      = 0;

                size_t pcm_bytes = final_samples * sizeof(int16_t);
                pkt.payload.resize(pcm_bytes);
                memcpy(pkt.payload.data(), final_pcm, pcm_bytes);

                app.AddAudioData(std::move(pkt));

                if (display) {
                    final_pcm_data_fft_ = display->MakeAudioBuffFFT(pcm_bytes);
                    display->FeedAudioDataFFT(final_pcm, pcm_bytes);
                }

                if (raw.len == 0) {
                    break;
                }
            }

            if (input_eos) {
                ESP_LOGI(TAG, "Input EOS - finishing simple-dec playback");
                break;
            }
        }

        esp_audio_simple_dec_close(dec);
        esp_audio_simple_dec_unregister_default();
        esp_audio_dec_unregister_default();
        fclose(fp);

        return !stop_requested_;
    }

    // ==============================
    //  NHÁNH MP3 (mini-MP3 decoder)
    // ==============================
    if (!mp3_decoder_initialized_ && !initializeMp3Decoder()) {
        state_.store(PlayerState::Error);
        return false;
    }

    FILE* fp = fopen(track.path.c_str(), "rb");
    if (!fp) {
        ESP_LOGE(TAG, "Cannot open MP3 file: %s", track.path.c_str());
        state_.store(PlayerState::Error);
        return false;
    }

    struct stat st{};
    int64_t file_size = 0;
    if (stat(track.path.c_str(), &st) == 0) {
        file_size = st.st_size;
    }

    const int INPUT_BUF = 4096;

    uint8_t* input = (uint8_t*) heap_caps_malloc(
        INPUT_BUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!input) {
        ESP_LOGE(TAG, "Cannot allocate input buffer");
        fclose(fp);
        return false;
    }

    int16_t* pcm = (int16_t*) heap_caps_malloc(
        2304 * sizeof(int16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm) {
        ESP_LOGE(TAG, "Cannot allocate PCM buffer");
        heap_caps_free(input);
        fclose(fp);
        return false;
    }

    int bytes_left   = 0;
    uint8_t* read_ptr = input;
    bool id3_done    = false;

    current_play_time_ms_ = 0;
    total_duration_ms_    = 0;

    state_.store(PlayerState::Playing);

    while (true) {
        if (stop_requested_) break;

        if (pause_requested_) {
            {
                std::unique_lock<std::mutex> lk(state_mutex_);
                state_.store(PlayerState::Paused);
                state_cv_.wait(lk, [this]() {
                    return (!pause_requested_) || stop_requested_;
                });
            }

            if (stop_requested_) break;
            state_.store(PlayerState::Playing);
        }

        {
            DeviceState current_state = app.GetDeviceState();

            if (current_state == kDeviceStateListening ||
                current_state == kDeviceStateSpeaking) {
                app.ToggleChatState();
                vTaskDelay(pdMS_TO_TICKS(300));
                continue;
            } else if (current_state != kDeviceStateIdle) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
        }

        if (bytes_left < 1024) {
            if (bytes_left > 0 && read_ptr != input) {
                memmove(input, read_ptr, bytes_left);
            }

            size_t space = INPUT_BUF - bytes_left;
            size_t read_bytes = fread(input + bytes_left, 1, space, fp);
            if (stop_requested_) break;

            bytes_left += read_bytes;
            read_ptr = input;

            if (!id3_done && bytes_left >= 10) {
                size_t skip = SkipId3Tag(read_ptr, bytes_left);
                if (skip > 0 && skip <= (size_t)bytes_left) {
                    read_ptr  += skip;
                    bytes_left -= skip;
                    ESP_LOGI(TAG, "ID3v2 header skipped (%u bytes)", (unsigned)skip);
                }
                id3_done = true;
            }

            if (read_bytes == 0 && bytes_left == 0) {
                ESP_LOGI(TAG, "EOF reached");
                break;
            }
        }

        int off = MP3FindSyncWord(read_ptr, bytes_left);
        if (off < 0) {
            bytes_left = 0;
            continue;
        }

        if (off > 0) {
            read_ptr  += off;
            bytes_left -= off;
        }

        int ret = MP3Decode(mp3_decoder_, &read_ptr, &bytes_left, pcm, 0);
        if (stop_requested_) break;

        if (ret != 0) {
            if (bytes_left > 1) {
                read_ptr++;
                bytes_left--;
            } else {
                bytes_left = 0;
            }
            continue;
        }

        MP3GetLastFrameInfo(mp3_decoder_, &mp3_frame_info_);
        if (mp3_frame_info_.samprate == 0 ||
            mp3_frame_info_.nChans   == 0) {
            continue;
        }

        if (codec->output_sample_rate() != mp3_frame_info_.samprate) {
            ESP_LOGI(TAG, "Switch sample rate → %d Hz", mp3_frame_info_.samprate);
            codec->SetOutputSampleRate(mp3_frame_info_.samprate);
        }

        if (!codec->output_enabled()) {
            ESP_LOGW(TAG, "Audio output disabled - re-enabling.");
            codec->EnableOutput(true);
        }

        int frame_ms =
            (mp3_frame_info_.outputSamps * 1000) /
            (mp3_frame_info_.samprate * mp3_frame_info_.nChans);

        current_play_time_ms_ += frame_ms;

        if (total_duration_ms_.load() == 0 &&
            file_size > 0 &&
            mp3_frame_info_.bitrate > 0) {

            total_duration_ms_ =
                (file_size * 8LL * 1000LL) / mp3_frame_info_.bitrate;

            {
                std::lock_guard<std::mutex> lock(playlist_mutex_);
                if (current_index_ >= 0 &&
                    current_index_ < (int)playlist_.size()) {
                    auto& ti        = playlist_[current_index_];
                    ti.duration_ms  = (int)total_duration_ms_.load();
                    ti.bitrate_kbps = mp3_frame_info_.bitrate / 1000;
                    ti.file_size    = (size_t)file_size;
                }
            }
        }

        int16_t* final_pcm    = pcm;
        int final_samples = mp3_frame_info_.outputSamps;

        if (mp3_frame_info_.nChans == 2)
        {
            int mono_samples = final_samples / 2;
            for (int i = 0; i < mono_samples; i++) {
                int L = pcm[2 * i];
                int R = pcm[2 * i + 1];
                pcm[i] = (L + R) / 2;
            }
            final_pcm    = pcm;
            final_samples = mono_samples;
        }

        AudioStreamPacket pkt;
        pkt.sample_rate    = mp3_frame_info_.samprate;
        pkt.frame_duration =
            (mp3_frame_info_.outputSamps * 1000) /
            (mp3_frame_info_.samprate * mp3_frame_info_.nChans);
        pkt.timestamp      = 0;

        size_t pcm_bytes = final_samples * sizeof(int16_t);
        pkt.payload.resize(pcm_bytes);
        memcpy(pkt.payload.data(), final_pcm, pcm_bytes);

        app.AddAudioData(std::move(pkt));

        if (display) {
            final_pcm_data_fft_ = display->MakeAudioBuffFFT(pcm_bytes);
            display->FeedAudioDataFFT(final_pcm, pcm_bytes);
        }
    }

    heap_caps_free(pcm);
    heap_caps_free(input);
    fclose(fp);

    return !stop_requested_;
}

// ============================================================================
//                         PART 3 / 3
//      DECODER UTIL / STATE / PROGRESS / GỢI Ý BÀI HÁT
// ============================================================================

bool Esp32SdMusic::initializeMp3Decoder()
{
    if (mp3_decoder_initialized_) {
        ESP_LOGW(TAG, "MP3 decoder already initialized");
        return true;
    }

    mp3_decoder_ = MP3InitDecoder();
    if (!mp3_decoder_) {
        ESP_LOGE(TAG, "Failed to init MP3 decoder");
        return false;
    }

    mp3_decoder_initialized_ = true;
    ESP_LOGI(TAG, "MP3 decoder initialized");
    return true;
}

void Esp32SdMusic::cleanupMp3Decoder()
{
    if (mp3_decoder_) {
        MP3FreeDecoder(mp3_decoder_);
        mp3_decoder_ = nullptr;
    }
    mp3_decoder_initialized_ = false;
}

size_t Esp32SdMusic::SkipId3Tag(uint8_t* data, size_t size)
{
    if (!data || size < 10) return 0;
    if (memcmp(data, "ID3", 3) != 0) return 0;

    uint32_t tag_sz =
        ((data[6] & 0x7F) << 21) |
        ((data[7] & 0x7F) << 14) |
        ((data[8] & 0x7F) << 7)  |
         (data[9] & 0x7F);

    size_t total = 10 + tag_sz;
    if (total > size) total = size;

    return total;
}

void Esp32SdMusic::resetSampleRate()
{
    auto codec = Board::GetInstance().GetAudioCodec();
    if (!codec) return;

    int orig = codec->original_output_sample_rate();
    if (orig <= 0) return;

    int cur = codec->output_sample_rate();
    if (cur != orig) {
        ESP_LOGI(TAG, "Reset sample rate: %d → %d", cur, orig);
        codec->SetOutputSampleRate(-1);
    }
}

Esp32SdMusic::TrackProgress Esp32SdMusic::updateProgress() const
{
    TrackProgress p;
    p.position_ms = current_play_time_ms_.load();
    p.duration_ms = total_duration_ms_.load();
    return p;
}

int16_t* Esp32SdMusic::getFFTData() const
{
    return final_pcm_data_fft_;
}

Esp32SdMusic::PlayerState Esp32SdMusic::getState() const
{
    return state_.load();
}

int Esp32SdMusic::getBitrate() const
{
    int br = mp3_frame_info_.bitrate;
    if (br < 0) br = 0;
    return br;
}

int64_t Esp32SdMusic::getDurationMs() const
{
    return total_duration_ms_.load();
}

int64_t Esp32SdMusic::getCurrentPositionMs() const
{
    return current_play_time_ms_.load();
}

std::string Esp32SdMusic::getDurationString() const
{
    return MsToTimeString(total_duration_ms_.load());
}

std::string Esp32SdMusic::getCurrentTimeString() const
{
    return MsToTimeString(current_play_time_ms_.load());
}

// Gợi ý bài tiếp theo dựa trên lịch sử phát
std::vector<Esp32SdMusic::TrackInfo>
Esp32SdMusic::suggestNextTracks(size_t max_results)
{
    std::vector<TrackInfo> results;
    if (max_results == 0) return results;

    int base_index = -1;
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        if (!play_history_indices_.empty()) {
            base_index = play_history_indices_.back();
        }
    }

    std::vector<TrackInfo> playlist_copy;
    std::vector<uint32_t> count_copy;
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (playlist_.empty()) return results;
        playlist_copy = playlist_;
        count_copy    = play_count_;
    }

    if (base_index < 0 || base_index >= (int)playlist_copy.size()) {
        size_t limit = std::min(max_results, playlist_copy.size());
        results.assign(playlist_copy.begin(),
                       playlist_copy.begin() + limit);
        return results;
    }

    TrackInfo base = playlist_copy[base_index];

    struct Scored { int index; int score; };
    std::vector<Scored> scored;
    int n = static_cast<int>(playlist_copy.size());
    scored.reserve(std::max(0, n - 1));

    for (int i = 0; i < n; ++i) {
        if (i == base_index) continue;
        uint32_t pc = (i < (int)count_copy.size()) ? count_copy[i] : 0;
        int s = ComputeTrackScoreForBase(base, playlist_copy[i], pc);
        scored.push_back({i, s});
    }

    std::sort(scored.begin(), scored.end(),
              [](const Scored& a, const Scored& b) {
                  if (a.score != b.score) return a.score > b.score;
                  return a.index < b.index;
              });

    size_t limit = std::min(max_results, scored.size());
    results.reserve(limit);
    for (size_t i = 0; i < limit; ++i) {
        results.push_back(playlist_copy[scored[i].index]);
    }

    return results;
}

// Gợi ý bài giống bài X
std::vector<Esp32SdMusic::TrackInfo>
Esp32SdMusic::suggestSimilarTo(const std::string& name_or_path,
                               size_t max_results)
{
    std::vector<TrackInfo> results;
    if (max_results == 0) return results;

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (playlist_.empty()) {
            ESP_LOGW(TAG, "suggestSimilarTo(): playlist empty — reloading");
        }
    }
    if (playlist_.empty()) {
        if (!loadTrackList()) {
            ESP_LOGE(TAG, "suggestSimilarTo(): cannot load playlist");
            return results;
        }
    }

    int base_index = findTrackIndexByKeyword(name_or_path);
    if (base_index < 0) {
        return suggestNextTracks(max_results);
    }

    std::vector<TrackInfo> playlist_copy;
    std::vector<uint32_t> count_copy;
    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        playlist_copy = playlist_;
        count_copy    = play_count_;
    }

    if (playlist_copy.empty() ||
        base_index < 0 ||
        base_index >= (int)playlist_copy.size()) {
        return results;
    }

    TrackInfo base = playlist_copy[base_index];

    struct Scored { int index; int score; };
    std::vector<Scored> scored;
    int n = static_cast<int>(playlist_copy.size());
    scored.reserve(std::max(0, n - 1));

    for (int i = 0; i < n; ++i) {
        if (i == base_index) continue;
        uint32_t pc = (i < (int)count_copy.size()) ? count_copy[i] : 0;
        int s = ComputeTrackScoreForBase(base, playlist_copy[i], pc);
        scored.push_back({i, s});
    }

    std::sort(scored.begin(), scored.end(),
              [](const Scored& a, const Scored& b) {
                  if (a.score != b.score) return a.score > b.score;
                  return a.index < b.index;
              });

    size_t limit = std::min(max_results, scored.size());
    results.reserve(limit);
    for (size_t i = 0; i < limit; ++i) {
        results.push_back(playlist_copy[scored[i].index]);
    }

    return results;
}

// Tạo danh sách bài theo thể loại (genre từ ID3v1 / ID3v2)
bool Esp32SdMusic::buildGenrePlaylist(const std::string& genre)
{
    std::string kw = ToLowerAscii(genre);
    if (kw.empty()) return false;

    std::vector<int> indices;

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        for (int i = 0; i < (int)playlist_.size(); ++i) {
            std::string g = ToLowerAscii(playlist_[i].genre);
            if (!g.empty() && g.find(kw) != std::string::npos) {
                indices.push_back(i);
            }
        }
    }

    if (indices.empty()) {
        ESP_LOGW(TAG, "No tracks found with genre '%s'", genre.c_str());
        return false;
    }

    genre_playlist_   = indices;
    genre_current_key_ = genre;
    genre_current_pos_ = 0;

    ESP_LOGI(TAG, "Genre playlist built for '%s' (%d tracks)",
             genre.c_str(), (int)indices.size());
    return true;
}

bool Esp32SdMusic::playGenreIndex(int pos)
{
    if (genre_playlist_.empty())
        return false;

    if (pos < 0 || pos >= (int)genre_playlist_.size())
        return false;

    int track_index = genre_playlist_[pos];

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (track_index < 0 || track_index >= (int)playlist_.size())
            return false;

        current_index_ = track_index;
    }

    genre_current_pos_ = pos;

    ESP_LOGI(TAG, "Play genre-track [%d/%d] → index %d (%s)",
             pos + 1, (int)genre_playlist_.size(),
             track_index,
             playlist_[track_index].name.c_str());

    return play();
}

bool Esp32SdMusic::playNextGenre()
{
    if (genre_playlist_.empty())
        return false;

    int next_pos = genre_current_pos_ + 1;

    if (next_pos >= (int)genre_playlist_.size()) {
        ESP_LOGI(TAG, "End of genre playlist '%s'", genre_current_key_.c_str());
        return false;
    }

    genre_current_pos_ = next_pos;
    int track_index = genre_playlist_[next_pos];

    {
        std::lock_guard<std::mutex> lock(playlist_mutex_);
        if (track_index < 0 || track_index >= (int)playlist_.size())
            return false;

        current_index_ = track_index;
    }

    ESP_LOGI(TAG, "Next genre track → pos=%d → index=%d (%s)",
             next_pos,
             track_index,
             playlist_[track_index].name.c_str());

    return play();
}
