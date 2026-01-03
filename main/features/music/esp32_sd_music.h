#ifndef ESP32_SD_MUSIC_H
#define ESP32_SD_MUSIC_H

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "mp3dec.h"
}

class Esp32SdMusic {
public:
    // ============================================================
    // 1) ENUM — State Machine
    // ============================================================
    enum class PlayerState {
        Stopped = 0,
        Preparing,
        Playing,
        Paused,
        Error
    };

    // ============================================================
    // 2) ENUM — Repeat modes
    // ============================================================
    enum class RepeatMode {
        None = 0,
        RepeatOne,
        RepeatAll
    };

    // ============================================================
    // 3) Struct TrackInfo — dữ liệu một bài (đã tối giản ID3)
    //  - Chỉ còn ID3v1 (title/artist/album/genre/comment/year/track)
    //  - KHÔNG còn mtime, cover_offset, ID3v2 text
    //  - Duration/bitrate tính từ MP3 frame + file_size
    // ============================================================
    struct TrackInfo {
        // Hiển thị
        std::string name;   // Tên hiển thị (ưu tiên ID3v1 title, fallback tên file)
        std::string path;   // Đường dẫn tuyệt đối

        // Metadata cơ bản (từ ID3v1 — rất nhẹ)
        std::string title;
        std::string artist;
        std::string album;
        std::string genre;
        std::string comment;
        std::string year;
        int         track_number = 0;

        // Audio info (cập nhật khi decode)
        int   duration_ms  = 0;
        int   bitrate_kbps = 0;
        size_t file_size   = 0;  // dùng cho info/debug, không cache theo mtime

        // Cover art (không parse nữa nhưng giữ field để không phá tool MCP)
        uint32_t    cover_size = 0;   // luôn 0 vì không parse APIC
        std::string cover_mime;       // luôn rỗng
    };

    // ============================================================
    // 4) Struct Progress — dành cho UI
    // ============================================================
    struct TrackProgress {
        int64_t position_ms = 0;
        int64_t duration_ms = 0;
    };

public:
    // ============================================================
    // 5) Constructor / Destructor
    // ============================================================
    Esp32SdMusic();
    ~Esp32SdMusic();

    void Initialize(class SdCard* sd_card);

    // ============================================================
    // 6) Playlist API cơ bản
    // ============================================================
    // Đọc playlist từ playlist.json, nếu không có/hỏng sẽ quét SD và ghi lại
    bool loadTrackList();
    size_t getTotalTracks() const;
    std::vector<TrackInfo> listTracks() const;

    // Chọn thư mục gốc phát nhạc (sẽ dùng playlist.json trong thư mục đó)
    bool setDirectory(const std::string& relative_dir);
    bool playDirectory(const std::string& relative_dir);

    // Phát theo tên / keyword
    bool playByName(const std::string& keyword);
    TrackInfo getTrackInfo(int index) const;
    bool setTrack(int index);

    std::string getCurrentTrack() const;
    std::string getCurrentTrackPath() const;

    std::vector<std::string> listDirectories() const;
    std::vector<TrackInfo> searchTracks(const std::string& keyword) const;

    std::string resolveLongName(const std::string& path);
    std::string resolveCaseInsensitiveDir(const std::string& path);

    // Đếm số bài trong 1 thư mục (dùng playlist hiện tại, không quét lại SD)
    size_t countTracksInDirectory(const std::string& relative_dir);
    size_t countTracksInCurrentDirectory() const;

    std::vector<TrackInfo> listTracksPage(size_t page_index,
                                          size_t page_size = 10) const;

    // Quét lại toàn bộ SD (từ root_directory_) và ghi đè playlist.json + RAM
    bool rebuildPlaylistFromSd();

    // ============================================================
    // 7) Playback API
    // ============================================================
    bool play();
    void pause();
    void stop();

    bool next();
    bool prev();
    bool IsPlaying() const;

    // ============================================================
    // 8) Playback Settings
    // ============================================================
    void shuffle(bool enabled);
    void repeat(RepeatMode mode);

    // ============================================================
    // 9) Query state / FFT
    // ============================================================
    PlayerState getState() const;
    TrackProgress updateProgress() const;
    int16_t* getFFTData() const;

    int64_t getDurationMs() const;
    int64_t getCurrentPositionMs() const;
    int getBitrate() const;
    std::string getDurationString() const;
    std::string getCurrentTimeString() const;

    // ============================================================
    // 10) Gợi ý bài hát
    // ============================================================
    std::vector<TrackInfo> suggestNextTracks(size_t max_results = 5);
    std::vector<TrackInfo> suggestSimilarTo(const std::string& name_or_path,
                                            size_t max_results = 5);

    // ============================================================
    // 11) Playlist theo THỂ LOẠI (dựa trên ID3v1 genre index)
    // ============================================================
    bool buildGenrePlaylist(const std::string& genre);
    bool playGenreIndex(int pos);
    bool playNextGenre();
    std::vector<std::string> listGenres() const;

private:
    // ============================================================
    // Playlist helpers
    // ============================================================
    void scanDirectoryRecursive(const std::string& dir,
                                std::vector<TrackInfo>& out);

    int findNextTrackIndex(int start, int direction);
    bool resolveDirectoryRelative(const std::string& relative_dir,
                                  std::string& out_full);
    int findTrackIndexByKeyword(const std::string& keyword) const;

    // Playlist file (playlist.json)
    bool loadPlaylistFromFile(const std::string& playlist_path,
                              std::vector<TrackInfo>& out) const;
    bool savePlaylistToFile(const std::string& playlist_path,
                            const std::vector<TrackInfo>& list) const;

    // ============================================================
    // Playback Thread
    // ============================================================
    void playbackThreadFunc();
    bool decodeAndPlayFile(const TrackInfo& track);
    void joinPlaybackThreadWithTimeout();

    // ============================================================
    // MP3 Decoder Utilities
    // ============================================================
    bool initializeMp3Decoder();
    void cleanupMp3Decoder();
    size_t SkipId3Tag(uint8_t* data, size_t size);
    void resetSampleRate();

    // ============================================================
    // Lịch sử phát & gợi ý
    // ============================================================
    void recordPlayHistory(int index);

private:
    SdCard* sd_card_;
    // Playlist / thư mục
    std::string root_directory_;
    std::vector<TrackInfo> playlist_;
    mutable std::mutex playlist_mutex_;
    int current_index_ = -1;
    std::vector<uint32_t> play_count_;

    // Playback state / thread
    std::thread playback_thread_;
    std::atomic<bool> stop_requested_;
    std::atomic<bool> pause_requested_;
    std::atomic<PlayerState> state_;

    mutable std::mutex state_mutex_;
    std::condition_variable state_cv_;

    // Playback options
    bool shuffle_enabled_;
    RepeatMode repeat_mode_;

    // Progress tracking
    std::atomic<int64_t> current_play_time_ms_;
    std::atomic<int64_t> total_duration_ms_;

    // FFT buffer (display owns memory)
    int16_t* final_pcm_data_fft_;

    // mini-mp3 decoder
    void* mp3_decoder_;
    bool mp3_decoder_initialized_;
    MP3FrameInfo mp3_frame_info_{};

    // Playlist theo thể loại
    std::vector<int> genre_playlist_;
    int genre_current_pos_ = -1;
    std::string genre_current_key_;

    // History / gợi ý
    mutable std::mutex history_mutex_;
    std::vector<int> play_history_indices_;
};

#endif // ESP32_SD_MUSIC_H
