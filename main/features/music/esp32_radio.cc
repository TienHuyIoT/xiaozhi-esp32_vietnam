/**
 * @file esp32_radio.cc
 * @brief Internet radio player  supports AAC and MP3 streams.
 *
 * Inherits AudioStreamPlayer for streaming, decoding and playback.
 * Adds: preset station list, station search, volume per station,
 *       decoder type auto-detection.
 */

#include "esp32_radio.h"
#include "board.h"
#include "audio/audio_codec.h"

#include <esp_log.h>
#include <algorithm>
#include <cctype>

static const char* TAG = "Esp32Radio";

/* ================================================================== */
/*  Constructor / Destructor                                          */
/* ================================================================== */

Esp32Radio::Esp32Radio()
    : AudioStreamPlayer(),
      station_name_displayed_(false),
      current_station_volume_(RADIO_DEFAULT_VOLUME)
{
}

Esp32Radio::~Esp32Radio()
{
    ESP_LOGI(TAG, "Destroying Esp32Radio");
    Stop();
    ESP_LOGI(TAG, "Esp32Radio destroyed");
}

void Esp32Radio::Initialize(AudioCodec* codec)
{
    if (codec) {
        SetAudioCodec(codec);
    }
    InitializeRadioStations();
    ESP_LOGI(TAG, "Radio player initialised with %d stations (codec=%s)",
             (int)radio_stations_.size(), codec ? "direct" : "app-pipeline");
}

/* ================================================================== */
/*  Station presets                                                    */
/* ================================================================== */

void Esp32Radio::InitializeRadioStations()
{
    // /* === VOV - National channels === */
    // radio_stations_["VOV1"]         = RadioStation("VOV 1 - Thời sự",                   "https://stream.vovmedia.vn/vov-1",       "Tin tức & thời sự quốc gia",               "News/Talk",            4.5f);
    // radio_stations_["VOV2"]         = RadioStation("VOV 2 - Văn hóa & Giáo dục",        "https://stream.vovmedia.vn/vov-2",       "Văn hóa - giáo dục - xã hội",              "Culture/Education",    4.0f);
    // radio_stations_["VOV3"]         = RadioStation("VOV 3 - Âm nhạc & Giải trí",        "https://stream.vovmedia.vn/vov-3",       "Nhạc & giải trí tổng hợp",                 "Music/Entertainment",  4.4f);
    // radio_stations_["VOV5"]         = RadioStation("VOV 5 - Đối ngoại",                 "https://stream.vovmedia.vn/vov5",        "Kênh tiếng Việt & quốc tế",                "International",        4.1f);
    // 
    // /* === VOV Traffic === */
    // radio_stations_["VOV_GT_HN"]    = RadioStation("VOV Giao thông Hà Nội",             "https://stream.vovmedia.vn/vovgt-hn",    "Giao thông & đời sống Hà Nội",             "Traffic",              4.7f);
    // radio_stations_["VOV_GT_HCM"]   = RadioStation("VOV Giao thông TP.HCM",             "https://stream.vovmedia.vn/vovgt-hcm",   "Giao thông & đời sống TP.HCM",             "Traffic",              4.7f);

    // /* === VOV Regional (VOV4) === */
    // radio_stations_["VOV_MEKONG"]       = RadioStation("VOV Mekong FM",                 "https://stream.vovmedia.vn/vovmekong",   "Miền Tây - Đồng bằng sông Cửu Long",       "Regional",             4.6f);
    // radio_stations_["VOV4_MIENTRUNG"]   = RadioStation("VOV4 Miền Trung",               "https://stream.vovmedia.vn/vov4mt",      "Dân tộc - Miền Trung",                      "Regional",             4.3f);
    // radio_stations_["VOV4_TAYBAC"]      = RadioStation("VOV4 Tây Bắc",                  "https://stream.vovmedia.vn/vov4tb",      "Dân tộc - Tây Bắc",                         "Regional",             4.4f);
    // radio_stations_["VOV4_DONGBAC"]     = RadioStation("VOV4 Đông Bắc",                 "https://stream.vovmedia.vn/vov4db",      "Dân tộc - Đông Bắc",                        "Regional",             4.4f);
    // radio_stations_["VOV4_TAYNGUYEN"]   = RadioStation("VOV4 Tây Nguyên",               "https://stream.vovmedia.vn/vov4tn",      "Dân tộc - Tây Nguyên",                      "Regional",             4.5f);
    // radio_stations_["VOV4_DBSCL"]       = RadioStation("VOV4 ĐBSCL",                    "https://stream.vovmedia.vn/vov4dbscl",   "Dân tộc - Đồng bằng sông Cửu Long",         "Regional",             4.5f);
    // radio_stations_["VOV4_HCM"]         = RadioStation("VOV4 TP.HCM",                   "https://stream.vovmedia.vn/vov4hcm",     "Dân tộc - TP.HCM",                          "Regional",             4.5f);

    // /* === VOV English === */
    // radio_stations_["VOV5_ENGLISH"]     = RadioStation("VOV 5 – English 24/7",          "https://stream.vovmedia.vn/vov247",      "Kênh tiếng Anh quốc tế",                   "International",        4.0f);

    ESP_LOGI(TAG, "Initialised %d radio stations", (int)radio_stations_.size());
}

/* ================================================================== */
/*  Decoder-type heuristic                                            */
/* ================================================================== */

AudioDecoderType Esp32Radio::GuessDecoderType(const std::string& url) const
{
    /* VOV streams are AAC/AAC+ */
    if (url.find("vovmedia.vn") != std::string::npos) {
        return AudioDecoderType::AAC;
    }

    /* Check common URL patterns */
    std::string lower = url;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower.find(".aac") != std::string::npos ||
        lower.find("aacp") != std::string::npos ||
        lower.find("aac+") != std::string::npos) {
        return AudioDecoderType::AAC;
    }

    if (lower.find(".mp3") != std::string::npos) {
        return AudioDecoderType::MP3;
    }

    /* Default to AAC for unknown radio streams (common for internet radio) */
    ESP_LOGW(TAG, "Cannot determine stream type from URL, defaulting to AAC");
    return AudioDecoderType::AAC;
}

/* ================================================================== */
/*  PlayStation  find station by name/key                            */
/* ================================================================== */

bool Esp32Radio::PlayStation(const std::string& station_name)
{
    ESP_LOGI(TAG, "PlayRadio: %s", station_name.c_str());

    std::string lower_input = station_name;
    std::transform(lower_input.begin(), lower_input.end(), lower_input.begin(), ::tolower);

    /* 1) Search by display name (partial, case-insensitive) */
    for (const auto& kv : radio_stations_) {
        std::string lower_name = kv.second.name;
        std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);

        if (lower_name.find(lower_input) != std::string::npos ||
            lower_input.find(lower_name) != std::string::npos) {
            ESP_LOGI(TAG, "Matched display name: '%s' -> %s (vol=%.1f)",
                     station_name.c_str(), kv.second.name.c_str(), kv.second.volume);
            current_station_volume_ = kv.second.volume;
            return PlayUrl(kv.second.url, kv.second.name);
        }
    }

    /* 2) Exact key match */
    auto it = radio_stations_.find(station_name);
    if (it != radio_stations_.end()) {
        current_station_volume_ = it->second.volume;
        return PlayUrl(it->second.url, it->second.name);
    }

    /* 3) Key match (case-insensitive) */
    for (const auto& kv : radio_stations_) {
        std::string lower_key = kv.first;
        std::transform(lower_key.begin(), lower_key.end(), lower_key.begin(), ::tolower);
        if (lower_key == lower_input) {
            current_station_volume_ = kv.second.volume;
            return PlayUrl(kv.second.url, kv.second.name);
        }
    }

    /* 4) Vietnamese phonetic / keyword matching */
    static const std::vector<std::pair<std::string, std::string>> keyword_map = {
        {"tây nguyên", "VOV4_TAYNGUYEN"},
        {"tay nguyen", "VOV4_TAYNGUYEN"},
        {"giao thông", "VOV_GT_HN"},
        {"mê kông"   , "VOV_MEKONG"},
        {"mekong"    , "VOV_MEKONG"},
    };

    for (const auto& pair : keyword_map) {
        if (lower_input.find(pair.first) != std::string::npos) {
            auto found = radio_stations_.find(pair.second);
            if (found != radio_stations_.end()) {
                current_station_volume_ = found->second.volume;
                return PlayUrl(found->second.url, found->second.name);
            }
        }
    }

    /* 5) VOV + number shorthand */
    if (lower_input.find("vov") != std::string::npos) {
        for (char c = '1'; c <= '5'; ++c) {
            if (lower_input.find(c) != std::string::npos) {
                std::string key = "VOV" + std::string(1, c);
                auto found = radio_stations_.find(key);
                if (found != radio_stations_.end()) {
                    current_station_volume_ = found->second.volume;
                    return PlayUrl(found->second.url, found->second.name);
                }
            }
        }
        /* Default to VOV1 for generic "vov" */
        auto vov1 = radio_stations_.find("VOV1");
        if (vov1 != radio_stations_.end()) {
            current_station_volume_ = vov1->second.volume;
            return PlayUrl(vov1->second.url, vov1->second.name);
        }
    }

    ESP_LOGE(TAG, "Station not found: '%s'", station_name.c_str());
    return false;
}

/* ================================================================== */
/*  PlayUrl                                                           */
/* ================================================================== */

bool Esp32Radio::PlayUrl(const std::string& radio_url, const std::string& station_name)
{
    if (radio_url.empty()) {
        ESP_LOGE(TAG, "Radio URL is empty");
        return false;
    }

    ESP_LOGI(TAG, "PlayUrl: %s (%s)",
             station_name.empty() ? "Custom URL" : station_name.c_str(),
             radio_url.c_str());

    Stop();

    current_station_url_  = radio_url;
    current_station_name_ = station_name.empty() ? "Custom Radio" : station_name;
    station_name_displayed_ = false;

    if (current_station_volume_ <= 0.0f) {
        current_station_volume_ = RADIO_DEFAULT_VOLUME;
    }
    SetVolume(current_station_volume_);

    /* Auto-detect decoder type */
    AudioDecoderType dtype = GuessDecoderType(radio_url);
    ESP_LOGI(TAG, "Using decoder: %s", (dtype == AudioDecoderType::AAC) ? "AAC" : "MP3");

    return StartStream(radio_url, dtype);
}

/* ================================================================== */
/*  Stop                                                              */
/* ================================================================== */

bool Esp32Radio::Stop()
{
    if (!IsPlaying() && !IsDownloading()) {
        return true;
    }

    ESP_LOGI(TAG, "Stopping radio");
    return StopStream();
}

/* ================================================================== */
/*  GetStationList                                                    */
/* ================================================================== */

std::vector<std::string> Esp32Radio::GetStationList() const
{
    std::vector<std::string> list;
    list.reserve(radio_stations_.size());
    for (const auto& kv : radio_stations_) {
        list.push_back(kv.first + " - " + kv.second.name);
    }
    return list;
}

/* ================================================================== */
/*  AudioStreamPlayer hooks                                           */
/* ================================================================== */

void Esp32Radio::OnStreamInfoReady(int sample_rate, int bits_per_sample, int channels, int bitrate, int frame_size)
{
    ESP_LOGI(TAG, "Stream info: %s, %d Hz, %d bit, %d ch, %d kbps, %d frame size",
             current_station_name_.c_str(), sample_rate, bits_per_sample, channels, bitrate, frame_size);
}

void Esp32Radio::OnDisplayReady()
{
    /* Display is now handled externally via Application callbacks */
    ESP_LOGD(TAG, "Display ready for station: %s", current_station_name_.c_str());
    station_name_displayed_ = true;
}

bool Esp32Radio::OnPlaybackFinishedAndContinue()
{
    ESP_LOGI(TAG, "Radio playback finished");
    return false;
}
