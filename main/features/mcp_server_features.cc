/**
 * @file mcp_server_features.cc
 * @brief MCP tool registrations for media features (music, radio, SD, video).
 *
 * Each method receives a component pointer and captures it in tool
 * lambdas, avoiding Application::GetInstance() calls from handlers.
 */

#include "mcp_server_features.h"
#include "mcp_server.h"
#include "board.h"
#include "esp32_music.h"
#include "esp32_radio.h"
#include "esp32_sd_music.h"
#include "features/video/video_player.h"
#include "wifi_station.h"
#include "system_info.h"
#include "display.h"
#include "features/QRCode/qrcode_display.h"
#include "features/alarm_clock/alarm_manager.h"
#include "features/alarm_clock/alarm_sounds.h"

#include <esp_log.h>
#include <cJSON.h>
#include <algorithm>
#include <cstring>

#define TAG "McpFeatures"

/* ================================================================== */
/*  Network / QR code tool                                             */
/* ================================================================== */

void McpFeatureTools::RegisterIp2QrCodeTool(std::function<void(bool)> overlay_cb) {
    // Wire the overlay callback to the QRCodeDisplay singleton so that
    // Show() and Clear() automatically notify the host display.
    qrcode::QRCodeDisplay::GetInstance().SetOverlayCallback(std::move(overlay_cb));

    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.network.ip2qrcode",
        "Print the QR code of the IP address connected to WiFi network.\n"
        "Use this tool when user asks about network connection, IP address and print QR code.\n"
        "Returns the new IP address, SSID, and connection status. Also displays IP address as QR code on screen.",
        PropertyList(),
        [](const PropertyList& /*properties*/) -> ReturnValue {
            auto& wifi_station = WifiStation::GetInstance();
            ESP_LOGI(TAG, "Getting network status for IP address tool");
            cJSON* json = cJSON_CreateObject();
            cJSON_AddBoolToObject(json, "connected", wifi_station.IsConnected());

            if (wifi_station.IsConnected()) {
                std::string ip_address = wifi_station.GetIpAddress();
                cJSON_AddStringToObject(json, "ip_address", ip_address.c_str());
                cJSON_AddStringToObject(json, "ssid", wifi_station.GetSsid().c_str());
                cJSON_AddNumberToObject(json, "rssi", wifi_station.GetRssi());
                cJSON_AddNumberToObject(json, "channel", wifi_station.GetChannel());
                cJSON_AddStringToObject(json, "mac_address", SystemInfo::GetMacAddress().c_str());
                cJSON_AddStringToObject(json, "status", "connected");

                // Use standalone QRCode feature module
                std::string ota_path = ip_address + "/ota";
                std::string qr_url  = "http://" + ota_path;

                auto& qr = qrcode::QRCodeDisplay::GetInstance();
                bool ok = qr.Show(qr_url, ota_path);
                cJSON_AddBoolToObject(json, "qrcode_displayed", ok);

                if (ok) {
                    ESP_LOGI(TAG, "QR code displayed for IP: %s", ip_address.c_str());
                } else {
                    ESP_LOGE(TAG, "Failed to display QR code");
                    // Fallback: show IP in chat message
                    auto display = Board::GetInstance().GetDisplay();
                    if (display) {
                        display->SetChatMessage("assistant", ota_path.c_str());
                    }
                }
            } else {
                cJSON_AddStringToObject(json, "status", "disconnected");
                cJSON_AddStringToObject(json, "message", "Device is not connected to WiFi");
            }
            return json;
        });
}

/* ------------------------------------------------------------------ */
/*  Online Music tools                                                 */
/* ------------------------------------------------------------------ */

void McpFeatureTools::RegisterMusicTools(Esp32Music* music) {
    if (!music) return;

    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.music.play_song",
        "Play a specified song ONLINE. Đây là chế độ PHÁT NHẠC MẶC ĐỊNH.\n"
        "Khi người dùng nói: 'phát nhạc', 'mở nhạc', 'phát bài hát', "
        "'play music', 'play song', 'mở bài ...', AI phải ưu tiên dùng tool này.\n"
        "\n"
        "Chỉ dùng SD card nếu người dùng nói rõ: 'nhạc trong thẻ nhớ', "
        "'nhạc offline', 'bài trong thẻ', 'SD card', 'chạy nhạc nội bộ', v.v.\n"
        "\n"
        "Args:\n"
        "  song_name: Tên bài hát (bắt buộc)\n"
        "  artist_name: Tên ca sĩ (tùy chọn)\n"
        "Return:\n"
        "  Phát bài hát online ngay lập tức.\n",
        PropertyList({
            Property("song_name", kPropertyTypeString),
            Property("artist_name", kPropertyTypeString, "")
        }),
        [music](const PropertyList& properties) -> ReturnValue {
            auto song   = properties["song_name"].value<std::string>();
            auto artist = properties["artist_name"].value<std::string>();
            if (!music->Download(song, artist)) {
                return "{\"success\": false, \"message\": \"Failed to get music resource\"}";
            }
            return "{\"success\": true, \"message\": \"Music started playing\"}";
        });

    mcp.AddTool("self.music.set_display_mode",
        "Set the display mode for music playback. You can choose to display spectrum or lyrics.\n"
        "Args:\n"
        "  `mode`: Display mode, options are 'spectrum' or 'lyrics'.\n"
        "Return:\n"
        "  Setting result information.",
        PropertyList({
            Property("mode", kPropertyTypeString)
        }),
        [music](const PropertyList& properties) -> ReturnValue {
            auto mode_str = properties["mode"].value<std::string>();
            std::transform(mode_str.begin(), mode_str.end(), mode_str.begin(), ::tolower);

            auto* esp32_music = static_cast<Esp32Music*>(music);
            if (mode_str == "spectrum") {
                esp32_music->SetDisplayMode(Esp32Music::DISPLAY_MODE_SPECTRUM);
                return "{\"success\": true, \"message\": \"Switched to spectrum display mode\"}";
            } else if (mode_str == "lyrics") {
                esp32_music->SetDisplayMode(Esp32Music::DISPLAY_MODE_LYRICS);
                return "{\"success\": true, \"message\": \"Switched to lyrics display mode\"}";
            }
            return "{\"success\": false, \"message\": \"Invalid display mode, use 'spectrum' or 'lyrics'\"}";
        });
}

/* ------------------------------------------------------------------ */
/*  Radio tools                                                        */
/* ------------------------------------------------------------------ */

void McpFeatureTools::RegisterRadioTools(Esp32Radio* radio) {
    if (!radio) return;

    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.radio.play_station",
        "Play a radio station by name. Use this tool when user requests to play radio or listen to a specific station."
        "VOV mộc/mốc/mốt/mậu/máu/một/mút/mót/mục means VOV1 channel.\n"
        "Args:\n"
        "  `station_name`: The name of the radio station to play (e.g., 'VOV1', 'BBC', 'NPR').\n"
        "Return:\n"
        "  Playback status information.",
        PropertyList({
            Property("station_name", kPropertyTypeString)
        }),
        [radio](const PropertyList& properties) -> ReturnValue {
            auto name = properties["station_name"].value<std::string>();
            if (!radio->PlayStation(name)) {
                return "{\"success\": false, \"message\": \"Failed to find or play radio station: " + name + "\"}";
            }
            return "{\"success\": true, \"message\": \"Radio station " + name + " started playing\"}";
        });

    mcp.AddTool("self.radio.get_stations",
        "Get the list of available radio stations.\n"
        "Return:\n"
        "  JSON array of available radio stations.",
        PropertyList(),
        [radio](const PropertyList& /*properties*/) -> ReturnValue {
            auto stations = radio->GetStationList();
            std::string result = "{\"success\": true, \"stations\": [";
            for (size_t i = 0; i < stations.size(); ++i) {
                result += "\"" + stations[i] + "\"";
                if (i < stations.size() - 1) result += ", ";
            }
            result += "]}";
            return result;
        });
}

#ifdef CONFIG_SD_CARD_ENABLE
/* ------------------------------------------------------------------ */
/*  SD-card Video tools                                               */
/* ------------------------------------------------------------------ */

void McpFeatureTools::RegisterSdVideoTools(VideoPlayer* video) {
    if (!video) return;

    auto& mcp = McpServer::GetInstance();
    /* ---------- 1) Playback control ---------- */
    mcp.AddTool("self.sdvideo.playback",
        "Điều khiển phát video từ THẺ NHỚ (SD card).\n"
        "KHÔNG dùng tool này khi người dùng chỉ nói: 'phát video', 'mở video', "
        "'play video', 'phát clip'.\n"
        "\n"
        "Tool này chỉ dùng khi người dùng nói rõ:\n"
        "- video trong thẻ nhớ / video offline / phát video trong thẻ / SD card\n"
        "\n"
        "action = play | pause | stop | next | prev | shuffle | repeat\n"
        "  play:    bắt đầu phát video\n"
        "  pause:   dừng tạm video\n"
        "  stop:    dừng phát video\n"
        "  next:    phát video tiếp theo\n"
        "  prev:    phát video trước đó\n"
        "  shuffle: phát ngẫu nhiên video từ thẻ\n"
        "  repeat:  lặp lại video hiện tại hoặc toàn bộ danh sách\n"
        "Return: trạng thái điều khiển SD card.\n",
        PropertyList({
            Property("action", kPropertyTypeString),
        }),
        [video](const PropertyList& props) -> ReturnValue {
            std::string action = props["action"].value<std::string>();
            video->SetEndCallback(nullptr); // Clear any existing callback
            if (video->GetPlaylist().empty()) {
                ESP_LOGI(TAG, "[SdVideo] Scanning SD card for video files...");
                return "{\"success\": false, \"message\": \"No video files found on SD card\"}";
            }
            ESP_LOGI(TAG, "[SdVideo] Setting mode: %s", action.c_str());
            if (action == "play") {
                video->Play(video->GetPlaylist()[0].path);
                return "{\"success\": true, \"message\": \"Playing first video\"}";
            } else if (action == "pause") {
                video->Pause();
                return "{\"success\": true, \"message\": \"Video paused\"}";
            } else if (action == "stop") {
                video->Stop();
                return "{\"success\": true, \"message\": \"Video stopped\"}";
            } else if (action == "next") {
                return video->Next() ? "{\"success\": true, \"message\": \"Playing next video\"}"
                                     : "{\"success\": false, \"message\": \"No next video\"}";
            } else if (action == "prev") {
                return video->Prev() ? "{\"success\": true, \"message\": \"Playing previous video\"}"
                                         : "{\"success\": false, \"message\": \"No previous video\"}";
            } else if (action == "shuffle") {
                auto& playlist = video->GetPlaylist();
                int random_idx = rand() % playlist.size();
                video->Play(playlist[random_idx].path);
                return "{\"success\": true, \"message\": \"Shuffle: playing random video\"}";
            } else if (action == "repeat") {
                video->SetEndCallback([video](const std::string& /*finished_path*/) -> bool {
                    return video->Next();
                });
                video->Next();
                return "{\"success\": true, \"message\": \"Repeat mode enabled\"}";
            }
            return "{\"success\": false, \"message\": \"Unknown playback action\"}";
        });

    mcp.AddTool("self.sdvideo.search_play",
        "Search and play video by name from SD card.\n"
        "Args:\n"
        "  `video_name`: The name of the video file to play (e.g., 'demo.avi'). You can also specify part of the name.\n"
        "Return:\n"
        "  Playback status information.",
        PropertyList({
            Property("video_name", kPropertyTypeString)
        }),
        [video](const PropertyList& properties) -> ReturnValue {
            auto video_name = properties["video_name"].value<std::string>();
            video->SetEndCallback(nullptr); // Clear any existing callback
            if (video->GetPlaylist().empty()) {
                ESP_LOGI(TAG, "[SdVideo] Scanning SD card for video files...");
                return "{\"success\": false, \"message\": \"No video files found on SD card\"}";
            }

            // Search playlist for matching video name and play it
            for (const auto& entry : video->GetPlaylist()) {
                if (entry.path.find(video_name) != std::string::npos) {
                    ESP_LOGI(TAG, "[SdVideo] Playing: %s", entry.path.c_str());
                    video->Play(entry.path);
                    return "{\"success\": true, \"message\": \"Video started playing: " + entry.path + "\"}";
                }
            }
            return "{\"success\": false, \"message\": \"No video matching: " + video_name + "\"}";
        });
}

/* ------------------------------------------------------------------ */
/*  SD-card music tools                                               */
/* ------------------------------------------------------------------ */
void McpFeatureTools::RegisterSdMusicTools(Esp32SdMusic* sd_music) {
    if (!sd_music) return;

    auto& mcp = McpServer::GetInstance();

    /* ---------- 1) Playback control ---------- */
    mcp.AddTool("self.sdmusic.playback",
        "Điều khiển phát nhạc từ THẺ NHỚ (SD card).\n"
        "KHÔNG dùng tool này khi người dùng chỉ nói: 'phát nhạc', 'mở bài', "
        "'play music', 'phát bài hát'.\n"
        "\n"
        "Tool này chỉ dùng khi người dùng nói rõ:\n"
        "- nhạc trong thẻ nhớ / nhạc offline / phát bài trong thẻ / SD card\n"
        "\n"
        "action = play | pause | stop | next | prev\n"
        "Return: trạng thái điều khiển SD card.\n",
        PropertyList({
            Property("action", kPropertyTypeString),
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            std::string action = props["action"].value<std::string>();

            if (action == "play") {
                if (sd_music->GetTotalTracks() == 0) {
                    if (!sd_music->LoadPlaylist()) {
                        return "{\"success\": false, \"message\": \"No MP3 files found on SD card\"}";
                    }
                }
                sd_music->SetEndCallback(nullptr); // Clear any existing callback
                bool ok = sd_music->Play();
                return ok ? "{\"success\": true, \"message\": \"Playback started\"}"
                          : "{\"success\": false, \"message\": \"Failed to play\"}";
            }
            if (action == "pause") { sd_music->Pause(); return true; }
            if (action == "stop")  { sd_music->Stop();  return true; }
            if (action == "next")  { return sd_music->Next(); }
            if (action == "prev")  { return sd_music->Prev(); }
            return "{\"success\":false,\"message\":\"Unknown playback action\"}";
        });

    /* ---------- 2) Shuffle / Repeat mode ---------- */
    mcp.AddTool("self.sdmusic.mode",
        "Control playback mode: shuffle and repeat.\n"
        "action = shuffle | repeat\n"
        "For shuffle: `enabled` (bool)\n"
        "For repeat: `mode` = none | one | all",
        PropertyList({
            Property("action",  kPropertyTypeString),
            Property("enabled", kPropertyTypeBoolean),
            Property("mode",    kPropertyTypeString)
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            std::string action = props["action"].value<std::string>();

            if (action == "shuffle") {
                bool enabled = props["enabled"].value<bool>();
                sd_music->SetShuffleMode(enabled);
                if (enabled) {
                    if (sd_music->GetTotalTracks() == 0) sd_music->LoadPlaylist();
                    if (sd_music->GetTotalTracks() == 0) return false;
                    int idx = rand() % sd_music->GetTotalTracks();
                    sd_music->SetTrack(idx);
                }
                return true;
            }
            if (action == "repeat") {
                std::string mode = props["mode"].value<std::string>();
                if (mode == "none")      sd_music->SetRepeatMode(Esp32SdMusic::RepeatMode::None);
                else if (mode == "one")  sd_music->SetRepeatMode(Esp32SdMusic::RepeatMode::RepeatOne);
                else if (mode == "all")  sd_music->SetRepeatMode(Esp32SdMusic::RepeatMode::RepeatAll);
                else return "Invalid repeat mode";
                return true;
            }
            return "Unknown mode action";
        });

    /* ---------- 3) Track operations ---------- */
    mcp.AddTool("self.sdmusic.track",
        "Track-level operations.\n"
        "action = set | info | list | current\n"
        "  set:     needs `index`\n"
        "  info:    needs `index`\n"
        "  list:    return JSON { count }\n"
        "  current: return name string",
        PropertyList({
            Property("action", kPropertyTypeString),
            Property("index",  kPropertyTypeInteger, 0, 0, 9999)
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            std::string action = props["action"].value<std::string>();

            auto ensure_playlist = [sd_music]() {
                if (sd_music->GetTotalTracks() == 0) sd_music->LoadPlaylist();
            };

            if (action == "set") {
                ensure_playlist();
                return sd_music->SetTrack(props["index"].value<int>());
            }
            if (action == "info") {
                ensure_playlist();
                auto info = sd_music->GetTrackInfo(props["index"].value<int>());
                cJSON* json = cJSON_CreateObject();
                cJSON_AddStringToObject(json, "name",    info.name.c_str());
                cJSON_AddStringToObject(json, "path",    info.path.c_str());
                cJSON_AddStringToObject(json, "title",   info.title.c_str());
                cJSON_AddStringToObject(json, "artist",  info.artist.c_str());
                cJSON_AddStringToObject(json, "album",   info.album.c_str());
                cJSON_AddStringToObject(json, "genre",   info.genre.c_str());
                cJSON_AddStringToObject(json, "comment", info.comment.c_str());
                cJSON_AddStringToObject(json, "year",    info.year.c_str());
                cJSON_AddNumberToObject(json, "track_number", info.track_number);
                cJSON_AddNumberToObject(json, "duration_ms",  info.duration_ms);
                cJSON_AddNumberToObject(json, "bitrate_kbps", info.bitrate_kbps);
                cJSON_AddNumberToObject(json, "file_size",    (double)info.file_size);
                cJSON_AddBoolToObject(json,   "has_cover",    info.cover_size > 0);
                cJSON_AddNumberToObject(json, "cover_size",   (int)info.cover_size);
                cJSON_AddStringToObject(json, "cover_mime",   info.cover_mime.c_str());
                return json;
            }
            if (action == "list") {
                ensure_playlist();
                cJSON* o = cJSON_CreateObject();
                cJSON_AddNumberToObject(o, "count", (int)sd_music->GetTotalTracks());
                return o;
            }
            if (action == "current") {
                ensure_playlist();
                return sd_music->GetCurrentTrack();
            }
            return "Unknown track action";
        });

    /* ---------- 4) Directory operations ---------- */
    mcp.AddTool("self.sdmusic.directory",
        "Directory-level operations.\n"
        "action = play | list\n"
        "  play: requires `directory`\n"
        "  list: list directories under current root",
        PropertyList({
            Property("action",    kPropertyTypeString),
            Property("directory", kPropertyTypeString)
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            std::string action = props["action"].value<std::string>();

            if (action == "play") {
                std::string dir = props["directory"].value<std::string>();
                if (!sd_music->PlayDirectory(dir)) {
                    return "{\"success\": false, \"message\": \"Cannot play directory or has no MP3\"}";
                }
                return "{\"success\": true, \"message\": \"Playing directory\"}";
            }
            if (action == "list") {
                cJSON* arr = cJSON_CreateArray();
                auto list = sd_music->GetDirectories();
                for (auto& d : list) {
                    cJSON_AddItemToArray(arr, cJSON_CreateString(d.c_str()));
                }
                return arr;
            }
            return "{\"success\": false, \"message\": \"Unknown directory action\"}";
        });

    /* ---------- 5) Search / Play by name ---------- */
    mcp.AddTool("self.sdmusic.search",
        "Search and play tracks by name.\n"
        "action = search | play\n"
        "  search: returns matching tracks (needs `keyword`)\n"
        "  play:   play by name (needs `keyword`)",
        PropertyList({
            Property("action",  kPropertyTypeString),
            Property("keyword", kPropertyTypeString)
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            std::string action  = props["action"].value<std::string>();
            std::string keyword = props["keyword"].value<std::string>();

            auto ensure_playlist = [sd_music]() {
                if (sd_music->GetTotalTracks() == 0) sd_music->LoadPlaylist();
            };

            if (action == "search") {
                cJSON* arr = cJSON_CreateArray();
                ensure_playlist();
                auto list = sd_music->SearchTracks(keyword);
                for (auto& t : list) {
                    cJSON* o = cJSON_CreateObject();
                    cJSON_AddStringToObject(o, "name",    t.name.c_str());
                    cJSON_AddStringToObject(o, "path",    t.path.c_str());
                    cJSON_AddStringToObject(o, "title",   t.title.c_str());
                    cJSON_AddStringToObject(o, "artist",  t.artist.c_str());
                    cJSON_AddStringToObject(o, "album",   t.album.c_str());
                    cJSON_AddStringToObject(o, "genre",   t.genre.c_str());
                    cJSON_AddStringToObject(o, "year",    t.year.c_str());
                    cJSON_AddNumberToObject(o, "track_number", t.track_number);
                    cJSON_AddNumberToObject(o, "duration_ms",  t.duration_ms);
                    cJSON_AddNumberToObject(o, "bitrate_kbps", t.bitrate_kbps);
                    cJSON_AddBoolToObject(o,   "has_cover",    t.cover_size > 0);
                    cJSON_AddNumberToObject(o, "cover_size",   (int)t.cover_size);
                    cJSON_AddStringToObject(o, "cover_mime",   t.cover_mime.c_str());
                    cJSON_AddItemToArray(arr, o);
                }
                return arr;
            }
            if (action == "play") {
                if (keyword.empty())
                    return "{\"success\": false, \"message\": \"Keyword cannot be empty\"}";
                ensure_playlist();
                bool ok = sd_music->PlayByName(keyword);
                return ok ? "{\"success\": true, \"message\": \"Playing song by name\"}"
                          : "{\"success\": false, \"message\": \"Song not found\"}";
            }
            return "{\"success\": false, \"message\": \"Unknown search action\"}";
        });

    /* ---------- 6) Library / pagination ---------- */
    mcp.AddTool("self.sdmusic.library",
        "Thông tin THƯ VIỆN BÀI HÁT (tracks).\n"
        "action = count_dir | count_current | page\n"
        "  count_dir: đếm SỐ BÀI HÁT trong thư mục chỉ định\n"
        "  count_current: đếm SỐ BÀI HÁT trong thư mục hiện tại\n"
        "  page: phân trang DANH SÁCH BÀI HÁT\n",
        PropertyList({
            Property("action",    kPropertyTypeString),
            Property("directory", kPropertyTypeString),
            Property("page",      kPropertyTypeInteger, 1, 1, 10000),
            Property("page_size", kPropertyTypeInteger, 10, 1, 1000)
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            std::string action = props["action"].value<std::string>();

            auto ensure_playlist = [sd_music]() {
                if (sd_music->GetTotalTracks() == 0) sd_music->LoadPlaylist();
            };

            if (action == "count_dir") {
                std::string dir = props["directory"].value<std::string>();
                cJSON* o = cJSON_CreateObject();
                cJSON_AddStringToObject(o, "directory", dir.c_str());
                cJSON_AddNumberToObject(o, "count", (int)sd_music->GetTrackCountInDir(dir));
                return o;
            }
            if (action == "count_current") {
                ensure_playlist();
                cJSON* o = cJSON_CreateObject();
                cJSON_AddNumberToObject(o, "count", (int)sd_music->GetTrackCount());
                return o;
            }
            if (action == "page") {
                ensure_playlist();
                int page      = props["page"].value<int>();
                int page_size = props["page_size"].value<int>();
                if (page <= 0) page = 1;
                if (page_size <= 0) page_size = 10;

                size_t page_index = (size_t)(page - 1);
                auto list = sd_music->GetPlaylistPage(page_index, (size_t)page_size);
                size_t start_index = page_index * (size_t)page_size;

                cJSON* arr = cJSON_CreateArray();
                for (size_t i = 0; i < list.size(); ++i) {
                    const auto& t = list[i];
                    cJSON* o = cJSON_CreateObject();
                    cJSON_AddNumberToObject(o, "index",        (int)(start_index + i));
                    cJSON_AddStringToObject(o, "name",         t.name.c_str());
                    cJSON_AddStringToObject(o, "path",         t.path.c_str());
                    cJSON_AddStringToObject(o, "title",        t.title.c_str());
                    cJSON_AddStringToObject(o, "artist",       t.artist.c_str());
                    cJSON_AddStringToObject(o, "album",        t.album.c_str());
                    cJSON_AddStringToObject(o, "genre",        t.genre.c_str());
                    cJSON_AddStringToObject(o, "year",         t.year.c_str());
                    cJSON_AddNumberToObject(o, "track_number", t.track_number);
                    cJSON_AddNumberToObject(o, "duration_ms",  t.duration_ms);
                    cJSON_AddNumberToObject(o, "bitrate_kbps", t.bitrate_kbps);
                    cJSON_AddBoolToObject(o,   "has_cover",    t.cover_size > 0);
                    cJSON_AddNumberToObject(o, "cover_size",   (int)t.cover_size);
                    cJSON_AddStringToObject(o, "cover_mime",   t.cover_mime.c_str());
                    cJSON_AddItemToArray(arr, o);
                }
                return arr;
            }
            return "{\"success\": false, \"message\": \"Unknown library action\"}";
        });

    /* ---------- 7) Reload playlist ---------- */
    mcp.AddTool("self.sdmusic.reload",
        "Quét lại toàn bộ thư viện nhạc trong thẻ SD và cập nhật playlist.json.\n"
        "Return: JSON báo thành công / thất bại.",
        PropertyList(),
        [sd_music](const PropertyList&) -> ReturnValue {
            if (sd_music->RebuildPlaylist()) {
                return "{\"success\": false, \"message\": \"Failed to rescan SD card\"}";
            }
            return "{\"success\": true, \"message\": \"SD playlist reloaded\"}";
        });

    /* ---------- 8) Song suggestions ---------- */
    mcp.AddTool("self.sdmusic.suggest",
        "Song suggestion based on history / similarity.\n"
        "action = next | similar\n"
        "  next:    uses `max_results`\n"
        "  similar: uses `keyword` + `max_results`",
        PropertyList({
            Property("action",      kPropertyTypeString),
            Property("keyword",     kPropertyTypeString),
            Property("max_results", kPropertyTypeInteger, 5, 1, 50)
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            cJSON* arr = cJSON_CreateArray();
            std::string action = props["action"].value<std::string>();

            auto ensure_playlist = [sd_music]() {
                if (sd_music->GetTotalTracks() == 0) sd_music->LoadPlaylist();
            };
            ensure_playlist();

            std::string keyword = props["keyword"].value<std::string>();
            int max_results     = props["max_results"].value<int>();
            if (max_results <= 0) max_results = 5;

            auto add_track = [arr](const Esp32SdMusic::TrackInfo& t) {
                cJSON* o = cJSON_CreateObject();
                cJSON_AddStringToObject(o, "name",    t.name.c_str());
                cJSON_AddStringToObject(o, "path",    t.path.c_str());
                cJSON_AddStringToObject(o, "title",   t.title.c_str());
                cJSON_AddStringToObject(o, "artist",  t.artist.c_str());
                cJSON_AddStringToObject(o, "album",   t.album.c_str());
                cJSON_AddStringToObject(o, "genre",   t.genre.c_str());
                cJSON_AddStringToObject(o, "year",    t.year.c_str());
                cJSON_AddNumberToObject(o, "track_number", t.track_number);
                cJSON_AddNumberToObject(o, "duration_ms",  t.duration_ms);
                cJSON_AddNumberToObject(o, "bitrate_kbps", t.bitrate_kbps);
                cJSON_AddBoolToObject(o,   "has_cover",    t.cover_size > 0);
                cJSON_AddNumberToObject(o, "cover_size",   (int)t.cover_size);
                cJSON_AddStringToObject(o, "cover_mime",   t.cover_mime.c_str());
                cJSON_AddItemToArray(arr, o);
            };

            if (action == "next") {
                for (auto& t : sd_music->SuggestNextTracks((size_t)max_results)) add_track(t);
            } else if (action == "similar") {
                for (auto& t : sd_music->SuggestSimilarTo(keyword, (size_t)max_results)) add_track(t);
            }
            return arr;
        });

    /* ---------- 9) Progress ---------- */
    mcp.AddTool("self.sdmusic.progress",
        "Get current playback progress and duration.",
        PropertyList(),
        [sd_music](const PropertyList&) -> ReturnValue {
            cJSON* o = cJSON_CreateObject();
            auto prog  = sd_music->GetProgress();
            auto state = sd_music->GetState();
            int  br    = sd_music->GetBitrate();

            const char* s = "unknown";
            switch (state) {
                case Esp32SdMusic::PlayerState::Stopped:   s = "stopped";   break;
                case Esp32SdMusic::PlayerState::Preparing: s = "preparing"; break;
                case Esp32SdMusic::PlayerState::Playing:   s = "playing";   break;
                case Esp32SdMusic::PlayerState::Paused:    s = "paused";    break;
                case Esp32SdMusic::PlayerState::Error:     s = "error";     break;
            }

            cJSON_AddNumberToObject(o, "position_ms", (int)prog.position_ms);
            cJSON_AddNumberToObject(o, "duration_ms", (int)prog.duration_ms);
            cJSON_AddStringToObject(o, "state",        s);
            cJSON_AddNumberToObject(o, "bitrate_kbps", br);
            cJSON_AddStringToObject(o, "position_str", sd_music->GetCurrentTimeString().c_str());
            cJSON_AddStringToObject(o, "duration_str", sd_music->GetDurationString().c_str());
            cJSON_AddStringToObject(o, "track_name",   sd_music->GetCurrentTrack().c_str());
            cJSON_AddStringToObject(o, "track_path",   sd_music->GetCurrentTrackPath().c_str());
            return o;
        });

    /* ---------- 10) Genre operations ---------- */
    mcp.AddTool("self.sdmusic.genre",
        "Genre-based music operations.\n"
        "action = search | play | play_index | next",
        PropertyList({
            Property("action", kPropertyTypeString),
            Property("genre",  kPropertyTypeString),
            Property("index",  kPropertyTypeInteger, 0, 0, 9999)
        }),
        [sd_music](const PropertyList& props) -> ReturnValue {
            std::string action = props["action"].value<std::string>();
            std::string genre  = props["genre"].value<std::string>();

            auto ensure_playlist = [sd_music]() {
                if (sd_music->GetTotalTracks() == 0) sd_music->LoadPlaylist();
            };
            ensure_playlist();

            auto ascii_lower = [](std::string s) {
                for (char& c : s) {
                    unsigned char u = (unsigned char)c;
                    if (u < 128) c = std::tolower(u);
                }
                return s;
            };

            if (action == "search") {
                cJSON* arr = cJSON_CreateArray();
                if (genre.empty()) return arr;
                auto all  = sd_music->GetPlaylist();
                auto low  = ascii_lower(genre);
                for (auto& t : all) {
                    if (ascii_lower(t.genre).find(low) != std::string::npos) {
                        cJSON* o = cJSON_CreateObject();
                        cJSON_AddStringToObject(o, "name",   t.name.c_str());
                        cJSON_AddStringToObject(o, "path",   t.path.c_str());
                        cJSON_AddStringToObject(o, "artist", t.artist.c_str());
                        cJSON_AddStringToObject(o, "album",  t.album.c_str());
                        cJSON_AddStringToObject(o, "genre",  t.genre.c_str());
                        cJSON_AddNumberToObject(o, "duration_ms", t.duration_ms);
                        cJSON_AddItemToArray(arr, o);
                    }
                }
                return arr;
            }
            if (action == "play") {
                if (genre.empty())
                    return "{\"success\": false, \"message\": \"Genre cannot be empty\"}";
                if (!sd_music->BuildGenrePlaylist(genre))
                    return "{\"success\": false, \"message\": \"No tracks found for this genre\"}";
                bool ok = sd_music->PlayGenreIndex(0);
                return ok ? "{\"success\": true, \"message\": \"Playing first track of genre\"}"
                          : "{\"success\": false, \"message\": \"Failed to play genre\"}";
            }
            if (action == "play_index") {
                bool ok = sd_music->PlayGenreIndex(props["index"].value<int>());
                return ok ? "{\"success\": true, \"message\": \"Playing track in genre list\"}"
                          : "{\"success\": false, \"message\": \"Index invalid or genre list empty\"}";
            }
            if (action == "next") {
                bool ok = sd_music->PlayNextGenre();
                return ok ? "{\"success\": true, \"message\": \"Playing next track in genre\"}"
                          : "{\"success\": false, \"message\": \"No next track or no active genre mode\"}";
            }
            return "{\"success\": false, \"message\": \"Unknown genre action\"}";
        });

    /* ---------- 11) List all genres ---------- */
    mcp.AddTool("self.sdmusic.genre_list",
        "List all unique genres available in the current SD music library.",
        PropertyList(),
        [sd_music](const PropertyList&) -> ReturnValue {
            cJSON* arr = cJSON_CreateArray();
            if (sd_music->GetTotalTracks() == 0) sd_music->LoadPlaylist();
            for (auto& g : sd_music->GetGenres()) {
                cJSON_AddItemToArray(arr, cJSON_CreateString(g.c_str()));
            }
            return arr;
        });
}
#endif // CONFIG_SD_CARD_ENABLE

/* ------------------------------------------------------------------ */
/*  Alarm clock tools                                                  */
/* ------------------------------------------------------------------ */
void McpFeatureTools::RegisterAlarmTools(AlarmManager* alarm_mgr) {
    if (!alarm_mgr) return;

    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.alarm.set",
        "Đặt báo thức mới.\n"
        "Khi người dùng nói: 'đặt báo thức', 'báo thức lúc', 'hẹn giờ', "
        "'nhắc tôi lúc', 'alarm', AI phải dùng tool này.\n"
        "\n"
        "QUAN TRỌNG - Chọn đúng sound_id dựa trên ý định của người dùng:\n"
        "- 'school': khi user nói về đi học, đi làm, đi làm việc, school, work.\n"
        "  Ví dụ: 'báo thức đi học', 'đặt báo thức đi làm', 'alarm school', "
        "'nhắc đi học', 'hẹn giờ đi làm'.\n"
        "- 'wakeup': khi user nói về thức dậy, dậy sáng, wake up, wake.\n"
        "  Ví dụ: 'báo thức thức dậy', 'nhắc dậy', 'wake me up'.\n"
        "- 'medicine': khi user nói về uống thuốc, thuốc, medicine.\n"
        "  Ví dụ: 'báo thức uống thuốc', 'nhắc uống thuốc', 'take medicine'.\n"
        "- 'alarm': mặc định - chuông báo thức thông thường.\n"
        "\n"
        "Nếu user không nói rõ loại âm thanh, dùng 'alarm'.\n"
        "\n"
        "Args:\n"
        "  `hour`: Giờ (0-23), bắt buộc.\n"
        "  `minute`: Phút (0-59), bắt buộc.\n"
        "  `message`: Nội dung nhắc nhở, ví dụ 'đi học', 'uống thuốc'.\n"
        "  `repeated`: Lặp lại hàng ngày hay không (mặc định: false).\n"
        "  `sound_id`: ID âm thanh. Hỗ trợ: 'alarm' (default), 'school', 'wakeup', 'medicine'.\n"
        "Return:\n"
        "  Kết quả đặt báo thức.",
        PropertyList({
            Property("hour",     kPropertyTypeInteger, 0, 0, 23),
            Property("minute",   kPropertyTypeInteger, 0, 0, 59),
            Property("message",  kPropertyTypeString, ""),
            Property("repeated", kPropertyTypeBoolean, false),
            Property("sound_id", kPropertyTypeString, "alarm")
        }),
        [alarm_mgr](const PropertyList& props) -> ReturnValue {
            AlarmManager::Alarm alarm;
            alarm.hour     = props["hour"].value<int>();
            alarm.minute   = props["minute"].value<int>();
            alarm.message  = props["message"].value<std::string>();
            alarm.enabled   = true;
            alarm.repeated  = props["repeated"].value<bool>();
            alarm.sound_id  = props["sound_id"].value<std::string>();

            if (alarm_mgr->isDuplicateAlarm(alarm.hour, alarm.minute)) {
                return "{\"success\": false, \"message\": \"Báo thức đã tồn tại ở giờ này\"}";
            }
            alarm_mgr->addAlarm(alarm);
            cJSON* json = cJSON_CreateObject();
            cJSON_AddBoolToObject(json, "success", true);
            cJSON_AddStringToObject(json, "message", "Đã đặt báo thức thành công");
            char time_buf[16];
            snprintf(time_buf, sizeof(time_buf), "%02d:%02d", alarm.hour, alarm.minute);
            cJSON_AddStringToObject(json, "alarm_time", time_buf);
            return json;
        });

    mcp.AddTool("self.alarm.list",
        "Lấy danh sách tất cả báo thức đang hoạt động.\n"
        "Khi người dùng hỏi: 'có bao nhiêu báo thức', 'xem danh sách báo thức', "
        "'kiểm tra báo thức', AI phải dùng tool này.\n"
        "Return:\n"
        "  Danh sách báo thức.",
        PropertyList(),
        [alarm_mgr](const PropertyList&) -> ReturnValue {
            cJSON* arr = cJSON_CreateArray();
            for (const auto& a : alarm_mgr->getAlarms()) {
                cJSON* o = cJSON_CreateObject();
                char time_buf[16];
                snprintf(time_buf, sizeof(time_buf), "%02d:%02d", a.hour, a.minute);
                cJSON_AddStringToObject(o, "time",      time_buf);
                cJSON_AddStringToObject(o, "message",   a.message.c_str());
                cJSON_AddStringToObject(o, "sound_id",  a.sound_id.c_str());
                cJSON_AddBoolToObject(o,  "enabled",   a.enabled);
                cJSON_AddBoolToObject(o,  "repeated",  a.repeated);
                cJSON_AddItemToArray(arr, o);
            }
            return arr;
        });

    mcp.AddTool("self.alarm.cancel",
        "Hủy báo thức cụ thể.\n"
        "Args:\n"
        "  `hour`: Giờ của báo thức cần hủy (0-23).\n"
        "  `minute`: Phút của báo thức cần hủy (0-59).\n"
        "Return:\n"
        "  Kết quả hủy báo thức.",
        PropertyList({
            Property("hour",   kPropertyTypeInteger, 0, 0, 23),
            Property("minute", kPropertyTypeInteger, 0, 0, 59)
        }),
        [alarm_mgr](const PropertyList& props) -> ReturnValue {
            int hour   = props["hour"].value<int>();
            int minute = props["minute"].value<int>();

            auto alarms = alarm_mgr->getAlarms();
            for (auto it = alarms.begin(); it != alarms.end(); ++it) {
                if (it->hour == hour && it->minute == minute) {
                    alarm_mgr->clearAll();
                    cJSON* json = cJSON_CreateObject();
                    cJSON_AddBoolToObject(json, "success", true);
                    cJSON_AddStringToObject(json, "message", "Đã hủy báo thức thành công");
                    return json;
                }
            }
            return "{\"success\": false, \"message\": \"Không tìm thấy báo thức để hủy\"}";
        });

    mcp.AddTool("self.alarm.cancel_all",
        "Hủy TẤT CẢ báo thức đang hoạt động.\n"
        "Khi người dùng nói: 'hủy tất cả báo thức', 'xóa hết báo thức', "
        "'tắt hết báo thức', AI phải dùng tool này.\n"
        "Return:\n"
        "  Kết quả hủy.",
        PropertyList(),
        [alarm_mgr](const PropertyList&) -> ReturnValue {
            alarm_mgr->clearAll();
            cJSON* json = cJSON_CreateObject();
            cJSON_AddBoolToObject(json, "success", true);
            cJSON_AddStringToObject(json, "message", "Đã hủy tất cả báo thức");
            return json;
        });

    mcp.AddTool("self.alarm.next",
        "Lấy thông tin báo thức tiếp theo sẽ kích hoạt.\n"
        "Return:\n"
        "  Thông tin báo thức gần nhất.",
        PropertyList(),
        [alarm_mgr](const PropertyList&) -> ReturnValue {
            auto info = alarm_mgr->getNextAlarmInfo();
            if (info == "Khong co bao thuc") {
                cJSON* json = cJSON_CreateObject();
                cJSON_AddBoolToObject(json, "success", false);
                cJSON_AddStringToObject(json, "message", "Không có báo thức nào");
                return json;
            }
            cJSON* json = cJSON_CreateObject();
            cJSON_AddBoolToObject(json, "success", true);
            cJSON_AddStringToObject(json, "message", info.c_str());
            return json;
        });

    mcp.AddTool("self.alarm.set_sound",
        "Đặt âm thanh cho một báo thức ĐÃ TỒN TẠI.\n"
        "QUAN TRỌNG: Tool này chỉ dùng để CẬP NHẬT âm thanh cho báo thức đã được "
        "đặt trước đó. Sau khi gọi self.alarm.set, nếu bạn phát hiện user muốn "
        "âm thanh đặc biệt (đi học, thức dậy, uống thuốc), gọi tool này ngay.\n"
        "\n"
        "Ví dụ: User nói 'đặt báo thức 7h đi học' -> gọi self.alarm.set "
        "xong rồi gọi self.alarm.set_sound với sound_id='school'.\n"
        "\n"
        "Args:\n"
        "  `hour`: Giờ của báo thức (0-23).\n"
        "  `minute`: Phút của báo thức (0-59).\n"
        "  `sound_id`: ID âm thanh:\n"
        "    - 'alarm': Chuông báo thức mặc định\n"
        "    - 'school': Đã tới giờ đi học rồi đấy\n"
        "    - 'wakeup': Dậy thôi, sáng rồi!\n"
        "    - 'medicine': Đã tới giờ uống thuốc\n",
        PropertyList({
            Property("hour",    kPropertyTypeInteger, 0, 0, 23),
            Property("minute",  kPropertyTypeInteger, 0, 0, 59),
            Property("sound_id", kPropertyTypeString, "alarm")
        }),
        [alarm_mgr](const PropertyList& props) -> ReturnValue {
            int hour   = props["hour"].value<int>();
            int minute = props["minute"].value<int>();
            std::string sound_id = props["sound_id"].value<std::string>();

            if (FindAlarmSound(sound_id) == nullptr) {
                cJSON* json = cJSON_CreateObject();
                cJSON_AddBoolToObject(json, "success", false);
                cJSON_AddStringToObject(json, "message",
                    ("Sound ID '" + sound_id + "' không hợp lệ. "
                     "Các giá trị: alarm, school, wakeup, medicine").c_str());
                return json;
            }

            if (!alarm_mgr->setAlarmSound(hour, minute, sound_id)) {
                cJSON* json = cJSON_CreateObject();
                cJSON_AddBoolToObject(json, "success", false);
                cJSON_AddStringToObject(json, "message", "Không tìm thấy báo thức để cập nhật");
                return json;
            }

            cJSON* json = cJSON_CreateObject();
            cJSON_AddBoolToObject(json, "success", true);
            cJSON_AddStringToObject(json, "message",
                ("Đã đặt âm thanh '" + sound_id + "' cho báo thức " +
                 std::to_string(hour) + ":" +
                 std::to_string(minute)).c_str());
            return json;
        });

    mcp.AddTool("self.alarm.available_sounds",
        "Lấy danh sách các âm thanh báo thức khả dụng.\n"
        "Return:\n"
        "  Danh sách ID và mô tả của từng âm thanh.",
        PropertyList(),
        [alarm_mgr](const PropertyList&) -> ReturnValue {
            cJSON* arr = cJSON_CreateArray();
            for (size_t i = 0; i < kAlarmSoundCount; i++) {
                cJSON* obj = cJSON_CreateObject();
                cJSON_AddStringToObject(obj, "id", kAlarmSounds[i].id);
                cJSON_AddStringToObject(obj, "name", kAlarmSounds[i].name);
                cJSON_AddItemToArray(arr, obj);
            }
            return arr;
        });
}
