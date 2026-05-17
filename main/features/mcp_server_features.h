/**
 * @file mcp_server_features.h
 * @brief MCP tool registrations for media feature modules.
 *
 * Each static method receives a component pointer and registers
 * the appropriate MCP tools, capturing the pointer in lambdas.
 * This avoids calling Application::GetInstance() from tool handlers.
 */
#ifndef MCP_SERVER_FEATURES_H
#define MCP_SERVER_FEATURES_H

#include <functional>

class Esp32Music;
class Esp32Radio;
class Esp32SdMusic;
class VideoPlayer;
class AlarmManager;

/**
 * @brief Static helper that registers MCP tools per media component.
 *
 * Call individual methods from Application::InitXxx() after the
 * component is created and initialized.
 */
class McpFeatureTools {
public:
    /**
     * @brief Network / QR code tool.
     *
     * @param overlay_cb  Optional callback fired with `true` when the QR
     *                    canvas appears and `false` when it is cleared.
     *                    Use it to hide/restore the host display's normal UI.
     */
    static void RegisterIp2QrCodeTool(
        std::function<void(bool)> overlay_cb = nullptr);

    /** Online music tools (play_song, set_display_mode). */
    static void RegisterMusicTools(Esp32Music* music);

    /** Radio tools (play_station, play_url, stop, get_stations, set_display_mode). */
    static void RegisterRadioTools(Esp32Radio* radio);

    /** SD card music tools (playback, mode, track, directory, search, etc.). */
    static void RegisterSdMusicTools(Esp32SdMusic* sd_music);

    /** SD card video tools (play_video). */
    static void RegisterSdVideoTools(VideoPlayer* video);

    /** Alarm clock tools (set, list, cancel, next). */
    static void RegisterAlarmTools(AlarmManager* alarm_mgr);

private:
    McpFeatureTools() = delete;
};

#endif // MCP_SERVER_FEATURES_H
