#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <deque>
#include <memory>

#include "protocol.h"
#include "ota.h"
#include "audio_service.h"
#include "device_state_event.h"
#include "esp32_sd_music.h"
#include "esp32_music.h"
#include "esp32_radio.h"
class AudioStreamPlayer;
class VideoPlayer;
class MediaPlayerService;

// Forward declaration for MusicVisualizer (owned by Application)
namespace music { class MusicVisualizer; struct MusicInfo; }
namespace spectrum { class SpectrumManager; }

// --- Display Weather ---
#include "display.h"
#include "features/weather/weather_service.h"
#include "features/weather/weather_model.h"
// ---------------------

#define MAIN_EVENT_SCHEDULE (1 << 0)
#define MAIN_EVENT_SEND_AUDIO (1 << 1)
#define MAIN_EVENT_WAKE_WORD_DETECTED (1 << 2)
#define MAIN_EVENT_VAD_CHANGE (1 << 3)
#define MAIN_EVENT_ERROR (1 << 4)
#define MAIN_EVENT_CHECK_NEW_VERSION_DONE (1 << 5)
#define MAIN_EVENT_CLOCK_TICK (1 << 6)


enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void Start();
    void MainEventLoop();
    DeviceState GetDeviceState() const { return device_state_; }
    bool IsVoiceDetected() const { return audio_service_.IsVoiceDetected(); }
    void Schedule(std::function<void()> callback);
    void SetDeviceState(DeviceState state);
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    bool UpgradeFirmware(Ota& ota, const std::string& url = "");
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void SetAecMode(AecMode mode);
    AecMode GetAecMode() const { return aec_mode_; }
    // 新增：接收外部音频数据（如音乐播放）
    void AddAudioData(AudioStreamPacket&& packet);
    void PlaySound(const std::string_view& sound);
    AudioService& GetAudioService() { return audio_service_; }
	Esp32Music* GetMusic() { return music_; }
	Esp32Radio* GetRadio() { return radio_; }
	Esp32SdMusic* GetSdMusic() { return sd_music_; }
	VideoPlayer* GetVideo() { return sd_video_; }

    /** Get the music visualizer (owned by Application). */
    music::MusicVisualizer* GetMusicVisualizer() { return music_visualizer_.get(); }

    /* ================================================================== */
    /*  Media Player APIs                                                 */
    /* ================================================================== */

    /**
     * @brief Play online music by song name.
     * @param song_name   Song name to search for
     * @param artist_name Optional artist name filter
     * @return true if playback started
     */
    bool PlayMusic(const std::string& song_name, const std::string& artist_name = "");

    /**
     * @brief Play a radio station by name.
     * @param station_name Station name or key (e.g. "VOV1")
     * @return true if playback started
     */
    bool PlayRadio(const std::string& station_name);

    /**
     * @brief Play radio from custom URL.
     * @param url          Stream URL
     * @param station_name Optional display name
     * @return true if playback started
     */
    bool PlayRadioUrl(const std::string& url, const std::string& station_name = "");

    /**
     * @brief Play media from SD card (music or video).
     * @param keyword  File name or search keyword
     * @param is_video true to play as AVI video, false for audio
     * @return true if playback started
     */
    bool PlaySdMedia(const std::string& keyword, bool is_video = false);

    /**
     * @brief Play AVI video from SD card by full path.
     * @param file_path Absolute path to AVI file
     * @return true if playback started
     */
    bool PlayVideo(const std::string& file_path);

    /**
     * @brief Stop all media playback (music, radio, SD music, video).
     */
    void StopAllMedia();

    /**
     * @brief Check if any media is currently playing.
     */
    bool IsMediaPlaying() const;

    /**
     * @brief Ensure device is in idle state before media playback.
     *
     * If the device is in Listening or Speaking state, toggles the chat
     * to transition back to Idle.  Blocks until the transition completes
     * (up to a configurable timeout).
     *
     * Must be called from outside the audio player — keeps media
     * components decoupled from Application state management.
     *
     * @return true if device is now in idle (or was already idle)
     */
    bool EnsureIdleForMedia();

    /**
     * @brief Setup FFT display callback for a given audio player.
     * Installs FFT data callback and state callback for FFT lifecycle.
     */
    void SetupAudioPlayerCallback(AudioStreamPlayer* player);

    /**
     * @brief Build a MusicInfo snapshot by auto-detecting the active player.
     * Called by MusicVisualizer's periodic callback to update the UI.
     */
    music::MusicInfo BuildMusicInfo();

    /* ================================================================== */
    /*  Component Initializers                                            */
    /* ================================================================== */

    /** Initialize online music player and register MCP tools. */
    bool InitMusic();

    /** Initialize internet radio player and register MCP tools. */
    bool InitRadio();

    /** Initialize SD card music player and register MCP tools. */
    bool InitSdMusic();

    /** Initialize SD card video player and register MCP tools. */
    bool InitVideo();

    /** Initialize media player service (MP4/audio/video/HTTP) and register MCP tools. */
    bool InitMedia();

private:
    Application();
    ~Application();

    std::mutex mutex_;
    std::deque<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    volatile DeviceState device_state_ = kDeviceStateUnknown;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
    AecMode aec_mode_ = kAecOff;
    std::string last_error_message_;
    AudioService audio_service_;
    Esp32Music* music_ = nullptr;
    Esp32Radio* radio_ = nullptr;
    Esp32SdMusic* sd_music_ = nullptr;
    VideoPlayer* sd_video_ = nullptr;

    // Music spectrum visualizer (owned by Application, not Display)
    std::unique_ptr<music::MusicVisualizer> music_visualizer_;

    // OLED spectrum (simple bars, no music UI overlay)
    std::unique_ptr<spectrum::SpectrumManager> oled_spectrum_mgr_;

    bool has_server_time_ = false;
    bool aborted_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;
    TaskHandle_t main_event_loop_task_handle_ = nullptr;

    /**
     * @brief Identifies which media component to exclude from stopping.
     * Used by StopOtherMedia() to skip the component about to play.
     */
    enum class MediaComponent : uint8_t {
        kNone     = 0,   ///< Stop all media (no exclusion)
        kMusic    = 1,   ///< Keep music, stop everything else
        kRadio    = 2,   ///< Keep radio, stop everything else
        kSdMusic  = 3,   ///< Keep SD music, stop everything else
        kVideo    = 4,   ///< Keep video, stop everything else
        kMedia    = 5,   ///< Keep MediaPlayerService, stop everything else
    };

    /**
     * @brief Stop all active media playback except the specified component.
     *
     * Centralized media teardown: conditionally stops music, radio,
     * SD music and video. Each component is stopped only when currently
     * active. Call with kNone (default) to stop everything.
     *
     * @param except  Component to skip (default: kNone = stop all)
     */
    void StopOtherMedia(MediaComponent except = MediaComponent::kNone);

    void OnWakeWordDetected();
    void CheckNewVersion(Ota& ota);
    void CheckAssetsVersion();
    void ShowActivationCode(const std::string& code, const std::string& message);
    void SetListeningMode(ListeningMode mode);

#ifdef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    // --- Weather Info ---
    void UpdateIdleDisplay();
    // -------------------
#endif
};


class TaskPriorityReset {
public:
    TaskPriorityReset(BaseType_t priority) {
        original_priority_ = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, priority);
    }
    ~TaskPriorityReset() {
        vTaskPrioritySet(NULL, original_priority_);
    }

private:
    BaseType_t original_priority_;
};

#endif // _APPLICATION_H_
