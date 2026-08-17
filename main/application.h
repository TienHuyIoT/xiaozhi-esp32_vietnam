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
#include <cstdint>
#include <atomic>
#include <array>

#include "protocol.h"
#include "ota.h"
#include "audio_service.h"
#include "protocols/device_tts_client.h"
#include "device_state_event.h"
#include "esp32_sd_music.h"
#include "esp32_music.h"
#include "esp32_radio.h"
class AudioStreamPlayer;
class VideoPlayer;
class AlarmManager;

// Forward declaration for MusicVisualizer (owned by Application)
namespace music { class MusicVisualizer; struct MusicInfo; }
namespace spectrum { class SpectrumManager; }

// --- Display Weather ---
#include "display.h"
#include "features/weather/weather_service.h"
#include "features/weather/weather_model.h"
// ---------------------
#include "features/sd_media_manager.h"

#define MAIN_EVENT_SCHEDULE (1 << 0)
#define MAIN_EVENT_SEND_AUDIO (1 << 1)
#define MAIN_EVENT_WAKE_WORD_DETECTED (1 << 2)
#define MAIN_EVENT_VAD_CHANGE (1 << 3)
#define MAIN_EVENT_ERROR (1 << 4)
#define MAIN_EVENT_CHECK_NEW_VERSION_DONE (1 << 5)
#define MAIN_EVENT_CLOCK_TICK (1 << 6)
#define MAIN_EVENT_AUDIO_PLAYBACK_STATE (1 << 7)
#define MAIN_EVENT_SERVER_OPUS_FINISHED (1 << 8)


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
    /**
     * @param source Ten duong goi, in ra log duoi khoa `abort_src=`. BAT BUOC,
     *        khong co mac dinh: do 17/08 co 3 dong `Abort speaking` ma chi truy
     *        duoc nguon cua 2, vi hai cho goi khong ghi gi. Thieu tham so nay
     *        thi khong bien dich duoc -- chac hon quy uoc "nho ghi log".
     */
    void AbortSpeaking(AbortReason reason, const char* source);
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
    SdMediaManager* GetSdMediaManager() { return sd_media_manager_; }

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

    /** Initialize SD media manager and register MCP tools. */
    bool InitSdMediaManager();

    /** Initialize alarm clock manager and register MCP tools. */
    bool InitAlarm();

    /** Get the alarm manager instance. */
    AlarmManager* GetAlarmManager() const { return alarm_manager_; }

private:
    enum class SpeechAudioSource : uint8_t {
        kNone = 0,
        kSwitching,
        kServerOpus,
        kDeviceTts,
    };

    struct SpeechAudioLease {
        SpeechAudioSource source = SpeechAudioSource::kNone;
        std::string turn_id;
        uint32_t generation = 0;
    };

    enum class AudioPlaybackStateKind : uint8_t {
        kQueued = 0,
        kStarted,
        kFinished,
        kAborted,
        kDroppedStale,
    };

    static constexpr size_t kAudioPlaybackFieldCapacity = 65;
    struct AudioPlaybackState {
        AudioPlaybackStateKind state = AudioPlaybackStateKind::kQueued;
        std::array<char, kAudioPlaybackFieldCapacity> turn_id{};
        std::array<char, kAudioPlaybackFieldCapacity> segment_id{};
        std::array<char, kAudioPlaybackFieldCapacity> audio_source{};
        uint32_t generation = 0;
        uint32_t session_epoch = 0;
    };

    static constexpr size_t kAudioPlaybackStateRingCapacity = 16;

    /**
     * Watchdog thoat Speaking. Dem GIAY KHONG CO TIEN TRIEN, khong phai tong
     * thoi gian noi -- mot luot bai hoc dai la binh thuong, cat no la lam hong.
     *
     * Log som (5s) vi muc dich chinh cua watchdog nay la BANG CHUNG: review
     * 16/08 dem duoc nam chot noi tiep trong `CheckSpeakingFinished`, va hai
     * lan va truoc do deu phai doc lai ma nguon vi serial khong noi duoc guard
     * nao dang giu. Cuong buc muon (20s) vi phai cho DeviceTtsClient het han
     * cua no truoc: `kNoAudioTimeoutMs` 7s va `kTotalTimeoutMinMs` 15s.
     */
    static constexpr int kSpeakingStallLogSeconds = 5;
    static constexpr int kSpeakingStallTimeoutSeconds = 20;

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
    // TTS tren thiet bi: khi server gui `tts_config` + `tts_body` thi robot tu
    // lay tieng qua day thay vi phat khung Opus cua server. nullptr = chua bat.
    std::unique_ptr<DeviceTtsClient> device_tts_client_;
    // Chi mot duong speech duoc so huu loa. transition_mutex_ serialize toan
    // bo revoke/drain/commit; state mutex chi bao ve lease doc nhanh tu callback.
    std::mutex speech_audio_transition_mutex_;
    std::mutex speech_audio_mutex_;
    SpeechAudioSource speech_audio_source_ = SpeechAudioSource::kNone;
    std::string speech_audio_turn_id_;
    uint32_t speech_audio_generation_ = 0;
    // Observer playback chi day metadata vao ring co tran; main task moi gui WS.
    std::atomic<bool> playback_ack_enabled_{false};
    std::atomic<uint32_t> playback_ack_session_epoch_{0};
    std::atomic<uint32_t> next_server_opus_trace_sequence_{1};
    std::atomic<uint32_t> server_opus_completed_generation_{0};
    std::mutex audio_playback_state_mutex_;
    std::array<AudioPlaybackState, kAudioPlaybackStateRingCapacity>
        audio_playback_state_ring_{};
    size_t audio_playback_state_head_ = 0;
    size_t audio_playback_state_count_ = 0;
    std::atomic<uint32_t> audio_playback_state_drop_count_{0};
    std::mutex audio_trace_mutex_;
    AudioTraceContext active_audio_trace_;
    std::string audio_trace_source_ = "none";
    // Server gui `tts:stop` NGAY sau cau cuoi (no khong con nhin thay tieng nua)
    // -> phai nho co nay va doi DeviceTtsClient bao het viec moi doi trang thai.
    bool tts_stop_received_ = false;
    // Anh chup tien trien cua lan tick truoc, chi main task doc/ghi. Ba tin
    // hieu doc lap: PCM da phat them chua, byte cua nha cung cap con ve khong,
    // hai co busy co doi khong. Bat cu cai nao doi = con song, dem lai tu 0.
    int speaking_stall_ticks_ = 0;
    int64_t speaking_stall_play_time_ms_ = -1;
    size_t speaking_stall_buffer_bytes_ = 0;
    bool speaking_stall_device_tts_busy_ = false;
    bool speaking_stall_opus_busy_ = false;
    Esp32Music* music_ = nullptr;
    Esp32Radio* radio_ = nullptr;
    Esp32SdMusic* sd_music_ = nullptr;
    VideoPlayer* sd_video_ = nullptr;
    SdMediaManager* sd_media_manager_ = nullptr;
    AlarmManager* alarm_manager_ = nullptr;

    // Music spectrum visualizer (owned by Application, not Display)
    std::unique_ptr<music::MusicVisualizer> music_visualizer_;

    // OLED spectrum (simple bars, no music UI overlay)
    std::unique_ptr<spectrum::SpectrumManager> oled_spectrum_mgr_;

    std::string pending_media_url_;
    std::string pending_media_title_;

    bool has_server_time_ = false;
    bool aborted_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;
    TaskHandle_t main_event_loop_task_handle_ = nullptr;
#ifdef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    TaskHandle_t weather_idle_task_handle_ = nullptr;
#endif

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
    SpeechAudioLease TransitionSpeechAudio(const AudioTraceContext& trace);
    bool IsSpeechAudioLeaseCurrent(const SpeechAudioLease& lease);
    bool AcceptServerOpusPacket(std::unique_ptr<AudioStreamPacket> packet);
    bool EnqueueDeviceTtsIfLeaseCurrent(
        const SpeechAudioLease& lease,
        const std::string& body,
        const AudioTraceContext& trace);
    void ObserveAudioTrace(const char* event, const char* action,
                           const AudioTraceContext& trace);
    void DrainAudioPlaybackStates();
    void HandleServerOpusPlaybackFinished();
    void RevokeSpeechAudio();
    void ResetDecoderBeforeSpeakingIfUnowned();
    void CheckNewVersion(Ota& ota);
    void CheckAssetsVersion();
    /**
     * Chuyen Speaking->Listening/Idle khi VA CHI KHI ca hai dieu kien du:
     * server da gui `tts:stop` VA DeviceTtsClient da phat het tieng.
     * Go ham nay = cau cuoi cua moi luot free chat bi cat cut.
     */
    void CheckSpeakingFinished();
    /**
     * Luoi cuoi cho ca LOP loi "ket Speaking": `CheckSpeakingFinished` co nam
     * guard noi tiep nhung chi duoc goi tu ba diem su kien, nen guard nao
     * khong nha thi khong con ai hoi lai. Ham nay chay moi giay, log dich danh
     * guard dang chan, va cuong buc roi Speaking khi server da het luot ma
     * robot dung yen qua lau.
     */
    void CheckSpeakingStall();
    void ShowActivationCode(const std::string& code, const std::string& message);
    void SetListeningMode(ListeningMode mode);

#ifdef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    // --- Weather Info ---
    void StartWeatherIdleTask();
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
