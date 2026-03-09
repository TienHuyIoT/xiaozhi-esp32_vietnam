#ifndef MEDIA_PLAYER_SERVICE_H
#define MEDIA_PLAYER_SERVICE_H

/**
 * @file media_player_service.h
 * @brief Thread-safe wrapper around tempotian/media_player component.
 *
 * Isolates third-party media_player API behind a clean internal interface.
 * All control commands are serialized through a FreeRTOS queue to guarantee
 * thread safety when called from multiple tasks (UI, MCP, application).
 *
 * Supports three rendering modes (MediaRenderMode):
 *   1. kDirectLcd:  Hardware av_render — I2S audio + direct LCD panel write.
 *   2. kLvglCanvas: I2S audio + LVGL canvas video rendering (like VideoPlayer).
 *   3. kCallback:   Forward decoded frames to user callbacks (AVI-style).
 *
 * Architecture:
 *   - Command path: public API -> command queue -> worker task -> media_player
 *   - Event path:   media_player callback -> event dispatch -> app listeners
 *   - Frame path:   media_player -> av_render -> render (hw/canvas/callback)
 *   - Lifecycle:    Init() -> SetSource() -> Play() -> Stop() -> Deinit()
 */

#include <string>
#include <functional>
#include <atomic>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

/* ------------------------------------------------------------------ */
/*  Forward declarations -- avoid exposing third-party headers        */
/* ------------------------------------------------------------------ */
typedef void* media_player_handle_t;
typedef void* audio_render_handle_t;
typedef void* video_render_handle_t;

class AudioCodec;
class Display;
class MediaVideoRenderer;
class MediaRenderCallback;

struct esp_lcd_panel_t;
typedef struct esp_lcd_panel_t* esp_lcd_panel_handle_t;

/* ------------------------------------------------------------------ */
/*  Public enums                                                      */
/* ------------------------------------------------------------------ */

/** Player state visible to the rest of the application. */
enum class MediaPlayerState : uint8_t {
    kIdle = 0,       ///< 0 Not initialized or stopped
    kConnecting,     ///< 1 Source connecting (network only)
    kPrepared,       ///< 2 Source parsed, ready to play
    kPlaying,        ///< 3 Active playback
    kPaused,         ///< 4 Paused (can resume)
    kStopped,        ///< 5 Stopped (can set new source)
    kError,          ///< 6 Error occurred
};

/** Source type for media content. */
enum class MediaSourceType : uint8_t {
    kFile = 0,       ///< 0 Local file (SD card, flash)
    kHttp,           ///< 1 HTTP/HTTPS network stream
};

/** Player events forwarded to application listeners. */
enum class MediaPlayerEvent : uint8_t {
    kSourceConnecting = 0,
    kSourceConnected,  ///< 1 Source connection established (network only) or file opened
    kPrepared,         ///< 2 Source parsed and ready to play (codecs identified, first frames decoded)
    kPlayStarted,      ///< 3 First frame rendered and playback started (after prepared)
    kSeekDone,         ///< 4 Seek operation completed (if supported by player, otherwise not used)
    kEndOfStream,      ///< 5 Playback reached end of stream
    kPlayError,        ///< 6 Playback error occurred
    kStateChanged,     ///< 7 Player state changed
};

/** Callback signature for player events. */
using MediaPlayerEventCallback = std::function<void(MediaPlayerEvent event,
                                                     MediaPlayerState state)>;

/**
 * @brief Video rendering strategy for MediaPlayerService.
 *
 * Mirrors VideoPlayer's VideoRenderMode concept:
 *   - kDirectLcd:  Hardware av_render — I2S audio + direct LCD panel write
 *                  (bypasses LVGL for maximum throughput).
 *   - kLvglCanvas: I2S audio + LVGL canvas video rendering — goes through
 *                  LVGL's refresh pipeline (supports UI overlays on video).
 *   - kCallback:   Forward decoded audio/video frames to user callbacks
 *                  for external rendering (similar to AVI player callbacks).
 */
enum class MediaRenderMode : uint8_t {
    kDirectLcd = 0,   ///< Hardware I2S audio + direct LCD panel video
    kLvglCanvas,      ///< Hardware I2S audio + LVGL canvas video
    kCallback,        ///< Forward all frames to user callbacks
};

/* ------------------------------------------------------------------ */
/*  Render callbacks (AVI-player style)                               */
/* ------------------------------------------------------------------ */

/**
 * Called when decoded audio PCM data is available for rendering.
 * Similar to AVI player's audio_cb.
 *
 * @param data      Decoded PCM audio data
 * @param size      Data size in bytes
 * @param pts_ms    Presentation timestamp in milliseconds
 * @param user_data User context pointer
 *
 * @note Called from media_player's audio render thread. Keep non-blocking.
 */
typedef void (*media_audio_frame_cb_t)(const uint8_t* data, int size,
                                       uint32_t pts_ms, void* user_data);

/**
 * Called when audio format is determined (before first audio frame).
 * Similar to AVI player's audio_set_clock_cb.
 *
 * @param sample_rate   Audio sample rate in Hz
 * @param bits_per_sample  Bits per sample (8/16/24/32)
 * @param channels      Number of audio channels
 * @param user_data     User context pointer
 */
typedef void (*media_audio_set_clock_cb_t)(uint32_t sample_rate,
                                           uint8_t bits_per_sample,
                                           uint8_t channels,
                                           void* user_data);

/**
 * Called when decoded video frame data is available for rendering.
 * Similar to AVI player's video_cb.
 *
 * @param data      Decoded video frame data (RGB565 or other format)
 * @param size      Data size in bytes
 * @param width     Frame width in pixels
 * @param height    Frame height in pixels
 * @param pts_ms    Presentation timestamp in milliseconds
 * @param user_data User context pointer
 *
 * @note Called from media_player's video render thread. Keep non-blocking.
 */
typedef void (*media_video_frame_cb_t)(const uint8_t* data, int size,
                                       uint16_t width, uint16_t height,
                                       uint32_t pts_ms, void* user_data);

/**
 * Called when video format info is determined (before first video frame).
 *
 * @param width       Video width in pixels
 * @param height      Video height in pixels
 * @param fps         Frames per second
 * @param frame_type  Decoded frame pixel type (RGB565, YUV420, etc.)
 * @param user_data   User context pointer
 */
typedef void (*media_video_set_info_cb_t)(uint16_t width, uint16_t height,
                                          uint8_t fps, uint8_t frame_type,
                                          void* user_data);

/**
 * Called when playback reaches end of stream.
 * Similar to AVI player's avi_play_end_cb.
 *
 * @param user_data User context pointer
 */
typedef void (*media_play_end_cb_t)(void* user_data);

/* ------------------------------------------------------------------ */
/*  Configuration                                                     */
/* ------------------------------------------------------------------ */

/** FIFO tuning preset for memory/latency tradeoff. */
struct MediaFifoConfig {
    uint32_t adec_fifo_size      = 8192;        ///< Audio decoder buffer (0 = library default)
    uint32_t vdec_fifo_size      = 16384;       ///< Video decoder buffer (0 = library default)
    uint32_t arender_fifo_size   = 4096;        ///< Audio render buffer (0 = library default)
    uint32_t vrender_fifo_size   = 8192;        ///< Video render buffer (0 = library default)
    uint32_t extractor_pool_size = 2 * 1024 * 1024;  ///< Extractor output pool (bytes, default 2MB)
};

/**
 * Render callback configuration.
 * When callbacks are set, decoded frames are forwarded to the user
 * instead of being rendered internally (similar to AVI player callbacks).
 * All callbacks share a single user_data context pointer.
 */
struct MediaRenderCallbacks {
    media_audio_frame_cb_t     audio_cb        = nullptr;  ///< Decoded audio PCM data
    media_audio_set_clock_cb_t audio_clock_cb   = nullptr;  ///< Audio format notification
    media_video_frame_cb_t     video_cb        = nullptr;  ///< Decoded video frame data
    media_video_set_info_cb_t  video_info_cb    = nullptr;  ///< Video format notification
    media_play_end_cb_t        play_end_cb     = nullptr;  ///< Playback end notification
    void*                      user_data       = nullptr;  ///< Shared context for all callbacks

    /** @return true if any audio callback is registered. */
    bool HasAudioCallbacks() const { return audio_cb != nullptr; }
    /** @return true if any video callback is registered. */
    bool HasVideoCallbacks() const { return video_cb != nullptr; }
};

/** Initialization config for the media player service. */
struct MediaPlayerConfig {
    bool enable_audio         = true;    ///< Enable audio playback
    bool enable_video         = false;   ///< Enable video playback
    bool fast_seek            = true;    ///< Non-accurate seek for lower latency
    MediaRenderMode render_mode = MediaRenderMode::kDirectLcd;  ///< Rendering strategy
    MediaFifoConfig fifo;                ///< FIFO tuning preset
    MediaRenderCallbacks callbacks;      ///< Render callbacks (only for kCallback mode)
};

/* ------------------------------------------------------------------ */
/*  MediaPlayerService                                                */
/* ------------------------------------------------------------------ */

class MediaPlayerService {
public:
    static MediaPlayerService& GetInstance();

    // Delete copy/move
    MediaPlayerService(const MediaPlayerService&) = delete;
    MediaPlayerService& operator=(const MediaPlayerService&) = delete;

    /* ---- Lifecycle ---- */

    /**
     * @brief Initialize player with full hardware and display parameters.
     *
     * Mirrors VideoPlayer::Initialize() signature.  Supports all three
     * render modes selected via config.render_mode:
     *   - kDirectLcd:  Uses codec for I2S audio, panel for direct LCD video.
     *   - kLvglCanvas: Uses codec for I2S audio, creates LVGL canvas for video
     *                  (requires display, lcd_width, lcd_height).
     *   - kCallback:   Forwards frames to config.callbacks (codec/panel unused).
     *
     * @param codec      AudioCodec instance (for I2S render; nullptr for callback mode)
     * @param panel      LCD panel handle (for direct LCD; nullptr for audio-only)
     * @param lcd_width  Display width in pixels (required for kLvglCanvas)
     * @param lcd_height Display height in pixels (required for kLvglCanvas)
     * @param display    Display instance for LVGL lock (required for kLvglCanvas)
     * @param config     Player configuration including render mode
     * @return true on success
     */
    bool Init(AudioCodec* codec, esp_lcd_panel_handle_t panel,
              uint16_t lcd_width, uint16_t lcd_height,
              Display* display = nullptr,
              const MediaPlayerConfig& config = {});

    /**
     * @brief Initialize player with callback rendering only.
     *
     * Convenience overload for kCallback mode without hardware handles.
     * config.render_mode is forced to kCallback.
     *
     * @param config Player configuration (must include callbacks)
     * @return true on success
     */
    bool Init(const MediaPlayerConfig& config = {});

    /**
     * @brief Release all resources and stop worker task.
     */
    void Deinit();

    /** @return true if Init() succeeded and Deinit() not yet called. */
    bool IsInitialized() const { return initialized_.load(); }

    /* ---- Source ---- */

    /**
     * @brief Set media source URI.
     * @param type  File or HTTP
     * @param uri   Path or URL (e.g. "/sdcard/music.mp3" or "http://...")
     * @return true if command enqueued
     */
    bool SetSource(MediaSourceType type, const std::string& uri);

    /* ---- Playback control (all thread-safe) ---- */

    bool Play();
    bool Pause();
    bool Resume();
    bool Stop();

    /**
     * @brief Seek to position.
     * @param time_ms  Target position in milliseconds
     * @return true if command enqueued
     */
    bool Seek(uint32_t time_ms);

    /**
     * @brief Set playback speed.
     * @param speed  0.5 = half, 1.0 = normal, 2.0 = double
     */
    bool SetSpeed(float speed);

    /**
     * @brief Enable/disable loop playback.
     */
    bool SetLoop(bool enable);

    /* ---- Status queries (lock-free reads) ---- */

    MediaPlayerState GetState() const { return state_.load(); }
    bool IsPlaying() const { return state_.load() == MediaPlayerState::kPlaying; }

    /**
     * @brief Get current playback position.
     * @return Position in milliseconds, or -1 on error
     */
    int64_t GetPosition() const;

    /**
     * @brief Get total media duration.
     * @return Duration in milliseconds, or -1 if unknown
     */
    int64_t GetDuration() const;

    /* ---- Event listener ---- */

    /**
     * @brief Register event callback for state/error notifications.
     * Callback is invoked from the worker task context (non-ISR safe).
     * Keep handlers lightweight; post heavy work to another task.
     */
    void SetEventCallback(MediaPlayerEventCallback cb);

    /* ---- Render callbacks access (used by callback renders) ---- */

    /** @return Current render callback configuration. */
    const MediaRenderCallbacks& GetCallbacks() const { return config_.callbacks; }

private:
    MediaPlayerService();
    ~MediaPlayerService();

    /* ---- Internal command queue ---- */
    enum class CmdType : uint8_t {
        kSetSource = 0,
        kPlay,
        kPause,
        kResume,
        kStop,
        kSeek,
        kSetSpeed,
        kSetLoop,
        kDeinit,
    };

    struct Command {
        CmdType type;
        union {
            struct { MediaSourceType src_type; } source;
            struct { uint32_t time_ms; }         seek;
            struct { float speed; }              speed;
            struct { bool enable; }              loop;
        } params;
        char uri[256];   ///< Used only for kSetSource
    };

    bool EnqueueCommand(const Command& cmd);
    void ProcessCommand(const Command& cmd);

    /* ---- Worker task ---- */
    static void WorkerTaskEntry(void* arg);
    void WorkerLoop();

    /* ---- Player callback (called by media_player lib) ---- */
    static int PlayerCallbackTrampoline(int event, void* ctx);
    void HandlePlayerEvent(int event);

    /* ---- Internal init ---- */
    bool InitInternal(AudioCodec* codec, esp_lcd_panel_handle_t panel,
                      uint16_t lcd_width, uint16_t lcd_height,
                      Display* display, const MediaPlayerConfig& config);

    /* ---- State management ---- */
    void SetState(MediaPlayerState new_state);

    /* ---- Members ---- */
    std::atomic<bool>              initialized_{false};
    std::atomic<MediaPlayerState>  state_{MediaPlayerState::kIdle};
    media_player_handle_t          player_{nullptr};
    audio_render_handle_t          audio_render_{nullptr};
    video_render_handle_t          video_render_{nullptr};
    MediaPlayerConfig              config_{};
    MediaPlayerEventCallback       event_callback_;
    MediaPlayerEvent               mapped_event_ = MediaPlayerEvent::kStateChanged;

    /* Hardware / display handles (stored for queries and LVGL canvas) */
    Display*               display_{nullptr};
    uint16_t               lcd_width_{0};
    uint16_t               lcd_height_{0};

    /* Internal LVGL canvas renderer (owned, only for kLvglCanvas mode) */
    MediaVideoRenderer*    internal_renderer_{nullptr};

    /* Callback render handler (owned, only for kCallback mode with hardware) */
    MediaRenderCallback*   render_callback_{nullptr};

    QueueHandle_t    cmd_queue_{nullptr};
    TaskHandle_t     worker_task_{nullptr};
    SemaphoreHandle_t callback_mutex_{nullptr};

    static constexpr int kCmdQueueLen     = 16;
    static constexpr int kWorkerStackSize = 6 * 1024;
    static constexpr int kWorkerPriority  = 5;
    static constexpr int kWorkerCore      = 0;
};

#endif // MEDIA_PLAYER_SERVICE_H
