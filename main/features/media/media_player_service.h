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
 * Architecture:
 *   - Command path: public API -> command queue -> worker task -> media_player
 *   - Event path:   media_player callback -> event dispatch -> app listeners
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

/* ------------------------------------------------------------------ */
/*  Public enums                                                      */
/* ------------------------------------------------------------------ */

/** Player state visible to the rest of the application. */
enum class MediaPlayerState : uint8_t {
    kIdle = 0,       ///< Not initialized or stopped
    kConnecting,     ///< Source connecting (network only)
    kPrepared,       ///< Source parsed, ready to play
    kPlaying,        ///< Active playback
    kPaused,         ///< Paused (can resume)
    kStopped,        ///< Stopped (can set new source)
    kError,          ///< Error occurred
};

/** Source type for media content. */
enum class MediaSourceType : uint8_t {
    kFile = 0,       ///< Local file (SD card, flash)
    kHttp,           ///< HTTP/HTTPS network stream
};

/** Player events forwarded to application listeners. */
enum class MediaPlayerEvent : uint8_t {
    kSourceConnecting = 0,
    kSourceConnected,
    kPrepared,
    kPlayStarted,
    kSeekDone,
    kEndOfStream,
    kPlayError,
    kStateChanged,
};

/** Callback signature for player events. */
using MediaPlayerEventCallback = std::function<void(MediaPlayerEvent event,
                                                     MediaPlayerState state)>;

/* ------------------------------------------------------------------ */
/*  Configuration                                                     */
/* ------------------------------------------------------------------ */

/** FIFO tuning preset for memory/latency tradeoff. */
struct MediaFifoConfig {
    uint32_t adec_fifo_size     = 8192;    ///< Audio decoder buffer
    uint32_t vdec_fifo_size     = 16384;   ///< Video decoder buffer
    uint32_t arender_fifo_size  = 4096;    ///< Audio render buffer
    uint32_t vrender_fifo_size  = 8192;    ///< Video render buffer
    uint32_t extractor_pool_size = 10;     ///< Frame pool size
};

/** Initialization config for the media player service. */
struct MediaPlayerConfig {
    bool enable_audio         = true;    ///< Enable audio playback
    bool enable_video         = false;   ///< Enable video playback
    bool fast_seek            = true;    ///< Non-accurate seek for lower latency
    MediaFifoConfig fifo;                ///< FIFO tuning preset
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
     * @brief Initialize player with given config.
     * Creates renderers, opens player instance, starts worker task.
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
     * @brief Register event callback.
     * Callback is invoked from the worker task context (non-ISR safe).
     * Keep handlers lightweight; post heavy work to another task.
     */
    void SetEventCallback(MediaPlayerEventCallback cb);

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

    /* ---- State management ---- */
    void SetState(MediaPlayerState new_state);

    /* ---- Members ---- */
    std::atomic<bool>              initialized_{false};
    std::atomic<MediaPlayerState>  state_{MediaPlayerState::kIdle};
    media_player_handle_t          player_{nullptr};
    MediaPlayerConfig              config_{};
    MediaPlayerEventCallback       event_callback_;

    QueueHandle_t    cmd_queue_{nullptr};
    TaskHandle_t     worker_task_{nullptr};
    SemaphoreHandle_t callback_mutex_{nullptr};

    static constexpr int kCmdQueueLen     = 16;
    static constexpr int kWorkerStackSize = 6 * 1024;
    static constexpr int kWorkerPriority  = 5;
    static constexpr int kWorkerCore      = 0;
};

#endif // MEDIA_PLAYER_SERVICE_H
