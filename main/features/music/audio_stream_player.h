#ifndef AUDIO_STREAM_PLAYER_H
#define AUDIO_STREAM_PLAYER_H

/**
 * @file audio_stream_player.h
 * @brief Base class for audio stream players (HTTP streams & SD card files).
 *
 * Provides common infrastructure:
 *   - Data source task (HTTP or file -- override SourceDataLoop())
 *   - Producer-consumer buffer in PSRAM
 *   - Decoder lifecycle (esp_audio_codec: MP3/AAC/FLAC, plus WAV passthrough)
 *   - PCM output, mono down-mix, volume amplification
 *   - Pause / resume support
 *   - FFT / display handled externally via callbacks
 *   - FreeRTOS tasks pinned to specific cores
 *
 * Subclasses override hooks to customize behaviour per use-case.
 */

#include <string>
#include <queue>
#include <deque>
#include <atomic>
#include <vector>
#include <functional>
#include <cstdint>
#include <mutex>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

extern "C" {
#include "esp_audio_simple_dec.h"
#include "esp_audio_simple_dec_default.h"
}

/* Forward declaration */
class AudioCodec;

/* ------------------------------------------------------------------ */
/*  Macros -- eliminates magic numbers across the whole module         */
/* ------------------------------------------------------------------ */

/** Enable static task creation (1 = enabled, 0 = disabled) */
#define AUDIO_STREAM_STATIC_TASK_CREATION  1

/** Maximum audio buffer size (bytes) -- limits PSRAM usage */
#define AUDIO_BUF_MAX_SIZE          (256 * 1024)

/** Minimum buffered data before playback starts -- HTTP streams */
#define AUDIO_BUF_MIN_SIZE          (32 * 1024)

/** Minimum buffered data before playback starts -- local file */
#define AUDIO_FILE_BUF_MIN_SIZE     (8 * 1024)

/** Per-chunk read size from HTTP stream (bytes) */
#define AUDIO_HTTP_CHUNK_SIZE       4096

/** Per-chunk read size from SD card file (bytes) */
#define AUDIO_FILE_READ_CHUNK_SIZE  4096

/** Decoder input buffer size (bytes) */
#define AUDIO_DEC_INPUT_BUF_SIZE    8192

/** Default PCM output buffer (enough for 1 MP3 frame: 1152*2ch*2B) */
#define AUDIO_PCM_OUT_BUF_SIZE      4608

/** Stack size for the source (download / file-read) task (bytes) */
#define AUDIO_SOURCE_TASK_STACK     (8 * 1024)

/** Stack size for the playback task (bytes) */
#define AUDIO_PLAY_TASK_STACK       (6 * 1024)

/** Source task priority */
#define AUDIO_SOURCE_TASK_PRIO      5

/** Playback task priority */
#define AUDIO_PLAY_TASK_PRIO        6

/** Core for source task (PRO_CPU = 0, APP_CPU = 1) */
#define AUDIO_SOURCE_TASK_CORE      0

/** Core for playback task */
#define AUDIO_PLAY_TASK_CORE        1

/** Progress log interval (bytes) */
#define AUDIO_LOG_INTERVAL          (128 * 1024)

/** Default volume amplification factor */
#define AUDIO_DEFAULT_VOLUME        1.0f

/** Maximum reconnect attempts for HTTP streaming */
#define AUDIO_MAX_RECONNECT         3

/** Reconnect delay (ms) */
#define AUDIO_RECONNECT_DELAY_MS    1500

/** WAV PCM block size in samples (matching MP3 frame) */
#define AUDIO_WAV_BLOCK_SAMPLES     (1152 * 2)

/* ------------------------------------------------------------------ */
/*  Audio player state machine (mirrors VideoPlayerState)             */
/* ------------------------------------------------------------------ */

enum class AudioPlayerState {
    Idle = 0,       ///< No stream active, ready for commands
    Loading,        ///< Opening stream / buffering initial data
    Playing,        ///< Active playback (decoding + audio output)
    Paused,         ///< Playback paused, can resume
    Stopping,       ///< Tearing down resources
    Error           ///< Unrecoverable error (call Stop to reset)
};

/* ------------------------------------------------------------------ */
/*  Callback typedefs for audio player events                        */
/* ------------------------------------------------------------------ */

/** Called when audio player state changes */
using AudioStateCallback = std::function<void(AudioPlayerState old_state, AudioPlayerState new_state)>;

/** Called when a stream finishes (naturally or stopped) */
using AudioEndCallback = std::function<bool(const std::string& source)>;

/** Called with FFT-ready PCM data for spectrum display */
using AudioFftCallback = std::function<void(int16_t* pcm_data, size_t pcm_bytes)>;

/** Called with decoded PCM frame data for custom processing */
using AudioPcmCallback = std::function<void(int16_t* pcm_data, int total_samples,
                                            int channels, int sample_rate)>;

/* ------------------------------------------------------------------ */
/*  Decoder type enum                                                 */
/* ------------------------------------------------------------------ */

enum class AudioDecoderType {
    MP3 = 0,
    AAC,
    FLAC,
    WAV,    ///< Raw PCM passthrough (16-bit, no decoder needed)
    AUTO    ///< Auto-detect from stream header
};

/* ------------------------------------------------------------------ */
/*  Audio data chunk (heap-allocated, lives in PSRAM)                 */
/* ------------------------------------------------------------------ */

struct StreamAudioChunk {
    uint8_t* data;
    size_t   size;
    uint32_t trace_sequence;
    uint32_t stream_generation;

    StreamAudioChunk()
        : data(nullptr), size(0), trace_sequence(0), stream_generation(0) {}
    StreamAudioChunk(uint8_t* d, size_t s, uint32_t sequence = 0,
                     uint32_t generation = 0)
        : data(d), size(s), trace_sequence(sequence),
          stream_generation(generation) {}
};

/** Metadata de noi cac moc audio ma khong ghi noi dung hoi thoai/payload TTS. */
struct AudioTraceContext {
    uint32_t trace_sequence = 0;
    std::string turn_id = "-";
    std::string segment_id = "-";
    std::string audio_source = "unknown";
};

/** Chuan hoa truong correlation truoc khi dua vao serial log. */
std::string SanitizeAudioTraceField(const std::string& value);

/** Structured event dung dong ho monotonic cua ESP-IDF. */
void LogAudioTraceEvent(const char* event, const AudioTraceContext& context,
                        int64_t timestamp_us = -1);
void LogAudioTraceQueue(const char* event, const char* action,
                        const AudioTraceContext& context, size_t queue_depth);

/* ------------------------------------------------------------------ */
/*  WAV header info (for raw PCM passthrough)                         */
/* ------------------------------------------------------------------ */

struct WavHeaderInfo {
    int    sample_rate     = 0;
    int    channels        = 0;
    int    bits_per_sample = 16;
    size_t data_offset     = 0;
    size_t data_size       = 0;
};

/* ------------------------------------------------------------------ */
/*  AudioStreamPlayer -- base class                                   */
/* ------------------------------------------------------------------ */

class AudioStreamPlayer {
public:
    /** Display mode (shared by music & radio) */
    enum DisplayMode {
        DISPLAY_MODE_SPECTRUM = 0,
        DISPLAY_MODE_INFO     = 1
    };

    AudioStreamPlayer();
    virtual ~AudioStreamPlayer();

    /* ---- Common public API ---- */
    bool  StartStream(const std::string& source,
                      AudioDecoderType type = AudioDecoderType::MP3);
    bool  StopStream();

    virtual bool IsPlaying() const     { return is_playing_.load(); }
    bool  IsDownloading() const        { return is_source_active_.load(); }
    bool  IsSourceActive() const       { return is_source_active_.load(); }
    size_t GetBufferSize() const       { return buffer_size_; }

    /* ---- State machine ---- */
    AudioPlayerState GetPlayerState() const { return player_state_.load(); }

    /* ---- Audio codec (direct PCM output) ---- */
    void SetAudioCodec(AudioCodec* codec) { audio_codec_ = codec; }
    AudioCodec* GetAudioCodec() const { return audio_codec_; }

    /* ---- Callbacks ---- */
    void SetStateCallback(AudioStateCallback cb)  { state_callback_ = std::move(cb); }
    void SetEndCallback(AudioEndCallback cb)      { end_callback_ = std::move(cb); }
    void SetFftCallback(AudioFftCallback cb)      { fft_callback_ = std::move(cb); }
    void SetPcmCallback(AudioPcmCallback cb)      { pcm_callback_ = std::move(cb); }

    /* ---- Pause / Resume ---- */
    void  PauseStream();
    void  ResumeStream();
    bool  IsPaused() const { return is_paused_.load(); }

    /** Get current playback time in milliseconds (public). */
    int64_t GetPlayTimeMs() const { return current_play_time_ms_; }

    /** Get total content length in bytes (from HTTP Content-Length). 0 if unknown. */
    size_t GetContentLength() const { return content_length_; }

    void SetDisplayMode(DisplayMode mode);
    DisplayMode GetDisplayMode() const { return display_mode_.load(); }

    /** Set volume amplification (1.0 = 100%) */
    void  SetVolume(float factor) { volume_factor_ = factor; }
    float GetVolume() const       { return volume_factor_; }

protected:
    /* ---- Hooks for subclasses (override as needed) ---- */

    /**
     * Data source loop. Override to provide custom data sources (e.g. file).
     * Default implementation performs HTTP streaming with reconnect.
     * Use PushToBuffer() to feed data into the playback pipeline.
     * Check IsSourceActive() and IsPlaying() for stop requests.
     * @param source  URL or file path passed to StartStream().
     */
    virtual void SourceDataLoop(const std::string& source);

    /** Called before HTTP download starts. Override to add custom headers. */
    virtual void OnPrepareHttp(void* http_ptr) {}

    /** Called once after the first frame is decoded. */
    virtual void OnStreamInfoReady(int sample_rate, int bits_per_sample,
                                   int channels, int bitrate, int frame_size) {}

    /** Called on each decoded PCM frame. */
    virtual void OnPcmFrame(int64_t play_time_ms, int sample_rate,
                            int channels) {}

    /** Called when playback finishes (naturally or stopped). */
    virtual bool OnPlaybackFinishedAndContinue() { return false; }

    /** Called once when FFT display canvas is ready. */
    virtual void OnDisplayReady() {}

    /** Called when pause state changes. */
    virtual void OnPauseStateChanged(bool paused) {}

    /* ---- Protected utilities for subclasses ---- */

    /** Push data into the playback buffer (copies to PSRAM). Thread-safe.
     *  Returns false if stopped or allocation failed. */
    bool PushToBuffer(const void* data, size_t size,
                      uint32_t trace_sequence = 0,
                      uint32_t stream_generation = 0);

    /**
     * Cat ngay pipeline hien tai ma van giu hai worker song. Byte/PCM den muon
     * cua generation cu se bi loai bo thay vi chen vao cau moi.
     */
    void InterruptStream();
    uint32_t GetStreamGeneration() const {
        return stream_generation_.load();
    }

    /** Dang ky correlation truoc khi byte cua segment vao buffer. */
    void RegisterAudioTraceSegment(const AudioTraceContext& context);

    /** Source bao da het byte; playback tu quyet dinh luc decoder da yen. */
    void RequestAudioTraceSegmentFinish(uint32_t trace_sequence);

    /** Get decoder type for the current stream. */
    AudioDecoderType GetDecoderType() const { return decoder_type_; }

    /** Access to the display mode atomic. */
    std::atomic<DisplayMode>& DisplayModeRef() { return display_mode_; }

    /** WAV header info -- set before StartStream with WAV type. */
    WavHeaderInfo wav_info_;

    /**
     * Bytes to buffer before playback starts. Default AUDIO_BUF_MIN_SIZE (32KB)
     * suits radio: a big prebuffer hides network jitter over a long stream.
     *
     * TTS is the opposite case -- one spoken sentence is only ~17KB of 48kbps
     * MP3, so with the radio default playback would wait for a SECOND sentence
     * before making any sound. DeviceTtsClient lowers this to a few KB.
     * Call before StartStream().
     */
    void SetMinBufferSize(size_t bytes) { min_buffer_size_ = bytes; }
    size_t min_buffer_size_ = AUDIO_BUF_MIN_SIZE;

private:
    /* ---- FreeRTOS task wrappers ---- */
    static void SourceTaskEntry(void* param);
    static void PlayTaskEntry(void* param);

    void PlayLoop();
    void PlayLoopCompressed();
    void PlayLoopWav();

    /* ---- PCM output helper (shared between compressed & WAV paths) ---- */
    void OutputPcmFrame(int16_t* pcm_in, int total_samples, int channels,
                        int sample_rate, int frame_duration_ms,
                        uint32_t expected_generation);

    /** Output PCM directly through AudioCodec (bypasses Application pipeline) */
    void OutputPcmDirect(int16_t* pcm_in, int total_samples, int channels,
                         int sample_rate);

    /** Transition player state with notification */
    void SetPlayerState(AudioPlayerState new_state);

    /* ---- Common playback helpers ---- */
    bool HandlePause();

    /* ---- Buffer helpers ---- */
    void ClearAudioBuffer();
    void DiscardQueuedAudio();

    /** Noi byte input da consume voi segment, khong log tren tung frame. */
    void ConsumeTraceInputBytes(size_t bytes);
    void ObserveDecodedPcm(uint32_t decoded_trace_sequence);
    void MaybeFinishRequestedAudioTrace();
    void FinishAudioTraceSegment(uint32_t trace_sequence);
    void EmitLastPcm(const AudioTraceContext& context, int64_t timestamp_us);
    void FinishActiveAudioTrace();

    /* ---- Decoder helpers ---- */
    bool  InitDecoder(AudioDecoderType type);
    void  CleanupDecoder();
    bool  ResetCompressedPlaybackForGeneration();
    AudioDecoderType DetectStreamType(const uint8_t* data, size_t len);

    /* ---- Sample-rate reset ---- */
    void ResetSampleRate();

    /* ---- State ---- */
    std::atomic<bool> is_playing_;
    std::atomic<bool> is_source_active_;
    std::atomic<bool> is_paused_;
    std::atomic<uint32_t> stream_generation_{1};
    std::mutex output_generation_mutex_;
    std::atomic<DisplayMode> display_mode_;
    std::atomic<AudioPlayerState> player_state_{AudioPlayerState::Idle};
    float volume_factor_;

    /* ---- Audio codec (direct output) ---- */
    AudioCodec* audio_codec_ = nullptr;

    /* ---- Callbacks ---- */
    AudioStateCallback state_callback_;
    AudioEndCallback   end_callback_;
    AudioFftCallback   fft_callback_;
    AudioPcmCallback   pcm_callback_;

    /* ---- FreeRTOS handles ---- */
    TaskHandle_t source_task_handle_;
    std::atomic<bool> source_task_exited_{true};
#if AUDIO_STREAM_STATIC_TASK_CREATION == 1
    StaticTask_t* source_task_buffer_;
    StackType_t* source_task_stack_;
#endif
    TaskHandle_t play_task_handle_;
    std::atomic<bool> play_task_exited_{true};
#if AUDIO_STREAM_STATIC_TASK_CREATION == 1
    StaticTask_t* play_task_buffer_;
    StackType_t* play_task_stack_;
#endif
    SemaphoreHandle_t pause_sem_;     ///< binary semaphore for pause/resume

    /* ---- Audio buffer (producer-consumer) ---- */
    std::queue<StreamAudioChunk> audio_buffer_;
    SemaphoreHandle_t            buffer_mutex_;
    SemaphoreHandle_t            buffer_data_sem_;
    SemaphoreHandle_t            buffer_space_sem_;
    size_t                       buffer_size_;

    struct TraceInputSpan {
        size_t bytes = 0;
        uint32_t trace_sequence = 0;
    };
    std::deque<TraceInputSpan> input_trace_spans_;
    mutable std::mutex trace_mutex_;
    std::deque<AudioTraceContext> trace_contexts_;
    uint32_t active_pcm_trace_sequence_ = 0;
    uint32_t finish_requested_trace_sequence_ = 0;
    int64_t last_pcm_timestamp_us_ = 0;

    /* ---- Decoder (esp_audio_codec) ---- */
    esp_audio_simple_dec_handle_t decoder_;
    esp_audio_simple_dec_info_t   dec_info_;
    bool                          decoder_initialized_;
    bool                          dec_info_ready_;
    AudioDecoderType              decoder_type_;
    uint8_t*                      pcm_out_buffer_;
    size_t                        pcm_out_buffer_size_;
    std::vector<uint8_t>          dec_out_vec_;

    /* ---- Decoder input ---- */
    uint8_t* input_buffer_;
    int      input_bytes_left_;

    /* ---- Playback timing ---- */
    int64_t current_play_time_ms_;
    int     total_frames_decoded_;

    /* ---- Source path / URL ---- */
    std::string stream_url_;

    /* ---- Content info (from HTTP) ---- */
    size_t content_length_ = 0;   ///< Total bytes from Content-Length header
};

#endif // AUDIO_STREAM_PLAYER_H
