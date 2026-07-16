#ifndef MP4_PLAYER_H
#define MP4_PLAYER_H

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_lcd_panel_ops.h>

extern "C" {
#include "audio_render.h"
#include "av_render.h"
#include "esp_extractor.h"
#include "esp_extractor_types.h"
#include "video_render.h"
}

class AudioCodec;
class SdCard;
class Display;

#define MP4_PLAYER_STATIC_TASK_CREATION 1
#define MP4_DEFAULT_DIRECTORY            "videos"
#define MP4_MAX_FILES                    256
#define MP4_EXTRACTOR_POOL_SIZE          (256 * 1024)
#define MP4_TASK_STACK_SIZE              (12 * 1024)
#define MP4_TASK_PRIORITY                5
#define MP4_TASK_CORE                    1
#define MP4_BUFFER_LINES                 40

enum class Mp4PlayerState {
    Idle = 0,
    Loading,
    Playing,
    Paused,
    Stopping,
    Error,
};

enum class Mp4RenderMode {
    DirectLcd,
    LvglCanvas,
};

struct Mp4FileInfo {
    std::string name;
    std::string path;
    uint32_t file_size = 0;
};

struct Mp4PlaybackStats {
    uint32_t video_frames = 0;
    uint32_t audio_frames = 0;
    uint32_t dropped_frames = 0;
    uint16_t video_width = 0;
    uint16_t video_height = 0;
    uint16_t video_fps = 0;
    uint32_t audio_rate = 0;
    uint8_t audio_channels = 0;
    uint8_t audio_bits = 0;
};

using Mp4StateCallback = std::function<void(Mp4PlayerState old_state, Mp4PlayerState new_state)>;
using Mp4EndCallback = std::function<bool(const std::string& file_path)>;
using Mp4FrameCallback = std::function<void(uint16_t* rgb565, uint16_t width, uint16_t height)>;
using Mp4AudioCallback = std::function<void(int16_t* pcm, size_t samples, int channels)>;

class Mp4Player {
public:
    static Mp4Player& GetInstance();

    bool Initialize(esp_lcd_panel_handle_t lcd_panel,
                    uint16_t lcd_width, uint16_t lcd_height,
                    AudioCodec* codec,
                    SdCard* sd_card,
                    Display* display = nullptr,
                    Mp4RenderMode mode = Mp4RenderMode::DirectLcd);
    void Deinitialize();

    bool Play(const std::string& file_path);
    bool PlayFile(const std::string& file_name);
    void Stop();
    void Pause();
    void Resume();
    bool Next();
    bool Prev();

    void SetDirectory(const std::string& dir);
    size_t ScanDirectory();

    const std::vector<Mp4FileInfo>& GetPlaylist() const { return playlist_; }
    int GetCurrentIndex() const { return current_index_; }

    Mp4PlayerState GetState() const { return state_.load(); }
    bool IsPlaying() const { return state_.load() == Mp4PlayerState::Playing; }
    bool IsPaused() const { return state_.load() == Mp4PlayerState::Paused; }
    bool IsInitialized() const { return initialized_.load(); }

    Mp4PlaybackStats GetStats() const;

    void SetStateCallback(Mp4StateCallback cb) { state_callback_ = std::move(cb); }
    void SetEndCallback(Mp4EndCallback cb) { end_callback_ = std::move(cb); }
    void SetFrameCallback(Mp4FrameCallback cb) { frame_callback_ = std::move(cb); }
    void SetAudioCallback(Mp4AudioCallback cb) { audio_callback_ = std::move(cb); }

    void SetVolume(float factor) { volume_factor_ = factor; }
    float GetVolume() const { return volume_factor_; }

    void SetRenderMode(Mp4RenderMode mode);
    Mp4RenderMode GetRenderMode() const { return render_mode_; }

private:
    struct RendererInitCfg {
        Mp4Player* owner = nullptr;
    };

    struct FileContext {
        FILE* fp = nullptr;
    };

    struct AudioRenderCtx;
    struct VideoRenderCtx;

    Mp4Player();
    ~Mp4Player();
    Mp4Player(const Mp4Player&) = delete;
    Mp4Player& operator=(const Mp4Player&) = delete;

    static void PlaybackTaskEntry(void* arg);
    void PlaybackTaskLoop();

    static int FileReadCb(void* buffer, uint32_t size, void* ctx);
    static int FileSeekCb(uint32_t position, void* ctx);
    static uint32_t FileSizeCb(void* ctx);

    static int AvRenderEventCb(av_render_event_t event, void* ctx);

    static audio_render_handle_t AudioRenderInit(void* cfg, int cfg_size);
    static int AudioRenderOpen(audio_render_handle_t render, av_render_audio_frame_info_t* info);
    static int AudioRenderWrite(audio_render_handle_t render, av_render_audio_frame_t* audio_data);
    static int AudioRenderGetLatency(audio_render_handle_t render, uint32_t* latency);
    static int AudioRenderGetFrameInfo(audio_render_handle_t render, av_render_audio_frame_info_t* info);
    static int AudioRenderSetSpeed(audio_render_handle_t render, float speed);
    static int AudioRenderClose(audio_render_handle_t render);
    static void AudioRenderDeinit(audio_render_handle_t render);

    static video_render_handle_t VideoRenderOpen(void* cfg, int size);
    static bool VideoRenderFormatSupported(video_render_handle_t render, av_render_video_frame_type_t type);
    static int VideoRenderSetFrameInfo(video_render_handle_t render, av_render_video_frame_info_t* info);
    static int VideoRenderGetFrameBuffer(video_render_handle_t render, av_render_frame_buffer_t* frame_buffer);
    static int VideoRenderWrite(video_render_handle_t render, av_render_video_frame_t* video_data);
    static int VideoRenderGetLatency(video_render_handle_t render, uint32_t* latency);
    static int VideoRenderGetFrameInfo(video_render_handle_t render, av_render_video_frame_info_t* info);
    static int VideoRenderClear(video_render_handle_t render);
    static int VideoRenderClose(video_render_handle_t render);

    bool PreparePlayback(const std::string& file_path);
    bool ConfigureRenderStreams();
    bool InitializeAvRender();
    void DestroyAvRender();
    void CleanupPlaybackResources();
    void CloseFile();
    bool ProcessAudioFrame(const esp_extractor_frame_info_t& frame);
    bool ProcessVideoFrame(const esp_extractor_frame_info_t& frame);
    bool PushEos();
    void ResetRenderEosFlags();
    bool WaitForRenderDrain(uint32_t timeout_ms);
    bool DrawFrameToLcd(const uint8_t* frame, uint16_t width, uint16_t height, av_render_video_frame_type_t frame_type);
    bool DrawFrameToCanvas(const uint8_t* frame, uint16_t width, uint16_t height, av_render_video_frame_type_t frame_type);
    void CreateVideoCanvas();
    void DestroyVideoCanvas();
    bool EnsureDrawBuffer(size_t bytes);
    void OutputPcmToCodec(const int16_t* pcm, size_t samples, int channels);
    void SetState(Mp4PlayerState new_state);
    std::string BuildFullPath(const std::string& filename) const;
    bool IsSupportedMp4Name(const std::string& name) const;
    av_render_audio_codec_t MapAudioCodec(esp_extractor_format_t format) const;
    av_render_video_codec_t MapVideoCodec(esp_extractor_format_t format) const;

    esp_lcd_panel_handle_t lcd_panel_ = nullptr;
    uint16_t lcd_width_ = 0;
    uint16_t lcd_height_ = 0;
    AudioCodec* audio_codec_ = nullptr;
    SdCard* sd_card_ = nullptr;

    std::atomic<Mp4PlayerState> state_{Mp4PlayerState::Idle};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> playback_task_running_{false};
    std::atomic<bool> audio_render_eos_{false};
    std::atomic<bool> video_render_eos_{false};

    TaskHandle_t playback_task_handle_ = nullptr;
#if MP4_PLAYER_STATIC_TASK_CREATION == 1
    StackType_t* playback_task_stack_ = nullptr;
    StaticTask_t* playback_task_buffer_ = nullptr;
#endif

    esp_extractor_handle_t extractor_ = nullptr;
    esp_extractor_config_t extractor_cfg_{};
    esp_extractor_stream_info_t audio_stream_info_{};
    esp_extractor_stream_info_t video_stream_info_{};
    av_render_audio_codec_t audio_codec_type_ = AV_RENDER_AUDIO_CODEC_NONE;
    av_render_video_codec_t video_codec_type_ = AV_RENDER_VIDEO_CODEC_NONE;
    // Display size from container/extractor (e.g. 320x180). H264 decoder may output
    // aligned coded size (e.g. 320x192), so render path crops to these expected values.
    uint16_t expected_video_width_ = 0;
    uint16_t expected_video_height_ = 0;
    FileContext file_ctx_{};
    std::string current_file_path_;

    RendererInitCfg renderer_init_cfg_{};
    audio_render_handle_t audio_render_ = nullptr;
    video_render_handle_t video_render_ = nullptr;
    av_render_handle_t av_render_ = nullptr;

    uint8_t* video_draw_buf_ = nullptr;
    size_t video_draw_buf_size_ = 0;

    Display* display_ = nullptr;
    void* video_canvas_ = nullptr;
    uint16_t* canvas_buf_ = nullptr;
    Mp4RenderMode render_mode_ = Mp4RenderMode::DirectLcd;

    mutable std::mutex stats_mutex_;
    Mp4PlaybackStats stats_{};

    std::string mp4_directory_ = MP4_DEFAULT_DIRECTORY;
    std::string mount_point_;
    std::vector<Mp4FileInfo> playlist_;
    int current_index_ = -1;

    float volume_factor_ = 1.0f;

    Mp4StateCallback state_callback_;
    Mp4EndCallback end_callback_;
    Mp4FrameCallback frame_callback_;
    Mp4AudioCallback audio_callback_;
};

#endif // MP4_PLAYER_H
