#ifndef MEDIA_RENDER_CALLBACK_H
#define MEDIA_RENDER_CALLBACK_H

/**
 * @file media_render_callback.h
 * @brief Callback-mode render handler for MediaPlayerService.
 *
 * When MediaPlayerService uses kCallback render mode, decoded audio/video
 * frames are forwarded here for processing:
 *   - Audio PCM → AudioCodec I2S output (like VideoPlayer::OutputAudioPcm)
 *   - Video YUV420 → RGB565 conversion → LVGL canvas display
 *
 * Video rendering runs on a separate task to avoid blocking the media
 * player's decode/demux pipeline (same pattern as VideoPlayer's render task).
 */

#include <atomic>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_lcd_panel_ops.h>

/* Forward declarations */
class AudioCodec;
class Display;

class MediaRenderCallback {
public:
    MediaRenderCallback();
    ~MediaRenderCallback();

    /**
     * @brief Initialize the callback render handler.
     *
     * @param codec      AudioCodec for I2S audio output (nullable for video-only)
     * @param panel      LCD panel handle (nullable for audio-only)
     * @param lcd_width  Display width in pixels
     * @param lcd_height Display height in pixels
     * @param display    Display for LVGL lock (nullable for audio-only)
     * @return true on success
     */
    bool Init(AudioCodec* codec, esp_lcd_panel_handle_t panel,
              uint16_t lcd_width, uint16_t lcd_height, Display* display);

    /** Release all resources. */
    void Deinit();

    /* ---- Audio callbacks (called from media_player audio render thread) ---- */

    /**
     * @brief Called when audio format is determined.
     * Sets codec sample rate for correct playback.
     */
    void OnAudioClock(uint32_t sample_rate, uint8_t bits_per_sample,
                      uint8_t channels);

    /**
     * @brief Called with decoded PCM audio data.
     * Outputs audio through AudioCodec (blocking until I2S DMA accepts).
     */
    void OnAudioData(const uint8_t* data, int size, uint32_t pts_ms);

    /* ---- Video callbacks (called from media_player video render thread) ---- */

    /**
     * @brief Called when video format is determined.
     * Creates LVGL canvas and initializes color converter.
     */
    void OnVideoInfo(uint16_t width, uint16_t height, uint8_t fps,
                     uint8_t frame_type);

    /**
     * @brief Called with decoded video frame data (typically YUV420).
     * Copies frame to pending buffer and signals render task.
     */
    void OnVideoFrame(const uint8_t* data, int size, uint16_t width,
                      uint16_t height, uint32_t pts_ms);

    /** @return true if Init() succeeded. */
    bool IsInitialized() const { return initialized_.load(); }

private:
    /* ---- Audio output ---- */
    void OutputAudioPcm(const uint8_t* pcm_data, size_t data_bytes,
                        uint8_t channels, uint8_t bits_per_sample);

    /* ---- Video render task ---- */
    static void RenderTaskEntry(void* arg);
    void RenderTaskLoop();

    /* ---- LVGL canvas lifecycle ---- */
    void CreateVideoCanvas();
    void DestroyVideoCanvas();
    void DrawFrameToCanvas(uint16_t vw, uint16_t vh);

    /* ---- Color conversion ---- */
    bool InitColorConverter(uint16_t width, uint16_t height, uint8_t frame_type);
    void DeinitColorConverter();

    /* ---- Members ---- */
    std::atomic<bool> initialized_{false};

    /* Hardware handles */
    AudioCodec*            audio_codec_{nullptr};
    esp_lcd_panel_handle_t lcd_panel_{nullptr};
    Display*               display_{nullptr};
    uint16_t               lcd_width_{0};
    uint16_t               lcd_height_{0};

    /* Audio state */
    uint32_t audio_sample_rate_{0};
    uint8_t  audio_bits_{16};
    uint8_t  audio_channels_{1};

    /* Video state */
    uint16_t video_width_{0};
    uint16_t video_height_{0};
    uint8_t  video_frame_type_{0};

    /* LVGL canvas */
    void*     video_canvas_{nullptr};   ///< lv_obj_t* canvas widget
    uint16_t* canvas_buf_{nullptr};     ///< RGB565 canvas buffer in PSRAM

    /* Color converter (YUV420 → RGB565) */
    void* color_converter_{nullptr};    ///< color_convert_table_t handle
    uint8_t* rgb565_buf_{nullptr};      ///< Converted RGB565 output buffer

    /* Render task and synchronization */
    TaskHandle_t      render_task_{nullptr};
    SemaphoreHandle_t render_sem_{nullptr};
    SemaphoreHandle_t frame_mutex_{nullptr};
    std::atomic<bool> render_exit_{false};
    std::atomic<bool> render_running_{false};

    /* Pending frame buffer (double-buffer pattern) */
    uint8_t* pending_frame_buf_{nullptr};  ///< Raw frame data from callback
    uint8_t* decode_frame_buf_{nullptr};   ///< Copy for render task processing
    size_t   pending_frame_size_{0};
    uint16_t pending_frame_w_{0};
    uint16_t pending_frame_h_{0};
    size_t   frame_buf_capacity_{0};       ///< Allocated capacity

    /* Task configuration */
    static constexpr int kRenderTaskStack = 8 * 1024;
    static constexpr int kRenderTaskPrio  = 15;
    static constexpr int kRenderTaskCore  = 1;
};

#endif // MEDIA_RENDER_CALLBACK_H
