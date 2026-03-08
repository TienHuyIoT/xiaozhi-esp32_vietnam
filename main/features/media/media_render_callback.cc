/**
 * @file media_render_callback.cc
 * @brief Implementation of callback-mode render handler.
 *
 * Audio: Decoded PCM → channel mixing/pass-through → AudioCodec I2S output.
 * Video: Decoded YUV420 → color_convert (lookup table) → RGB565 → LVGL canvas.
 *
 * The video render task runs independently from the media player's decode
 * pipeline. The callback copies raw frame data to a pending buffer and signals
 * the render task via semaphore; the render task then performs the potentially
 * expensive color conversion and LVGL canvas update without blocking the
 * demux/decode threads.
 */

#include "media_render_callback.h"
#include "audio_codec.h"
#include "display.h"

#include <algorithm>
#include <cstring>
#include <vector>

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <lvgl.h>

extern "C" {
#include "av_render_types.h"
}

/* color_convert API from tempotian__av_render (private header, declare here) */
extern "C" {
    typedef void* color_convert_table_t;
    typedef struct {
        av_render_video_frame_type_t from;
        av_render_video_frame_type_t to;
        int width;
        int height;
    } color_convert_cfg_t;

    color_convert_table_t init_convert_table(color_convert_cfg_t* cfg);
    int convert_color(color_convert_table_t table, uint8_t* src, int src_size,
                      uint8_t* dst, int dst_size);
    void deinit_convert_table(color_convert_table_t t);
    int convert_table_get_image_size(av_render_video_frame_type_t fmt,
                                     int width, int height);
}

static const char* TAG = "💡 MediaRenderCb";

/* ================================================================== */
/*  Constructor / Destructor                                          */
/* ================================================================== */

MediaRenderCallback::MediaRenderCallback() = default;

MediaRenderCallback::~MediaRenderCallback() {
    Deinit();
}

/* ================================================================== */
/*  Lifecycle                                                         */
/* ================================================================== */

bool MediaRenderCallback::Init(AudioCodec* codec, esp_lcd_panel_handle_t panel,
                                uint16_t lcd_width, uint16_t lcd_height,
                                Display* display) {
    if (initialized_.load()) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    audio_codec_ = codec;
    lcd_panel_   = panel;
    lcd_width_   = lcd_width;
    lcd_height_  = lcd_height;
    display_     = display;

    /* Create synchronization primitives */
    render_sem_ = xSemaphoreCreateBinary();
    frame_mutex_ = xSemaphoreCreateMutex();
    if (!render_sem_ || !frame_mutex_) {
        ESP_LOGE(TAG, "Failed to create sync primitives");
        Deinit();
        return false;
    }

    /* Start render task for video processing */
    if (display_) {
        render_exit_.store(false);
        BaseType_t ret = xTaskCreatePinnedToCore(
            RenderTaskEntry, "media_render", kRenderTaskStack,
            this, kRenderTaskPrio, &render_task_, kRenderTaskCore);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create render task");
            Deinit();
            return false;
        }
    }

    initialized_.store(true);
    ESP_LOGI(TAG, "Initialized (audio=%d video=%d lcd=%dx%d)",
             codec != nullptr, display != nullptr, lcd_width, lcd_height);
    return true;
}

void MediaRenderCallback::Deinit() {
    /* Stop render task */
    if (render_task_) {
        render_exit_.store(true);
        if (render_sem_) {
            xSemaphoreGive(render_sem_);
        }
        int timeout = 100;
        while (render_running_.load() && --timeout > 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        render_task_ = nullptr;
        render_exit_.store(false);
    }

    DestroyVideoCanvas();
    DeinitColorConverter();

    /* Free frame buffers */
    if (pending_frame_buf_) {
        heap_caps_free(pending_frame_buf_);
        pending_frame_buf_ = nullptr;
    }
    if (decode_frame_buf_) {
        heap_caps_free(decode_frame_buf_);
        decode_frame_buf_ = nullptr;
    }
    if (rgb565_buf_) {
        heap_caps_free(rgb565_buf_);
        rgb565_buf_ = nullptr;
    }
    frame_buf_capacity_ = 0;

    if (render_sem_) {
        vSemaphoreDelete(render_sem_);
        render_sem_ = nullptr;
    }
    if (frame_mutex_) {
        vSemaphoreDelete(frame_mutex_);
        frame_mutex_ = nullptr;
    }

    audio_codec_ = nullptr;
    lcd_panel_   = nullptr;
    display_     = nullptr;

    initialized_.store(false);
    ESP_LOGI(TAG, "Deinitialized");
}

/* ================================================================== */
/*  Audio callbacks                                                   */
/* ================================================================== */

void MediaRenderCallback::OnAudioClock(uint32_t sample_rate,
                                        uint8_t bits_per_sample,
                                        uint8_t channels) {
    ESP_LOGI(TAG, "Audio clock: rate=%lu bits=%u ch=%u",
             sample_rate, bits_per_sample, channels);

    audio_sample_rate_ = sample_rate;
    audio_bits_ = bits_per_sample;
    audio_channels_ = channels;

    if (audio_codec_ && sample_rate > 0) {
        audio_codec_->SetOutputSampleRate(static_cast<int>(sample_rate));
    }
}

void MediaRenderCallback::OnAudioData(const uint8_t* data, int size,
                                       uint32_t pts_ms) {
    if (!audio_codec_ || !data || size <= 0) return;
    OutputAudioPcm(data, static_cast<size_t>(size),
                   audio_channels_, audio_bits_);
}

/* ================================================================== */
/*  Video callbacks                                                   */
/* ================================================================== */

void MediaRenderCallback::OnVideoInfo(uint16_t width, uint16_t height,
                                       uint8_t fps, uint8_t frame_type) {
    ESP_LOGI(TAG, "Video info: %dx%d @%dfps type=%d", width, height, fps,
             frame_type);

    video_width_ = width;
    video_height_ = height;
    video_frame_type_ = frame_type;

    /* Determine buffer sizes based on frame type */
    size_t src_size;
    if (frame_type == AV_RENDER_VIDEO_RAW_TYPE_YUV420) {
        src_size = static_cast<size_t>(width) * height * 3 / 2;
    } else {
        /* RGB565 or other: 2 bytes per pixel */
        src_size = static_cast<size_t>(width) * height * 2;
    }

    /* Allocate/reallocate frame buffers if needed */
    if (src_size > frame_buf_capacity_) {
        if (pending_frame_buf_) heap_caps_free(pending_frame_buf_);
        if (decode_frame_buf_) heap_caps_free(decode_frame_buf_);

        pending_frame_buf_ = static_cast<uint8_t*>(
            heap_caps_malloc(src_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        decode_frame_buf_ = static_cast<uint8_t*>(
            heap_caps_malloc(src_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

        if (!pending_frame_buf_ || !decode_frame_buf_) {
            ESP_LOGE(TAG, "Failed to allocate frame buffers (%zu bytes each)",
                     src_size);
            return;
        }
        frame_buf_capacity_ = src_size;
        ESP_LOGI(TAG, "Frame buffers allocated: 2 x %zu bytes in PSRAM",
                 src_size);
    }

    /* Allocate RGB565 output buffer for YUV420 conversion */
    size_t rgb_size = static_cast<size_t>(width) * height * 2;
    if (rgb565_buf_) heap_caps_free(rgb565_buf_);
    rgb565_buf_ = static_cast<uint8_t*>(
        heap_caps_malloc(rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!rgb565_buf_) {
        ESP_LOGE(TAG, "Failed to allocate RGB565 buffer (%zu bytes)", rgb_size);
        return;
    }

    /* Initialize color converter */
    InitColorConverter(width, height, frame_type);

    /* Create LVGL canvas for display */
    if (display_) {
        CreateVideoCanvas();
    }
}

void MediaRenderCallback::OnVideoFrame(const uint8_t* data, int size,
                                        uint16_t width, uint16_t height,
                                        uint32_t pts_ms) {
    if (!initialized_.load() || !data || size <= 0) return;
    if (!display_ || !render_sem_) return;

    size_t copy_size = static_cast<size_t>(size);
    if (copy_size > frame_buf_capacity_) {
        ESP_LOGW(TAG, "Frame too large: %zu > %zu", copy_size,
                 frame_buf_capacity_);
        return;
    }

    /* Copy frame data to pending buffer (mutex-protected, fast memcpy) */
    xSemaphoreTake(frame_mutex_, portMAX_DELAY);
    memcpy(pending_frame_buf_, data, copy_size);
    pending_frame_size_ = copy_size;
    pending_frame_w_ = width;
    pending_frame_h_ = height;
    xSemaphoreGive(frame_mutex_);

    /* Signal render task */
    xSemaphoreGive(render_sem_);
}

/* ================================================================== */
/*  Audio output (PCM → AudioCodec I2S)                               */
/* ================================================================== */

void MediaRenderCallback::OutputAudioPcm(const uint8_t* pcm_data,
                                          size_t data_bytes,
                                          uint8_t channels,
                                          uint8_t bits_per_sample) {
    if (!audio_codec_ || !pcm_data || data_bytes == 0) return;

    if (bits_per_sample != 16) {
        ESP_LOGW(TAG, "Unsupported audio bits: %u", bits_per_sample);
        return;
    }

    size_t num_samples = data_bytes / sizeof(int16_t);
    const int16_t* src = reinterpret_cast<const int16_t*>(pcm_data);

    std::vector<int16_t> audio_out;
    int codec_channels = audio_codec_->output_channels();

    if (channels == 1 && codec_channels >= 2) {
        /* Mono → stereo: duplicate each sample */
        audio_out.resize(num_samples * 2);
        for (size_t i = 0; i < num_samples; i++) {
            audio_out[i * 2]     = src[i];
            audio_out[i * 2 + 1] = src[i];
        }
    } else if (channels == 2 && codec_channels == 1) {
        /* Stereo → mono: average L+R */
        size_t frames = num_samples / 2;
        audio_out.resize(frames);
        for (size_t i = 0; i < frames; i++) {
            int32_t mixed = (static_cast<int32_t>(src[i * 2]) +
                             static_cast<int32_t>(src[i * 2 + 1])) / 2;
            if (mixed > 32767) mixed = 32767;
            if (mixed < -32768) mixed = -32768;
            audio_out[i] = static_cast<int16_t>(mixed);
        }
    } else {
        /* Same channel count: pass-through */
        audio_out.assign(src, src + num_samples);
    }

    audio_codec_->OutputData(audio_out);
}

/* ================================================================== */
/*  Color conversion                                                  */
/* ================================================================== */

bool MediaRenderCallback::InitColorConverter(uint16_t width, uint16_t height,
                                              uint8_t frame_type) {
    DeinitColorConverter();

    if (frame_type != AV_RENDER_VIDEO_RAW_TYPE_YUV420) {
        ESP_LOGI(TAG, "Frame type %d does not need color conversion", frame_type);
        return true;
    }

    color_convert_cfg_t cfg = {};
    cfg.from = AV_RENDER_VIDEO_RAW_TYPE_YUV420;
    cfg.to = AV_RENDER_VIDEO_RAW_TYPE_RGB565;  /* LE for LVGL */
    cfg.width = width;
    cfg.height = height;

    color_converter_ = init_convert_table(&cfg);
    if (!color_converter_) {
        ESP_LOGE(TAG, "Failed to init color converter YUV420→RGB565 %dx%d",
                 width, height);
        return false;
    }

    ESP_LOGI(TAG, "Color converter initialized: YUV420→RGB565 %dx%d",
             width, height);
    return true;
}

void MediaRenderCallback::DeinitColorConverter() {
    if (color_converter_) {
        deinit_convert_table(static_cast<color_convert_table_t>(color_converter_));
        color_converter_ = nullptr;
    }
}

/* ================================================================== */
/*  LVGL canvas lifecycle                                             */
/* ================================================================== */

void MediaRenderCallback::CreateVideoCanvas() {
    if (!display_) return;

    DestroyVideoCanvas();

    size_t buf_size = static_cast<size_t>(lcd_width_) * lcd_height_ *
                      sizeof(uint16_t);
    canvas_buf_ = static_cast<uint16_t*>(
        heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!canvas_buf_) {
        ESP_LOGE(TAG, "Failed to allocate canvas buffer (%zu bytes)", buf_size);
        return;
    }
    memset(canvas_buf_, 0, buf_size);

    {
        DisplayLockGuard lock(display_);
        lv_obj_t* canvas = lv_canvas_create(lv_scr_act());
        lv_canvas_set_buffer(canvas, canvas_buf_, lcd_width_, lcd_height_,
                             LV_COLOR_FORMAT_RGB565);
        lv_obj_set_pos(canvas, 0, 0);
        lv_obj_set_size(canvas, lcd_width_, lcd_height_);
        lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
        lv_obj_move_foreground(canvas);
        video_canvas_ = canvas;
    }

    ESP_LOGI(TAG, "Video canvas created: %dx%d", lcd_width_, lcd_height_);
}

void MediaRenderCallback::DestroyVideoCanvas() {
    if (video_canvas_ && display_) {
        DisplayLockGuard lock(display_);
        lv_obj_del(static_cast<lv_obj_t*>(video_canvas_));
        video_canvas_ = nullptr;
    }
    if (canvas_buf_) {
        heap_caps_free(canvas_buf_);
        canvas_buf_ = nullptr;
    }
}

void MediaRenderCallback::DrawFrameToCanvas(uint16_t vw, uint16_t vh) {
    if (!display_ || !video_canvas_ || !canvas_buf_ || !rgb565_buf_) return;

    const auto* src = reinterpret_cast<const uint16_t*>(rgb565_buf_);
    uint16_t draw_w = std::min(vw, lcd_width_);
    uint16_t draw_h = std::min(vh, lcd_height_);
    int x_offset = (lcd_width_ - draw_w) / 2;
    int y_offset = (lcd_height_ - draw_h) / 2;

    if (x_offset == 0 && draw_w == lcd_width_ &&
        y_offset == 0 && draw_h == lcd_height_) {
        size_t copy_size = draw_w * draw_h * sizeof(uint16_t);
        memcpy(canvas_buf_, src, copy_size);
    } else {
        /* Clear canvas to black, then copy centered video */
        memset(canvas_buf_, 0,
               static_cast<size_t>(lcd_width_) * lcd_height_ * sizeof(uint16_t));
        for (uint16_t row = 0; row < draw_h; row++) {
            uint16_t* dst_row = canvas_buf_ + (y_offset + row) * lcd_width_ +
                                x_offset;
            const uint16_t* src_row = src + row * vw;
            memcpy(dst_row, src_row, draw_w * sizeof(uint16_t));
        }
    }

    {
        DisplayLockGuard lock(display_);
        lv_obj_invalidate(static_cast<lv_obj_t*>(video_canvas_));
    }
}

/* ================================================================== */
/*  Render task                                                       */
/* ================================================================== */

void MediaRenderCallback::RenderTaskEntry(void* arg) {
    auto* self = static_cast<MediaRenderCallback*>(arg);
    self->RenderTaskLoop();
}

void MediaRenderCallback::RenderTaskLoop() {
    ESP_LOGI(TAG, "Render task started on core %d", xPortGetCoreID());
    render_running_.store(true);

    while (true) {
        if (xSemaphoreTake(render_sem_, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (render_exit_.load()) break;
            continue;
        }

        if (render_exit_.load()) break;

        /* Atomically copy pending frame data to decode buffer */
        size_t frame_size;
        uint16_t vw, vh;

        xSemaphoreTake(frame_mutex_, portMAX_DELAY);
        frame_size = pending_frame_size_;
        vw = pending_frame_w_;
        vh = pending_frame_h_;
        if (frame_size > 0 && frame_size <= frame_buf_capacity_) {
            memcpy(decode_frame_buf_, pending_frame_buf_, frame_size);
        }
        xSemaphoreGive(frame_mutex_);

        if (frame_size == 0) continue;

        /* Convert or copy to RGB565 buffer */
        if (video_frame_type_ == AV_RENDER_VIDEO_RAW_TYPE_YUV420 &&
            color_converter_) {
            /* YUV420 → RGB565 conversion via lookup table */
            int src_size = static_cast<int>(frame_size);
            int dst_size = static_cast<int>(vw) * vh * 2;
            int ret = convert_color(
                static_cast<color_convert_table_t>(color_converter_),
                decode_frame_buf_, src_size, rgb565_buf_, dst_size);
            if (ret != 0) {
                ESP_LOGW(TAG, "Color conversion failed");
                continue;
            }
        } else if (video_frame_type_ == AV_RENDER_VIDEO_RAW_TYPE_RGB565 ||
                   video_frame_type_ == AV_RENDER_VIDEO_RAW_TYPE_RGB565_BE) {
            /* Already RGB565, copy directly */
            size_t rgb_size = static_cast<size_t>(vw) * vh * 2;
            if (frame_size >= rgb_size) {
                memcpy(rgb565_buf_, decode_frame_buf_, rgb_size);
            }
        } else {
            ESP_LOGW(TAG, "Unsupported frame type for rendering: %d",
                     video_frame_type_);
            continue;
        }

        /* Draw to LVGL canvas */
        DrawFrameToCanvas(vw, vh);
    }

    render_running_.store(false);
    ESP_LOGI(TAG, "Render task exiting");
    vTaskDelete(nullptr);
}
