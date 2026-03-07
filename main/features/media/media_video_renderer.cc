/**
 * @file media_video_renderer.cc
 * @brief LVGL canvas video renderer implementation.
 *
 * Mirrors VideoPlayer's CreateVideoCanvas() / DrawFrameToCanvas() pattern
 * but receives decoded RGB565 frames from the av_render video pipeline
 * instead of doing its own JPEG decoding.
 *
 * Memory: canvas buffer is allocated in PSRAM at full LCD resolution.
 *         Typical: 320x240 × 2 bytes = 150 KB PSRAM.
 */

#include "media_video_renderer.h"
#include "display.h"

#include <algorithm>
#include <cstring>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <lvgl.h>

static const char* TAG = "MediaVideoRdr";

MediaVideoRenderer::MediaVideoRenderer() = default;

MediaVideoRenderer::~MediaVideoRenderer() {
    Deinit();
}

bool MediaVideoRenderer::Init(esp_lcd_panel_handle_t panel, uint16_t lcd_width,
                               uint16_t lcd_height, Display* display) {
    if (!display) {
        ESP_LOGE(TAG, "Display required for LVGL canvas rendering");
        return false;
    }

    panel_      = panel;
    lcd_width_  = lcd_width;
    lcd_height_ = lcd_height;
    display_    = display;
    initialized_.store(true);

    ESP_LOGI(TAG, "Initialized (lcd %dx%d)", lcd_width, lcd_height);
    return true;
}

void MediaVideoRenderer::Deinit() {
    if (!initialized_.load()) return;

    DestroyCanvas();
    panel_   = nullptr;
    display_ = nullptr;
    initialized_.store(false);

    ESP_LOGI(TAG, "Deinitialized");
}

/* ================================================================== */
/*  Callbacks from canvas video render vtable                         */
/* ================================================================== */

void MediaVideoRenderer::OnVideoInfo(uint16_t width, uint16_t height,
                                      uint8_t fps, uint8_t frame_type) {
    ESP_LOGI(TAG, "Video info: %dx%d @%dfps type=%d", width, height, fps, frame_type);
    video_width_  = width;
    video_height_ = height;
    CreateCanvas(width, height);
}

void MediaVideoRenderer::OnVideoFrame(const uint8_t* data, int size,
                                       uint16_t width, uint16_t height,
                                       uint32_t pts_ms) {
    if (!video_canvas_ || !canvas_buf_ || !display_) return;

    const auto* src = reinterpret_cast<const uint16_t*>(data);
    uint16_t draw_w = std::min(width, lcd_width_);
    uint16_t draw_h = std::min(height, lcd_height_);
    int x_offset = (lcd_width_  - draw_w) / 2;
    int y_offset = (lcd_height_ - draw_h) / 2;

    /*
     * Copy decoded RGB565 frame data into the LVGL canvas buffer.
     * If the video exactly matches LCD size, use a single fast memcpy.
     * Otherwise, center the frame and clear the border regions.
     * (Same algorithm as VideoPlayer::DrawFrameToCanvas)
     */
    if (x_offset == 0 && draw_w == lcd_width_ &&
        y_offset == 0 && draw_h == lcd_height_) {
        size_t copy_size = draw_w * draw_h * sizeof(uint16_t);
        if (size >= static_cast<int>(copy_size)) {
            memcpy(canvas_buf_, src, copy_size);
        }
    } else {
        /* Clear entire canvas to black */
        memset(canvas_buf_, 0,
               static_cast<size_t>(lcd_width_) * lcd_height_ * sizeof(uint16_t));
        /* Copy video rows with offset */
        for (uint16_t row = 0; row < draw_h; row++) {
            uint16_t* dst_row = canvas_buf_ + (y_offset + row) * lcd_width_ + x_offset;
            const uint16_t* src_row = src + row * width;
            memcpy(dst_row, src_row, draw_w * sizeof(uint16_t));
        }
    }

    /* Invalidate canvas so LVGL redraws on next refresh cycle */
    {
        DisplayLockGuard lock(display_);
        lv_obj_invalidate(static_cast<lv_obj_t*>(video_canvas_));
    }
}

/* ================================================================== */
/*  LVGL canvas lifecycle                                             */
/* ================================================================== */

void MediaVideoRenderer::CreateCanvas(uint16_t video_width, uint16_t video_height) {
    DestroyCanvas();

    if (!display_) return;

    /* Allocate canvas buffer in PSRAM (full LCD size, RGB565) */
    size_t buf_size = static_cast<size_t>(lcd_width_) * lcd_height_ * sizeof(uint16_t);
    canvas_buf_ = static_cast<uint16_t*>(
        heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!canvas_buf_) {
        ESP_LOGE(TAG, "Failed to allocate canvas buffer (%zu bytes)", buf_size);
        return;
    }
    memset(canvas_buf_, 0, buf_size);

    /* Create LVGL canvas (must hold display lock for all LVGL calls) */
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

    ESP_LOGI(TAG, "Canvas created %dx%d (%zu bytes PSRAM)",
             lcd_width_, lcd_height_, buf_size);
}

void MediaVideoRenderer::DestroyCanvas() {
    if (video_canvas_ && display_) {
        DisplayLockGuard lock(display_);
        lv_obj_del(static_cast<lv_obj_t*>(video_canvas_));
        video_canvas_ = nullptr;
    }
    if (canvas_buf_) {
        heap_caps_free(canvas_buf_);
        canvas_buf_ = nullptr;
    }
    video_width_  = 0;
    video_height_ = 0;
}
