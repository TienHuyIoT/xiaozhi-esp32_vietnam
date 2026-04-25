#ifndef MEDIA_VIDEO_RENDERER_H
#define MEDIA_VIDEO_RENDERER_H

/**
 * @file media_video_renderer.h
 * @brief Internal LVGL canvas video renderer for MediaPlayerService.
 *
 * Manages an LVGL canvas sized to the LCD dimensions and draws each decoded
 * video frame into the canvas buffer.  Used by MediaPlayerService when
 * render_mode == kLvglCanvas.  Mirrors VideoPlayer's DrawFrameToCanvas()
 * and CreateVideoCanvas() patterns but works with tempotian/media_player's
 * decoded video frames (RGB565) delivered through the av_render vtable.
 *
 * Lifecycle:
 *   MediaPlayerService creates this object → Init() with display params →
 *   OnVideoInfo() when format is known → OnVideoFrame() per decoded frame →
 *   Deinit() on shutdown.
 */

#include <cstdint>
#include <atomic>

/* Forward declarations — avoid dragging esp_lcd / LVGL into every includer */
struct esp_lcd_panel_t;
typedef struct esp_lcd_panel_t* esp_lcd_panel_handle_t;

class Display;

class MediaVideoRenderer {
public:
    MediaVideoRenderer();
    ~MediaVideoRenderer();

    /**
     * @brief Initialize with display parameters.
     *
     * @param panel      LCD panel handle (reserved for future direct-draw)
     * @param lcd_width  Display width in pixels
     * @param lcd_height Display height in pixels
     * @param display    Display instance (required for LVGL lock)
     * @return true on success
     */
    bool Init(esp_lcd_panel_handle_t panel, uint16_t lcd_width,
              uint16_t lcd_height, Display* display);

    /** Release canvas and all allocated buffers. */
    void Deinit();

    /**
     * @brief Called when video format is determined (before first frame).
     *
     * Creates the LVGL canvas sized to the LCD dimensions and prepares
     * the buffer for receiving frame data.
     */
    void OnVideoInfo(uint16_t width, uint16_t height,
                     uint8_t fps, uint8_t frame_type);

    /**
     * @brief Called for each decoded video frame (RGB565).
     *
     * Copies frame data into the LVGL canvas buffer (centered if video
     * is smaller than LCD) and invalidates the widget for redraw.
     *
     * @note Called from av_render's video render thread. Acquires the
     *       LVGL display lock briefly for invalidation only.
     */
    void OnVideoFrame(const uint8_t* data, int size,
                      uint16_t width, uint16_t height, uint32_t pts_ms);

    bool IsInitialized() const { return initialized_.load(); }

private:
    void CreateCanvas(uint16_t video_width, uint16_t video_height);
    void DestroyCanvas();

    esp_lcd_panel_handle_t panel_  = nullptr;
    uint16_t               lcd_width_  = 0;
    uint16_t               lcd_height_ = 0;
    Display*               display_    = nullptr;

    /* LVGL canvas (type-erased to avoid lvgl.h dependency in header) */
    void*     video_canvas_  = nullptr;   // lv_obj_t*
    uint16_t* canvas_buf_    = nullptr;   // PSRAM buffer for LVGL canvas
    uint16_t  video_width_   = 0;
    uint16_t  video_height_  = 0;

    std::atomic<bool> initialized_{false};
};

#endif // MEDIA_VIDEO_RENDERER_H
