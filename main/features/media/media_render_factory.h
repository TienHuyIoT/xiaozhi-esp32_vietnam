#ifndef MEDIA_RENDER_FACTORY_H
#define MEDIA_RENDER_FACTORY_H

/**
 * @file media_render_factory.h
 * @brief Factory functions to create av_render audio/video render handles.
 *
 * Supports two render modes:
 *   1. Hardware renders: Bridge AudioCodec/LcdDisplay to I2S/LCD av_render.
 *   2. Callback renders: Forward decoded frames to user callbacks
 *      (similar to AVI player's video_cb/audio_cb pattern).
 */

#include <esp_lcd_panel_ops.h>
#include <esp_codec_dev.h>

/* Forward-declare opaque av_render handle types */
typedef void* audio_render_handle_t;
typedef void* video_render_handle_t;

class AudioCodec;
class MediaPlayerService;
class MediaVideoRenderer;

namespace media_render {

/* ---- Hardware renders (internal I2S/LCD output) ---- */

/**
 * @brief Create an I2S audio render for media_player.
 * @param codec  AudioCodec instance (must support GetOutputDevHandle)
 * @return audio_render_handle_t, or nullptr on failure
 */
audio_render_handle_t CreateAudioRender(AudioCodec* codec);

/**
 * @brief Create an LCD video render for media_player.
 * @param panel  ESP LCD panel handle from LcdDisplay::GetPanelHandle()
 * @return video_render_handle_t, or nullptr on failure
 */
video_render_handle_t CreateVideoRender(esp_lcd_panel_handle_t panel);

/* ---- Callback renders (user-handled rendering) ---- */

/**
 * @brief Create a callback-based audio render.
 *
 * Decoded PCM audio frames are forwarded to the MediaPlayerService's
 * registered audio callbacks instead of being written to I2S hardware.
 *
 * @param service  MediaPlayerService instance (owns the callbacks)
 * @return audio_render_handle_t, or nullptr on failure
 */
audio_render_handle_t CreateCallbackAudioRender(MediaPlayerService* service);

/**
 * @brief Create a callback-based video render.
 *
 * Decoded video frames are forwarded to the MediaPlayerService's
 * registered video callbacks instead of being drawn to LCD panel.
 *
 * @param service  MediaPlayerService instance (owns the callbacks)
 * @return video_render_handle_t, or nullptr on failure
 */
video_render_handle_t CreateCallbackVideoRender(MediaPlayerService* service);

/* ---- Canvas render (LVGL canvas video output) ---- */

/**
 * @brief Create a canvas-based video render for LVGL canvas mode.
 *
 * Decoded video frames are forwarded to MediaVideoRenderer which
 * manages an LVGL canvas widget for display.  Audio still uses I2S.
 *
 * @param renderer  MediaVideoRenderer instance (owns the canvas)
 * @return video_render_handle_t, or nullptr on failure
 */
video_render_handle_t CreateCanvasVideoRender(MediaVideoRenderer* renderer);

}  // namespace media_render

#endif // MEDIA_RENDER_FACTORY_H
