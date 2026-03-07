#ifndef MEDIA_RENDER_FACTORY_H
#define MEDIA_RENDER_FACTORY_H

/**
 * @file media_render_factory.h
 * @brief Factory functions to create av_render audio/video render handles.
 *
 * Bridges the project's AudioCodec and LcdDisplay abstractions to the
 * tempotian/av_render I2S and LCD render implementations.
 */

#include <esp_lcd_panel_ops.h>
#include <esp_codec_dev.h>

/* Forward-declare opaque av_render handle types */
typedef void* audio_render_handle_t;
typedef void* video_render_handle_t;

class AudioCodec;

namespace media_render {

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

}  // namespace media_render

#endif // MEDIA_RENDER_FACTORY_H
