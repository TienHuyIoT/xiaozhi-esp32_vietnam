/**
 * @file media_render_factory.cc
 * @brief Implementation of av_render factory functions.
 *
 * Creates I2S audio render and LCD video render instances using
 * the tempotian/av_render default implementations.
 */

#include "media_render_factory.h"
#include "audio_codec.h"

#include <esp_log.h>

extern "C" {
#include "av_render_default.h"
}

static const char* TAG = "MediaRender";

namespace media_render {

audio_render_handle_t CreateAudioRender(AudioCodec* codec) {
    if (!codec) {
        ESP_LOGE(TAG, "AudioCodec is null");
        return nullptr;
    }

    esp_codec_dev_handle_t dev = codec->GetOutputDevHandle();
    if (!dev) {
        ESP_LOGE(TAG, "AudioCodec has no esp_codec_dev output handle");
        return nullptr;
    }

    i2s_render_cfg_t cfg = {};
    cfg.play_handle = dev;
    cfg.cb = nullptr;
    cfg.fixed_clock = false;
    cfg.ctx = nullptr;

    audio_render_handle_t render = av_render_alloc_i2s_render(&cfg);
    if (!render) {
        ESP_LOGE(TAG, "Failed to allocate I2S audio render");
        return nullptr;
    }

    ESP_LOGI(TAG, "I2S audio render created");
    return render;
}

video_render_handle_t CreateVideoRender(esp_lcd_panel_handle_t panel) {
    if (!panel) {
        ESP_LOGE(TAG, "LCD panel handle is null");
        return nullptr;
    }

    lcd_render_cfg_t cfg = {};
    cfg.lcd_handle = panel;
    cfg.rgb_panel = false;       /* SPI panel, not RGB */
    cfg.dsi_panel = false;
    cfg.use_frame_buffer = false;

    video_render_handle_t render = av_render_alloc_lcd_render(&cfg);
    if (!render) {
        ESP_LOGE(TAG, "Failed to allocate LCD video render");
        return nullptr;
    }

    ESP_LOGI(TAG, "LCD video render created");
    return render;
}

}  // namespace media_render
