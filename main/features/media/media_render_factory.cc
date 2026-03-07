/**
 * @file media_render_factory.cc
 * @brief Implementation of av_render factory functions.
 *
 * Creates render instances for two modes:
 *   1. Hardware: I2S audio render and LCD video render using av_render defaults.
 *   2. Callback: Custom renders that forward decoded frames to user callbacks,
 *      similar to AVI player's video_cb/audio_cb pattern.
 */

#include "media_render_factory.h"
#include "media_player_service.h"
#include "media_video_renderer.h"
#include "audio_codec.h"

#include <cstring>
#include <esp_log.h>

extern "C" {
#include "av_render_default.h"
#include "audio_render.h"
#include "video_render.h"
}

static const char* TAG = "MediaRender";

/* ================================================================== */
/*  Callback audio render implementation                              */
/* ================================================================== */

namespace {

/** Internal context for callback audio render. */
struct CbAudioCtx {
    MediaPlayerService* service;
    av_render_audio_frame_info_t info;
};

static audio_render_handle_t cb_audio_init(void* cfg, int cfg_size) {
    if (!cfg || cfg_size < (int)sizeof(CbAudioCtx)) return nullptr;
    auto* ctx = static_cast<CbAudioCtx*>(calloc(1, sizeof(CbAudioCtx)));
    if (!ctx) return nullptr;
    memcpy(ctx, cfg, sizeof(CbAudioCtx));
    return ctx;
}

static int cb_audio_open(audio_render_handle_t render, av_render_audio_frame_info_t* info) {
    auto* ctx = static_cast<CbAudioCtx*>(render);
    if (!ctx || !info) return -1;
    ctx->info = *info;
    const auto& cbs = ctx->service->GetCallbacks();
    if (cbs.audio_clock_cb) {
        cbs.audio_clock_cb(info->sample_rate, info->bits_per_sample,
                           info->channel, cbs.user_data);
    }
    return 0;
}

static int cb_audio_write(audio_render_handle_t render, av_render_audio_frame_t* audio_data) {
    auto* ctx = static_cast<CbAudioCtx*>(render);
    if (!ctx || !audio_data) return -1;
    const auto& cbs = ctx->service->GetCallbacks();
    if (cbs.audio_cb) {
        cbs.audio_cb(audio_data->data, audio_data->size,
                     audio_data->pts, cbs.user_data);
    }
    return 0;
}

static int cb_audio_get_latency(audio_render_handle_t, uint32_t* latency) {
    if (latency) *latency = 0;
    return 0;
}

static int cb_audio_get_frame_info(audio_render_handle_t render, av_render_audio_frame_info_t* info) {
    auto* ctx = static_cast<CbAudioCtx*>(render);
    if (!ctx || !info) return -1;
    *info = ctx->info;
    return 0;
}

static int cb_audio_set_speed(audio_render_handle_t, float) {
    return 0;
}

static int cb_audio_close(audio_render_handle_t) {
    return 0;
}

static void cb_audio_deinit(audio_render_handle_t render) {
    free(render);
}

/* ================================================================== */
/*  Callback video render implementation                              */
/* ================================================================== */

/** Internal context for callback video render. */
struct CbVideoCtx {
    MediaPlayerService* service;
    av_render_video_frame_info_t info;
};

static video_render_handle_t cb_video_open(void* cfg, int cfg_size) {
    if (!cfg || cfg_size < (int)sizeof(CbVideoCtx)) return nullptr;
    auto* ctx = static_cast<CbVideoCtx*>(calloc(1, sizeof(CbVideoCtx)));
    if (!ctx) return nullptr;
    memcpy(ctx, cfg, sizeof(CbVideoCtx));
    return ctx;
}

static bool cb_video_format_support(video_render_handle_t, av_render_video_frame_type_t type) {
    /* Accept RGB565 and RGB565_BE which are the most common decoded formats */
    return type == AV_RENDER_VIDEO_RAW_TYPE_RGB565 ||
           type == AV_RENDER_VIDEO_RAW_TYPE_RGB565_BE ||
           type == AV_RENDER_VIDEO_RAW_TYPE_YUV420 ||
           type == AV_RENDER_VIDEO_RAW_TYPE_YUV422;
}

static int cb_video_set_frame_info(video_render_handle_t render, av_render_video_frame_info_t* info) {
    auto* ctx = static_cast<CbVideoCtx*>(render);
    if (!ctx || !info) return -1;
    ctx->info = *info;
    const auto& cbs = ctx->service->GetCallbacks();
    if (cbs.video_info_cb) {
        cbs.video_info_cb(info->width, info->height, info->fps,
                          static_cast<uint8_t>(info->type), cbs.user_data);
    }
    return 0;
}

static int cb_video_get_frame_buffer(video_render_handle_t, av_render_frame_buffer_t*) {
    return -1;  /* No direct frame buffer; use write path */
}

static int cb_video_write(video_render_handle_t render, av_render_video_frame_t* video_data) {
    auto* ctx = static_cast<CbVideoCtx*>(render);
    if (!ctx || !video_data) return -1;
    const auto& cbs = ctx->service->GetCallbacks();
    if (cbs.video_cb) {
        cbs.video_cb(video_data->data, video_data->size,
                     ctx->info.width, ctx->info.height,
                     video_data->pts, cbs.user_data);
    }
    return 0;
}

static int cb_video_get_latency(video_render_handle_t, uint32_t* latency) {
    if (latency) *latency = 0;
    return 0;
}

static int cb_video_get_frame_info(video_render_handle_t render, av_render_video_frame_info_t* info) {
    auto* ctx = static_cast<CbVideoCtx*>(render);
    if (!ctx || !info) return -1;
    *info = ctx->info;
    return 0;
}

static int cb_video_clear(video_render_handle_t) {
    return 0;
}

static int cb_video_close(video_render_handle_t render) {
    free(render);
    return 0;
}

/* ================================================================== */
/*  Canvas video render implementation (LVGL canvas)                  */
/* ================================================================== */

/** Internal context for canvas video render. */
struct CanvasVideoCtx {
    MediaVideoRenderer* renderer;
    av_render_video_frame_info_t info;
};

static video_render_handle_t canvas_video_open(void* cfg, int cfg_size) {
    if (!cfg || cfg_size < (int)sizeof(CanvasVideoCtx)) return nullptr;
    auto* ctx = static_cast<CanvasVideoCtx*>(calloc(1, sizeof(CanvasVideoCtx)));
    if (!ctx) return nullptr;
    memcpy(ctx, cfg, sizeof(CanvasVideoCtx));
    return ctx;
}

static bool canvas_video_format_support(video_render_handle_t, av_render_video_frame_type_t type) {
    /* Accept RGB565 formats — LVGL canvas uses RGB565 */
    return type == AV_RENDER_VIDEO_RAW_TYPE_RGB565 ||
           type == AV_RENDER_VIDEO_RAW_TYPE_RGB565_BE;
}

static int canvas_video_set_frame_info(video_render_handle_t render, av_render_video_frame_info_t* info) {
    auto* ctx = static_cast<CanvasVideoCtx*>(render);
    if (!ctx || !info) return -1;
    ctx->info = *info;
    if (ctx->renderer) {
        ctx->renderer->OnVideoInfo(info->width, info->height, info->fps,
                                    static_cast<uint8_t>(info->type));
    }
    return 0;
}

static int canvas_video_get_frame_buffer(video_render_handle_t, av_render_frame_buffer_t*) {
    return -1;  /* No direct frame buffer; use write path */
}

static int canvas_video_write(video_render_handle_t render, av_render_video_frame_t* video_data) {
    auto* ctx = static_cast<CanvasVideoCtx*>(render);
    if (!ctx || !video_data) return -1;
    if (ctx->renderer) {
        ctx->renderer->OnVideoFrame(video_data->data, video_data->size,
                                     ctx->info.width, ctx->info.height,
                                     video_data->pts);
    }
    return 0;
}

static int canvas_video_get_latency(video_render_handle_t, uint32_t* latency) {
    if (latency) *latency = 0;
    return 0;
}

static int canvas_video_get_frame_info(video_render_handle_t render, av_render_video_frame_info_t* info) {
    auto* ctx = static_cast<CanvasVideoCtx*>(render);
    if (!ctx || !info) return -1;
    *info = ctx->info;
    return 0;
}

static int canvas_video_clear(video_render_handle_t) {
    return 0;
}

static int canvas_video_close(video_render_handle_t render) {
    free(render);
    return 0;
}

}  // anonymous namespace

/* ================================================================== */
/*  Factory: hardware renders                                         */
/* ================================================================== */

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
    cfg.rgb_panel = false;
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

/* ================================================================== */
/*  Factory: callback renders                                         */
/* ================================================================== */

audio_render_handle_t CreateCallbackAudioRender(MediaPlayerService* service) {
    if (!service) {
        ESP_LOGE(TAG, "MediaPlayerService is null");
        return nullptr;
    }

    CbAudioCtx init_ctx = {};
    init_ctx.service = service;

    audio_render_cfg_t cfg = {};
    cfg.ops.init           = cb_audio_init;
    cfg.ops.open           = cb_audio_open;
    cfg.ops.write          = cb_audio_write;
    cfg.ops.get_latency    = cb_audio_get_latency;
    cfg.ops.get_frame_info = cb_audio_get_frame_info;
    cfg.ops.set_speed      = cb_audio_set_speed;
    cfg.ops.close          = cb_audio_close;
    cfg.ops.deinit         = cb_audio_deinit;
    cfg.cfg                = &init_ctx;
    cfg.cfg_size           = sizeof(init_ctx);

    audio_render_handle_t render = audio_render_alloc_handle(&cfg);
    if (!render) {
        ESP_LOGE(TAG, "Failed to allocate callback audio render");
        return nullptr;
    }

    ESP_LOGI(TAG, "Callback audio render created");
    return render;
}

video_render_handle_t CreateCallbackVideoRender(MediaPlayerService* service) {
    if (!service) {
        ESP_LOGE(TAG, "MediaPlayerService is null");
        return nullptr;
    }

    CbVideoCtx init_ctx = {};
    init_ctx.service = service;

    video_render_cfg_t cfg = {};
    cfg.ops.open             = cb_video_open;
    cfg.ops.format_support   = cb_video_format_support;
    cfg.ops.set_frame_info   = cb_video_set_frame_info;
    cfg.ops.get_frame_buffer = cb_video_get_frame_buffer;
    cfg.ops.write            = cb_video_write;
    cfg.ops.get_latency      = cb_video_get_latency;
    cfg.ops.get_frame_info   = cb_video_get_frame_info;
    cfg.ops.clear            = cb_video_clear;
    cfg.ops.close            = cb_video_close;
    cfg.cfg                  = &init_ctx;
    cfg.cfg_size             = sizeof(init_ctx);

    video_render_handle_t render = video_render_alloc_handle(&cfg);
    if (!render) {
        ESP_LOGE(TAG, "Failed to allocate callback video render");
        return nullptr;
    }

    ESP_LOGI(TAG, "Callback video render created");
    return render;
}

/* ================================================================== */
/*  Factory: canvas video render (LVGL canvas)                        */
/* ================================================================== */

video_render_handle_t CreateCanvasVideoRender(MediaVideoRenderer* renderer) {
    if (!renderer) {
        ESP_LOGE(TAG, "MediaVideoRenderer is null");
        return nullptr;
    }

    CanvasVideoCtx init_ctx = {};
    init_ctx.renderer = renderer;

    video_render_cfg_t cfg = {};
    cfg.ops.open             = canvas_video_open;
    cfg.ops.format_support   = canvas_video_format_support;
    cfg.ops.set_frame_info   = canvas_video_set_frame_info;
    cfg.ops.get_frame_buffer = canvas_video_get_frame_buffer;
    cfg.ops.write            = canvas_video_write;
    cfg.ops.get_latency      = canvas_video_get_latency;
    cfg.ops.get_frame_info   = canvas_video_get_frame_info;
    cfg.ops.clear            = canvas_video_clear;
    cfg.ops.close            = canvas_video_close;
    cfg.cfg                  = &init_ctx;
    cfg.cfg_size             = sizeof(init_ctx);

    video_render_handle_t render = video_render_alloc_handle(&cfg);
    if (!render) {
        ESP_LOGE(TAG, "Failed to allocate canvas video render");
        return nullptr;
    }

    ESP_LOGI(TAG, "Canvas video render created");
    return render;
}

}  // namespace media_render
