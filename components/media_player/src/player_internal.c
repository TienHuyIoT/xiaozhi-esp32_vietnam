#include <inttypes.h>
#include "player_internal.h"
#include "esp_extractor_default.h"
#include "av_render.h"
#include "esp_timer.h"

typedef struct {
    media_src_t       *src;
    player_t          *player;
} player_src_wrapper_t;

static int _player_seek(player_t *player);

const char *_player_get_video_format_str(extractor_video_format_t format)
{
    switch (format) {
        case EXTRACTOR_VIDEO_FORMAT_H264:
            return "H264";
        case EXTRACTOR_VIDEO_FORMAT_MJPEG:
            return "MJPEG";
        default:
            return "none";
    }
}

static int get_cur_time()
{
    return esp_timer_get_time() / 1000;
}

#define MAX_MEASURE (10)

#define CURRENT() (uint32_t)(esp_timer_get_time()/100)

typedef struct {
    const char* name;
    uint32_t    start;
    uint32_t    loading;
} measure_item_t;

static bool measure_going = 0;
static uint32_t total_loading;
static measure_item_t measure_items[MAX_MEASURE];

static void show_measure() {
    if (total_loading == 0) {
        return;
    }
    for (int i = 0; i < MAX_MEASURE; i++) {
        if (measure_items[i].name == NULL) {
            break;
        }
        printf("%s %d%%\n", measure_items[i].name, (int)(measure_items[i].loading * 100 / total_loading));
    }
}

void measure_enable(bool enable) {
    if (measure_going == enable) {
        return;
    }
    measure_going = enable;
    if (enable) {
        memset(measure_items, 0, sizeof(measure_items));
        total_loading = CURRENT();
    } else {
        measure_going = false;
        total_loading = CURRENT() - total_loading;
        show_measure();
    }
}

static void measure_start(const char* tag) {
    for (int i = 0; i < MAX_MEASURE; i++) {
        if (measure_items[i].name == NULL || measure_items[i].name == tag) {
            measure_items[i].name = tag;
            measure_items[i].start = CURRENT();
            break;
        }
    }
}

static void measure_stop(const char* tag) {
    for (int i = 0; i < MAX_MEASURE; i++) {
        if (measure_items[i].name == tag) {
            measure_items[i].loading += CURRENT() - measure_items[i].start;
            break;
        }
    }
}

const char *_player_get_audio_format_str(extractor_audio_format_t format)
{
    switch (format) {
        case EXTRACTOR_AUDIO_FORMAT_PCM:
            return "PCM";
        case EXTRACTOR_AUDIO_FORMAT_ADPCM:
            return "ADPCM";
        case EXTRACTOR_AUDIO_FORMAT_AAC:
            return "AAC";
        case EXTRACTOR_AUDIO_FORMAT_MP3:
            return "MP3";
        case EXTRACTOR_AUDIO_FORMAT_AC3:
            return "AC3";
        case EXTRACTOR_AUDIO_FORMAT_VORBIS:
            return "VORBIS";
        case EXTRACTOR_AUDIO_FORMAT_OPUS:
            return "OPUS";
        case EXTRACTOR_AUDIO_FORMAT_FLAC:
            return "FLAC";
        case EXTRACTOR_AUDIO_FORMAT_AMRNB:
            return "AMR-NB";
        case EXTRACTOR_AUDIO_FORMAT_AMRWB:
            return "AMR-WB";
        case EXTRACTOR_AUDIO_FORMAT_G711A:
            return "G711-A";
        case EXTRACTOR_AUDIO_FORMAT_G711U:
            return "G711-U";
        case EXTRACTOR_AUDIO_FORMAT_ALAC:
            return "ALAC";
        default:
            return "none";
    }
}

static av_render_video_codec_t get_render_video_type(extractor_video_format_t video_fmt)
{
    switch (video_fmt) {
        case EXTRACTOR_VIDEO_FORMAT_H264:
            return AV_RENDER_VIDEO_CODEC_H264;
        case EXTRACTOR_VIDEO_FORMAT_MJPEG:
            return AV_RENDER_VIDEO_CODEC_MJPEG;
        default:
            return AV_RENDER_VIDEO_CODEC_NONE;
    }
}

static av_render_audio_codec_t get_render_audio_type(extractor_audio_format_t audio_fmt)
{
    switch (audio_fmt) {
        case EXTRACTOR_AUDIO_FORMAT_PCM:
            return AV_RENDER_AUDIO_CODEC_PCM;
        case EXTRACTOR_AUDIO_FORMAT_ADPCM:
            return AV_RENDER_AUDIO_CODEC_ADPCM;
        case EXTRACTOR_AUDIO_FORMAT_AAC:
            return AV_RENDER_AUDIO_CODEC_AAC;
        case EXTRACTOR_AUDIO_FORMAT_MP3:
            return AV_RENDER_AUDIO_CODEC_MP3;
        case EXTRACTOR_AUDIO_FORMAT_VORBIS:
            return AV_RENDER_AUDIO_CODEC_VORBIS;
        case EXTRACTOR_AUDIO_FORMAT_OPUS:
            return AV_RENDER_AUDIO_CODEC_OPUS;
        case EXTRACTOR_AUDIO_FORMAT_FLAC:
            return AV_RENDER_AUDIO_CODEC_FLAC;
        case EXTRACTOR_AUDIO_FORMAT_AMRNB:
            return AV_RENDER_AUDIO_CODEC_AMRNB;
        case EXTRACTOR_AUDIO_FORMAT_AMRWB:
            return AV_RENDER_AUDIO_CODEC_AMRWB;
        case EXTRACTOR_AUDIO_FORMAT_G711A:
            return AV_RENDER_AUDIO_CODEC_G711A;
        case EXTRACTOR_AUDIO_FORMAT_G711U:
            return AV_RENDER_AUDIO_CODEC_G711U;
        case EXTRACTOR_AUDIO_FORMAT_ALAC:
            return AV_RENDER_AUDIO_CODEC_ALAC;
        default:
            return AV_RENDER_AUDIO_CODEC_NONE;
    }
}

static int player_get_msg(player_t *player, player_msg_t *msg, bool no_wait)
{
    return msg_q_recv(player->event_q, msg, sizeof(player_msg_t), no_wait);
}

static int player_send_sync_msg(player_t *player, player_msg_t *msg, msg_q_handle_t q, media_lib_sema_handle_t sema)
{
    int ret;
    msg->sync_event = true;
    msg->ret_value = &ret;
    ret = msg_q_send(q, msg, sizeof(player_msg_t));
    if (ret == 0) {
        ret = media_lib_sema_lock(sema, 5000);
        if (ret != 0) {
            ESP_LOGE(TAG, "wait for event %d TIMEOUT", msg->player_event);
        }
    }
    return ret;
}

int player_send_msg(player_t *player, player_msg_t *msg)
{
    msg->sync_event = false;
    return msg_q_send(player->event_q, msg, sizeof(player_msg_t));
}

int player_send_play_msg_sync(player_t *player, player_msg_t *msg)
{
    return player_send_sync_msg(player, msg, player->event_q, player->play_sema);
}

void player_set_state(player_t *player, player_state_t state)
{
    player->play_state = state;
}

static bool player_allow_reading(player_t *player)
{
    if (player->eos_sent) {
        return false;
    }
    // Allow send data when seeking even paused
    if (player->play_state == PLAYER_STATE_SEEKING) {
        return true;
    }
    return (player->play_state == PLAYER_STATE_PLAYED) && (player->speed != PLAYER_SPEED_PAUSE);
}

static void notify_player_event(player_t *player, player_event_t event)
{
    if (player->cb) {
        player->cb(event, player->ctx);
    }
}

static media_src_type_t get_src_type(char *url)
{
    if (memcmp(url, "http", 4) == 0) {
        return MEDIA_SRC_TYPE_NETWORK;
    }
    return MEDIA_SRC_TYPE_STORAGE;
}

static void *_wrapper_open(char *uri, void *ctx)
{
    player_t *player = (player_t *) ctx;
    if (player->play_state == PLAYER_STATE_CLOSING) {
        return NULL;
    }
    player_src_wrapper_t *src = (player_src_wrapper_t *) media_lib_malloc(sizeof(player_src_wrapper_t));
    printf("open %p\n", src);
    do {
        if (src == NULL) {
            break;
        }
        src->player = player;
        src->src = media_src_open(get_src_type(uri));
        if (src->src == NULL) {
            break;
        }
        ESP_LOGI(TAG, "Open %p uri %s", src->src, uri);
        if (media_src_connect(src->src, uri) != 0) {
            ESP_LOGI(TAG, "Connect url %s failed", uri);
            break;
        }
        return src;
    } while (0);
    if (src) {
        if (src->src) {
            media_src_close(src->src);
        }
        media_lib_free(src);
    }
    return NULL;
}

static int _wrapper_read(void *data, uint32_t size, void *ctx)
{
    player_src_wrapper_t *src = (player_src_wrapper_t *) ctx;
    if (src && src->src) {
        measure_start("Read");
        int ret = media_src_read(src->src, data, size);
        measure_stop("Read");
        return ret;
    }
    ESP_LOGE(TAG, "Read fail");
    return -1;
}

static int _wrapper_seek(uint32_t position, void *ctx)
{
    player_src_wrapper_t *src = (player_src_wrapper_t *) ctx;
    if (src && src->src) {
        return media_src_seek(src->src, position);
    }
    return -1;
}

static uint32_t _wrapper_file_size(void *ctx)
{
    uint64_t size = 0;
    player_src_wrapper_t *src = (player_src_wrapper_t *) ctx;
    if (src && src->src) {
        media_src_get_size(src->src, &size);
    }
    return (uint32_t) size;
}

static int _wrapper_close(void *ctx)
{
    player_src_wrapper_t *src = (player_src_wrapper_t *) ctx;
    if (src) {
        ESP_LOGI(TAG, "Start to close SRC %p", src->src);
        if (src->src) {
            media_src_close(src->src);
            ESP_LOGI(TAG, "Remove SRC %p", src->src);
        }
        printf("close-free %p\n", src);
        media_lib_free(src);
    }
    return -1;
}

static void _player_start_avsync(player_t* player, int play_mask)
{
    if (play_mask & PLAY_MASK_AUDIO) {
        player->audio_locking = false;
    }
    if (play_mask & PLAY_MASK_VIDEO) {
        player->video_locking = false;
    }
    bool need_resume = false;
    if (player->audio_rendered && player->video_rendered) {
        if (player->audio_locking == false && player->video_locking == false) {
            need_resume = true;
        }
    } else if (player->audio_rendered && player->audio_locking == false) {
        need_resume = true;
    } else if (player->video_rendered && player->video_locking == false) {
        need_resume = true;
    }
    if (need_resume) {
        if (player->speed != PLAYER_SPEED_PAUSE) {
            printf("Unblock for frame reached\n");
            av_render_pause(player->render_handle, false);
        }
        if (player->play_state == PLAYER_STATE_SEEKING) {
            player_set_state(player, PLAYER_STATE_PLAYED);
            ESP_LOGI(TAG, "Seek End");
            notify_player_event(player, PLAYER_EVENT_SEEK_DONE);
        }
    }
}

static int _frame_reached(extractor_frame_info_t *frame, player_t *player, bool full)
{
    if (player->write_pending == false) {
        #if 0
        flush_print("%d type:%d pts:%" PRIu32 " size:%" PRIu32 " eos:%d\n", get_cur_time(),
        frame->stream_type, frame->pts, frame->frame_size, frame->eos);
        #endif
    }
    switch (frame->stream_type) {
        case EXTRACTOR_STREAM_TYPE_AUDIO: {
            if (player->audio_rendered == false) {
                break;
            }
            uint32_t audio_pts = frame->pts;
            // PCM PTS fix: AVI extractor gives chunk index, compute ms from bytes
            if (player->pcm_bytes_per_ms > 0) {
                audio_pts = (uint32_t)(player->pcm_bytes_sent / player->pcm_bytes_per_ms);
            }
            av_render_audio_data_t audio_data = {
                .data = frame->frame_buffer,
                .size = frame->frame_size,
                .pts = audio_pts,
                .eos = frame->eos,
            };
            player->audio_send_pts = audio_pts;
            bool enough = av_render_audio_fifo_enough(player->render_handle, &audio_data);
            // Unlock video for audio is full, but video still not enough
            if (player->video_locking || player->audio_locking) {
                if (enough == false) {
                    if (player->audio_locking) {
                        ESP_LOGE(TAG, "Force to start audio for audio fifo full");
                        _player_start_avsync(player, PLAY_MASK_AUDIO);
                    }
                    if (player->video_locking && player->video_send_pts < player->seeking_time) {
                        ESP_LOGE(TAG, "Force to start video for audio fifo full");
                        _player_start_avsync(player, PLAY_MASK_VIDEO);
                    }
                }
            }
            if (full) {
                return 0;
            }
            if (enough == false) {
                av_render_fifo_stat_t stat = {0};
                av_render_get_audio_fifo_level(player->render_handle, &stat);
                // For PCM audio, no decoder FIFO exists (q_num always 0).
                // Check render_q_num as well to distinguish "FIFO too small" from "temporarily full".
                if (stat.q_num == 0 && stat.render_q_num == 0) {
                    ESP_LOGE(TAG, "Please enlarge AFIFO size to %" PRIu32, frame->frame_size);
                    break;
                }
                if (frame != &player->pending_frame) {
                    memcpy(&player->pending_frame, frame, sizeof(extractor_frame_info_t));
                }
                player->write_pending = true;
                return 0;
            }
            measure_start("Add_Aud");
            av_render_add_audio_data(player->render_handle, &audio_data);
            measure_stop("Add_Aud");
            if (player->pcm_bytes_per_ms > 0) {
                player->pcm_bytes_sent += frame->frame_size;
            }
        }
        break;
        case EXTRACTOR_STREAM_TYPE_VIDEO: {
            av_render_video_data_t video_data = {
                .data = frame->frame_buffer,
                .size = frame->frame_size,
                .pts = frame->pts,
                .eos = frame->eos,
            };
            player->video_send_pts = frame->pts;
            bool enough = av_render_video_fifo_enough(player->render_handle, &video_data);
            if (player->video_locking || player->audio_locking) {
                if (enough == false) {
                    if (player->audio_locking && player->audio_send_pts < player->seeking_time) {
                        ESP_LOGE(TAG, "Force to start audio for video fifo full");
                        _player_start_avsync(player, PLAY_MASK_AUDIO);
                    }
                }
            }
            if (full) {
                return 0;
            }
            if (enough == false) {
                av_render_fifo_stat_t stat = {0};
                av_render_get_video_fifo_level(player->render_handle, &stat);
                if (stat.q_num == 0) {
                    ESP_LOGE(TAG, "Please enlarge VFIFO size to %" PRIu32, frame->frame_size);
                    break;
                }
                if (frame != &player->pending_frame) {
                    memcpy(&player->pending_frame, frame, sizeof(extractor_frame_info_t));
                }
                player->write_pending = true;
                return 0;
            }
            measure_start("Add_Vid");
            av_render_add_video_data(player->render_handle, &video_data);
            measure_stop("Add_Vid");
        }
        break;
        default:
            break;
    }
    player->write_pending = false;
#ifndef REUSE_EXTRACTOR_DATA
    if (frame->frame_buffer) {
        mem_pool_free(esp_extractor_get_output_pool(player->extractor), frame->frame_buffer);
    }
#endif
    return 0;
}

static int _check_eos(player_t* player)
{
    if (player->audio_rendered && player->audio_eos_recv == false) {
        return 0;
    }
    if (player->video_rendered && player->video_eos_recv == false) {
        return 0;
    }
    if (player->looping == false) {
        return 0;
    }
    ESP_LOGI(TAG, "Start to loop playing");
    return media_player_seek_time(player, 0);
}

static int _render_cb(av_render_event_t event, void *ctx)
{
    player_t* player = (player_t*)ctx;
    switch (event) {
        case AV_RENDER_EVENT_AUDIO_RENDERED: {
            ESP_LOGI(TAG, "Audio rendered");
            player_msg_t msg;
            msg.player_event = PLAYER_INTERNAL_MSG_AUDIO_REACHED;
            player_send_msg(player, &msg);
        }
        break;
        case AV_RENDER_EVENT_VIDEO_RENDERED: {
            ESP_LOGI(TAG, "Video rendered");
            player_msg_t msg;
            msg.player_event = PLAYER_INTERNAL_MSG_VIDEO_REACHED;
            player_send_msg(player, &msg);
        }
        break;
        case AV_RENDER_EVENT_AUDIO_EOS: {
            player->audio_eos_recv = true;
            _check_eos(player);
            notify_player_event(player, PLAYER_EVENT_AUDIO_EOS);
        }
        break;
        case AV_RENDER_EVENT_VIDEO_EOS: {
            player->video_eos_recv = true;
            _check_eos(player);
            notify_player_event(player, PLAYER_EVENT_VIDEO_EOS);
        }
        break;
        case AV_RENDER_EVENT_VIDEO_DECODE_ERR: {
            ESP_LOGE(TAG, "Video decoder Error");
            _player_start_avsync(player, PLAY_MASK_VIDEO);
            player_msg_t msg;
            msg.player_event = PLAYER_INTERNAL_MSG_VIDEO_DECODER_ERR;
            player_send_msg(player, &msg);
        }
        break;
        case AV_RENDER_EVENT_AUDIO_DECODE_ERR: {
            ESP_LOGE(TAG, "Audio decoder Error");
            _player_start_avsync(player, PLAY_MASK_AUDIO);
            player_msg_t msg;
            msg.player_event = PLAYER_INTERNAL_MSG_AUDIO_DECODER_ERR;
            player_send_msg(player, &msg);
        }
        break;
        default:
        break;
    }
    return 0;
}

static void _free_pool_data(void* data, void* ctx)
{
    mem_pool_free((mem_pool_handle_t)ctx, data);
}

static int _prepare_render(player_t *player)
{
    uint16_t audio_num = 0;
    uint16_t video_num = 0;
    esp_extractor_get_stream_num(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO, &audio_num);
    esp_extractor_get_stream_num(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO, &video_num);

    av_render_cfg_t render_cfg = {0};
    printf("Get audio %d video %d\n", audio_num, video_num);
    if (audio_num) {
        render_cfg.audio_render = player->audio_render;
        render_cfg.sync_mode = AV_RENDER_SYNC_FOLLOW_AUDIO;
        printf("set audio render %p\n", player->audio_render);
    }
    if (video_num) {
        render_cfg.video_render = player->video_render;
        if (audio_num == 0) {
            render_cfg.sync_mode = AV_RENDER_SYNC_FOLLOW_TIME;
        }
        extractor_stream_info_t stream_info;
        memset(&stream_info, 0, sizeof(stream_info));
        esp_extractor_get_stream_info(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO, 0, &stream_info);
        if (stream_info.stream_info.video_info.format == EXTRACTOR_VIDEO_FORMAT_MJPEG) {
            render_cfg.allow_drop_data = true;
        }
    }
    render_cfg.pause_render_only = true;
    render_cfg.pause_on_first_frame = true;
    if (player->render_handle == NULL) {
        player->render_handle = av_render_open(&render_cfg);
        if (player->render_handle == NULL) {
            return -1;
        }
        av_render_set_event_cb(player->render_handle, _render_cb, player);
#ifdef REUSE_EXTRACTOR_DATA
       av_render_use_data_pool(player->render_handle, _free_pool_data,
                               esp_extractor_get_output_pool(player->extractor));
#endif
    }
    return 0;
}

static int _player_prepare(player_t *player)
{
    if (player->src_type == MEDIA_SRC_TYPE_NONE &&
        player->src_by_callback == false &&
        player->src_by_fifo == false) {
        return -1;
    }
    // already prepared
    if (player->play_state != PLAYER_STATE_NONE) {
        return 0;
    }
    player_set_state(player, PLAYER_STATE_PREPARING);

    esp_extr_err_t ret = ESP_EXTR_ERR_OK;
    player->seeking_time = 0;
    uint32_t output_pool_size = EXTRACTOR_POOL_SIZE;
    if (player->fifo_by_user) {
        output_pool_size = player->fifo_cfg.extractor_pool_size;
    }
    printf("Extractor pool %d\n", (int)output_pool_size);
    if (player->src_type != MEDIA_SRC_TYPE_BT) {
        esp_extractor_config_t config = {
            .open = _wrapper_open,
            .read = _wrapper_read,
            .seek = _wrapper_seek,
            .file_size = _wrapper_file_size,
            .close = _wrapper_close,
            .input_ctx = player,
            .url = player->url,
            .output_pool_size = output_pool_size,
            .output_align = 64,
            .wait_for_output = false,
            .no_accurate_seek = player->no_accurate_seek,
        };
        if (player->play_mask & PLAY_MASK_AUDIO) {
            config.extract_mask |= ESP_EXTRACT_MASK_AUDIO;
        }
        if (player->play_mask & PLAY_MASK_VIDEO) {
            config.extract_mask |= ESP_EXTRACT_MASK_VIDEO;
        }
        if (player->src_by_callback) {
            config.open = NULL;
            config.read = player->src_read_cb;
            config.seek = player->src_seek_cb;
            config.file_size = NULL;
            config.close = NULL;
            config.input_ctx = player->src_cb_ctx;
        } else if (player->src_by_fifo) {
            if (player->extractor_cfg) {
                esp_extractor_free_config(player->extractor_cfg);
            }
            player->extractor_cfg = esp_extractor_alloc_buffer_config(player->src_fifo,
                 player->src_fifo_size, config.extract_mask, output_pool_size);
            if (player->extractor_cfg == NULL) {
                ESP_LOGE(TAG, "Fail to allocate memory for fifo config");
                return ESP_EXTR_ERR_NO_MEM;
            }
        }
        //audio_mem_print(TAG, __LINE__, __func__);
        ret = esp_extractor_open(player->src_by_fifo ? player->extractor_cfg : &config, &player->extractor);
        //audio_mem_print(TAG, __LINE__, __func__);
        if (player->resume_info) {
            ret = esp_extractor_set_resume_info(player->extractor, player->resume_info);
            ret = esp_extractor_free_resume_info(player->resume_info);
            player->seeking_time = player->resume_info->time;
            ESP_LOGI(TAG, "Resume need seek to %" PRIu32, player->seeking_time);
            SAFE_FREE(player->resume_info);
        }
        ret = esp_extractor_parse_stream_info(player->extractor);
    }
    if (ret != ESP_EXTR_ERR_OK) {
        ESP_LOGE(TAG, "Fail to parse stream info ret %d", ret);
        return ret;
    }
    player_set_state(player, PLAYER_STATE_PREPARED);
    player_msg_t msg;
    msg.player_event = PLAYER_INTERNAL_MSG_PREPARED;
    player_send_msg(player, &msg);
    return ret;
}

static int _play_audio(player_t *player)
{
    // select first audio
    extractor_stream_info_t stream_info;
    memset(&stream_info, 0, sizeof(stream_info));
    esp_extractor_get_stream_info(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO, 0, &stream_info);
    player->audio_format = stream_info.stream_info.audio_info.format;
    if (player->audio_format == EXTRACTOR_AUDIO_FORMAT_NONE) {
        // Workaround: AVI extractor may report NONE for PCM audio (format_tag=1)
        extractor_audio_stream_info_t *ai = &stream_info.stream_info.audio_info;
        if (ai->channel > 0 && ai->sample_rate > 0 && ai->bits_per_sample > 0) {
            ESP_LOGW(TAG, "Audio format NONE but valid params (ch:%d sr:%" PRIu32 " bps:%d), assuming PCM",
                     ai->channel, ai->sample_rate, ai->bits_per_sample);
            player->audio_format = EXTRACTOR_AUDIO_FORMAT_PCM;
        } else {
            ESP_LOGW(TAG, "Audio format not recognized, skip audio");
            return -1;
        }
    }
    ESP_LOGI(TAG, "Audio Format %s", _player_get_audio_format_str(player->audio_format));
    extractor_audio_stream_info_t* audio_info = &stream_info.stream_info.audio_info;

    av_render_fifo_cfg_t fifo_cfg = {0};
    if (player->fifo_by_user) {
        fifo_cfg.raw_fifo_size = player->fifo_cfg.adec_fifo_size;
        fifo_cfg.render_fifo_size = player->fifo_cfg.arender_fifo_size;
        ESP_LOGE(TAG, "Audio User fifo raw size %d render size %d", fifo_cfg.raw_fifo_size, fifo_cfg.render_fifo_size);
    } else {
        fifo_cfg.raw_fifo_size = AUDIO_FIFO_SIZE;
        uint32_t fifo_size = AUDIO_RENDER_SAMPLES * audio_info->channel * audio_info->bits_per_sample >> 3;
        if (fifo_size) {
            fifo_cfg.render_fifo_size = fifo_size + 64;
        } else {
            fifo_cfg.render_fifo_size = AUDIO_RENDER_FIFO_SIZE;
        }
        ESP_LOGE(TAG, "Audio default fifo raw size %d render size %d", fifo_cfg.raw_fifo_size, fifo_cfg.render_fifo_size);
    }
    av_render_config_audio_fifo(player->render_handle, &fifo_cfg);

    av_render_audio_info_t audio_render_info = {
        .codec =  get_render_audio_type(player->audio_format),
        .channel = audio_info->channel,
        .sample_rate = audio_info->sample_rate,
        .bits_per_sample = audio_info->bits_per_sample,
        .codec_spec_info = stream_info.spec_info,
        .spec_info_len = stream_info.spec_info_len,
    };

    int ret = av_render_add_audio_stream(player->render_handle, &audio_render_info);
    if (ret) {
        esp_extractor_disable_stream(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO);
        ESP_LOGE(TAG, "fail to open audio decoder %d", stream_info.stream_info.audio_info.format);
        return ret;
    }
    player->audio_locking = true;
    esp_extractor_enable_stream(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO, 0);
    player->stream_playing |= PLAY_MASK_AUDIO;
    player->audio_rendered = true;
    player->audio_send_pts = player->video_send_pts = 0;
    player->seeking_time = 0;
    // For PCM: pre-compute bytes/ms for PTS calculation
    player->pcm_bytes_sent = 0;
    if (player->audio_format == EXTRACTOR_AUDIO_FORMAT_PCM) {
        uint32_t byte_rate = (uint32_t)audio_info->sample_rate * audio_info->channel * (audio_info->bits_per_sample >> 3);
        player->pcm_bytes_per_ms = byte_rate / 1000;
        if (player->pcm_bytes_per_ms == 0) player->pcm_bytes_per_ms = 1;
    } else {
        player->pcm_bytes_per_ms = 0;
    }
    return ret;
}

static int _play_video(player_t *player)
{
    // select first video
    extractor_stream_info_t stream_info;
    memset(&stream_info, 0, sizeof(stream_info));
    esp_extractor_get_stream_info(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO, 0, &stream_info);
    player->video_format = stream_info.stream_info.video_info.format;
    player->video_locking = true;
    if (player->video_format == EXTRACTOR_VIDEO_FORMAT_NONE) {
        return -1;
    }
    ESP_LOGI(TAG, "Video Format %s", _player_get_video_format_str(player->video_format));
    extractor_video_stream_info_t* video_info = &stream_info.stream_info.video_info;
    av_render_fifo_cfg_t fifo_cfg = {0};
    if (player->fifo_by_user) {
        fifo_cfg.raw_fifo_size = player->fifo_cfg.vdec_fifo_size;
        fifo_cfg.render_fifo_size = player->fifo_cfg.vrender_fifo_size;
    } else {
        fifo_cfg.raw_fifo_size = VIDEO_FIFO_SIZE;
        uint32_t fifo_size = video_info->width * video_info->height * 2 * VIDEO_RENDER_FIFO_FPS;
        if (fifo_size) {
            fifo_size += VIDEO_RENDER_FIFO_FPS*64;
            fifo_cfg.render_fifo_size = fifo_size;
        } else {
            fifo_cfg.render_fifo_size = VIDEO_RENDER_FIFO_SIZE;
        }
    }
    av_render_config_video_fifo(player->render_handle, &fifo_cfg);

    av_render_video_info_t video_render_info = {
        .codec =  get_render_video_type(player->video_format),
        .width = video_info->width,
        .height = video_info->height,
        .fps = video_info->fps,
        .codec_spec_info = stream_info.spec_info,
        .spec_info_len = stream_info.spec_info_len,
    };
    int ret = av_render_add_video_stream(player->render_handle, &video_render_info);
    if (ret != 0) {
        esp_extractor_disable_stream(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO);
        ESP_LOGE(TAG, "fail to open video decoder %d", player->video_format);
        return ret;
    }
    ret = esp_extractor_enable_stream(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO, 0);
    player->stream_playing |= PLAY_MASK_VIDEO;
    player->video_rendered = true;
    return ret;
}

static int _player_play(player_t *player)
{
    player->speed = PLAYER_SPEED_1X;
    // only playone time
    if (player->play_state != PLAYER_STATE_PREPARED && player->play_state != PLAYER_STATE_STOPPED) {
        ESP_LOGE(TAG, "Can not play under state:%d", player->play_state);
        return -1;
    }
    player_set_state(player, PLAYER_STATE_PLAYING);

    if (_prepare_render(player) != 0) {
        ESP_LOGE(TAG, "Fail to prepare render");
        player_set_state(player, PLAYER_STATE_PLAY_ERROR);
        return -1;
    }
    int played_stream = 0;
    if (player->play_mask & PLAY_MASK_AUDIO) {
        uint16_t audio_num = 0;
        esp_extractor_get_stream_num(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO, &audio_num);
        if (audio_num > 0) {
            if (_play_audio(player) == 0) {
                played_stream++;
            } else {
                esp_extractor_disable_stream(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO);
            }
        }
    }
    if (player->play_mask & PLAY_MASK_VIDEO) {
        uint16_t video_num = 0;
        esp_extractor_get_stream_num(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO, &video_num);
        if (video_num > 0) {
            if (_play_video(player) ==  0) {
                played_stream++;
            }
        }
    }
    // If one stream can played
    if (played_stream) {
        player_set_state(player, PLAYER_STATE_PLAYED);
        if (player->seeking_time) {
            _player_seek(player);
        }
    } else {
        player_set_state(player, PLAYER_STATE_PLAY_ERROR);
    }
    return 0;
}

static int _player_stop_audio(player_t *player)
{
    // wait decoder exit
    player->stream_playing &= ~PLAY_MASK_AUDIO;
    av_render_audio_info_t audio_info = {0};
    flush_print("%d start close audio render\n", get_cur_time());
    av_render_add_audio_stream(player->render_handle, &audio_info);
    flush_print("%d audio render close done\n", get_cur_time());
    player->audio_rendered = false;
    player->audio_locking = false;
    return 0;
}

static int _player_stop_video(player_t *player)
{
    player->stream_playing &= ~PLAY_MASK_VIDEO;
    av_render_video_info_t video_info = {0};
    flush_print("%d start close video render\n", get_cur_time());
    av_render_add_video_stream(player->render_handle, &video_info);
    flush_print("%d video render close done\n", get_cur_time());
    player->video_rendered = false;
    player->video_locking = false;
    return 0;
}

static int _player_stop(player_t *player)
{
    if (player->play_state == PLAYER_STATE_STOPPED) {
        return 0;
    }

    ESP_LOGI(TAG, "%d Player Stop Start", get_cur_time());
    player_set_state(player, PLAYER_STATE_STOPPING);
    // Extractor resource is kept so that it can play again
    if (player->render_handle) {
        av_render_close(player->render_handle);
        player->render_handle = NULL;
    }
    player->audio_rendered = 0;
    player->video_rendered = 0;
    player->eos_sent = false;
    player_set_state(player, PLAYER_STATE_STOPPED);
    ESP_LOGI(TAG, "%d Player Stop End", get_cur_time());
    return 0;
}

static int _player_close(player_t *player)
{
    ESP_LOGI(TAG, "%d Player Close Start", get_cur_time());
    _player_stop(player);
    if (player->extractor) {
        esp_extractor_close(player->extractor);
        player->extractor = NULL;
    }
    if (player->event_q) {
        msg_q_destroy(player->event_q);
        player->event_q = NULL;
    }
    if (player->resume_info) {
        esp_extractor_free_resume_info(player->resume_info);
        SAFE_FREE(player->resume_info);
    }
    if (player->extractor_cfg) {
        esp_extractor_free_config(player->extractor_cfg);
        player->extractor_cfg = NULL;
    }
    ESP_LOGI(TAG, "%d Player Close End", get_cur_time());
    return 0;
}

static int _player_interrupt(player_t *player)
{
    ESP_LOGI(TAG, "%d Player Interrupt Start", get_cur_time());
    _player_stop(player);
    if (player->extractor) {
        esp_extractor_close(player->extractor);
        player->extractor = NULL;
    }
    // keep media src and resume info
    player_set_state(player, PLAYER_STATE_NONE);
    ESP_LOGI(TAG, "%d Player Interrupt End", get_cur_time());
    return 0;
}

static int _player_set_speed(player_t *player)
{
    // not allow set speed when seeking, add not allow set speed condition
    if (player->play_state == PLAYER_STATE_SEEKING) {
        return -1;
    }
    if (player->set_speed == player->speed) {
        return 0;
    }
    if (player->set_speed == PLAYER_SPEED_PAUSE) {
        av_render_pause(player->render_handle, true);
        notify_player_event(player, PLAYER_EVENT_PLAYED_PAUSE);
    } else {
        av_render_pause(player->render_handle, false);
        av_render_set_speed(player->render_handle, player->set_speed);
        notify_player_event(player, PLAYER_EVENT_PLAYED_RESUME);
    }
    player->speed = player->set_speed;
    return 0;
}

void mem_pool_dump(void*);

static int _player_seek(player_t *player)
{
    if (player->play_state != PLAYER_STATE_PLAYED) {
        flush_print("Not allow seek if not playing state:%d\n", player->play_state);
        return -1;
    }
    player_set_state(player, PLAYER_STATE_SEEKING);
    ESP_LOGI(TAG, "Seek to %" PRIu32 " Start",  player->seeking_time);
    if (player->render_handle) {
        av_render_flush(player->render_handle);
    }
    if (player->audio_rendered) {
        player->audio_locking = true;
    }
    if (player->video_rendered) {
        player->video_locking = true;
    }
    // mem_pool_dump(esp_extractor_get_output_pool(player->extractor));
    player->audio_eos_recv = false;
    player->video_eos_recv = false;
    player->eos_sent = false;
    player->audio_send_pts = player->video_send_pts = 0;
    if (player->extractor) {
        esp_extractor_seek(player->extractor, player->seeking_time);
    }
    if (player->video_rendered) {
        uint32_t seek_to = player->seeking_time;
        if (player->no_accurate_seek) {
            seek_to = 0;
        }
        av_render_set_video_start_pts(player->render_handle, seek_to);
    }
    // Resume again
    av_render_pause(player->render_handle, false);
    // when set pause on first frame video and audio will pause automatically
    return 0;
}

static int _player_add_eos(player_t *player)
{
    extractor_frame_info_t frame = {.eos = true};
    if (player->audio_rendered) {
        frame.stream_type = EXTRACTOR_STREAM_TYPE_AUDIO;
        _frame_reached(&frame, player, false);
    }
    if (player->video_rendered) {
        frame.stream_type = EXTRACTOR_STREAM_TYPE_VIDEO;
        _frame_reached(&frame, player, false);
    }
    return 0;
}

uint32_t _player_position(player_t *player)
{
    uint32_t pts = 0;
    av_render_get_render_pts(player->render_handle, &pts);
    return pts;
}

static int _player_check_slow_done(player_t *player)
{
    // PCM has no decoder stage; render FIFO is naturally small.
    // Slow-down is only useful for compressed codecs with decoder bottleneck.
    if (player->audio_format == EXTRACTOR_AUDIO_FORMAT_PCM) {
        return 0;
    }
    if (player->audio_rendered &&
        player->audio_locking == false &&
        player->speed == PLAYER_SPEED_1X) {
        av_render_fifo_stat_t fifo_stat = {};
        av_render_get_audio_fifo_level(player->render_handle, &fifo_stat);
        bool need_slow_down = (fifo_stat.render_q_num <= 2);
        if (need_slow_down != player->audio_slow_down) {
            if (need_slow_down == false) {
                player->audio_enough_count++;
                if (player->audio_enough_count < 100) {
                    return 0;
                }
                player->audio_enough_count = 0;
            }
            float speed = need_slow_down ? 0.6 : player->speed;
            av_render_set_speed(player->render_handle, speed);
            ESP_LOGI(TAG, "Audio slow down now %d", need_slow_down);
            player->audio_slow_down = need_slow_down;
        }
    }
    return 0;
}

static int _player_check_underflow(player_t *player)
{
    bool underflow = false;
    if (player->src_type != MEDIA_SRC_TYPE_NETWORK) {
        return 0;
    }
    return 0;
    // not underflow when eos
    if (player->eos_sent || player->audio_rendered == false) {
        return 0;
    }
    if (player->audio_rendered) {
        av_render_fifo_stat_t fifo_stat;
        av_render_get_audio_fifo_level(player->render_handle, &fifo_stat);
        if (fifo_stat.duration < 50) {
            underflow = true;
        } else if (player->underflowing && fifo_stat.duration >= 500) {
            underflow = true;
        }
    }
    if (player->video_rendered) {
        av_render_fifo_stat_t fifo_stat;
        av_render_get_audio_fifo_level(player->render_handle, &fifo_stat);
        if (fifo_stat.duration < 100) {
            underflow = true;
        } else if (player->underflowing && fifo_stat.duration >= 1000) {
            underflow = true;
        }
    }
    if (player->underflowing != underflow) {
        ESP_LOGE(TAG, "Underflow %d\n", underflow);
        av_render_pause(player->render_handle, underflow);
        player->underflowing = underflow;
    }
    return 0;
}

static void check_unlock_for_full(player_t* player)
{
    if (player->audio_locking == false && player->video_locking == false) {
        return;
    }
    av_render_fifo_stat_t stat = {0};
    extractor_stream_type_t stream_type = player->pending_frame.stream_type;
    if (stream_type == EXTRACTOR_STREAM_TYPE_AUDIO) {
        av_render_get_video_fifo_level(player->render_handle, &stat);
    } else {
        av_render_get_audio_fifo_level(player->render_handle, &stat);
    }
    if (stat.q_num == 0) {
        _player_start_avsync(player,
            stream_type == EXTRACTOR_STREAM_TYPE_AUDIO ? PLAY_MASK_VIDEO : PLAY_MASK_AUDIO);
        // ESP_LOGE(TAG, "Force %d for fifo full", stream_type);
    }
}

static void player_release_pending(player_t* player, bool force)
{
    // Process pending frame
    if (player->write_pending) {
        if (force) {
            player->write_pending = false;
            if (player->pending_frame.frame_buffer) {
                mem_pool_free(esp_extractor_get_output_pool(player->extractor), player->pending_frame.frame_buffer);
                player->pending_frame.frame_buffer = NULL;
            }
        } else {
            check_unlock_for_full(player);
            _frame_reached(&player->pending_frame, player, false);
        }
    }
    // Unhandled case
    // Audio fifo is full, video still not decoded out
}

static int player_handle_msg(player_t* player, player_msg_t* msg)
{
    flush_print("Player get msg:%d\n", msg->player_event);
    int running = 1;
    switch (msg->player_event) {
        case PLAYER_INTERNAL_MSG_CLOSE:
            player_release_pending(player, true);
            _player_close(player);
            running = 0;
            player->play_thread = NULL;
            break;

        case PLAYER_INTERNAL_MSG_INTERRUPT:
            player_release_pending(player, true);
            _player_interrupt(player);
            running = 0;
            player->play_thread = NULL;
            break;

        case PLAYER_INTERNAL_MSG_STOP:
            player_release_pending(player, true);
            _player_stop(player);
            break;

        case PLAYER_INTERNAL_MSG_VIDEO_DECODER_ERR:
            ESP_LOGE(TAG, "Video can not decoded");
            esp_extractor_disable_stream(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO);
            _player_stop_video(player);
            _player_start_avsync(player, PLAY_MASK_AUDIO);
            notify_player_event(player, PLAYER_EVENT_VIDEO_NOT_SUPPORT);
            break;

        case PLAYER_INTERNAL_MSG_AUDIO_DECODER_ERR:
            ESP_LOGE(TAG, "Audio can not decoded");
            esp_extractor_disable_stream(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO);
            _player_stop_audio(player);
            _player_start_avsync(player, PLAY_MASK_AUDIO);
            notify_player_event(player, PLAYER_EVENT_AUDIO_NOT_SUPPORT);
            break;

        case PLAYER_INTERNAL_MSG_PREPARE:
            if (_player_prepare(player) == 0) {
                notify_player_event(player, PLAYER_EVENT_PREPARED);
            } else {
                notify_player_event(player, PLAYER_EVENT_PREPARE_ERROR);
            }
            break;

        case PLAYER_INTERNAL_MSG_AUDIO_REACHED:
            _player_start_avsync(player, PLAY_MASK_AUDIO);
            break;

        case PLAYER_INTERNAL_MSG_VIDEO_REACHED:
            _player_start_avsync(player, PLAY_MASK_VIDEO);
            break;

        case PLAYER_INTERNAL_MSG_PREPARED:
            if (player->auto_start) {
                _player_play(player);
            }
            break;

        case PLAYER_INTERNAL_MSG_PLAY:
            flush_print("Curent state:%d\n", player->play_state);
            if (player->play_state == PLAYER_STATE_NONE) {
                player->auto_start = true;
                player_msg_t msg;
                msg.player_event = PLAYER_INTERNAL_MSG_PREPARE;
                player_send_msg(player, &msg);
            } else {
                // when replay seek to 0
                esp_extractor_seek(player->extractor, 0);
                _player_play(player);
            }
            break;

        case PLAYER_INTERNAL_MSG_SEEK:
            player_release_pending(player, true);
            _player_seek(player);
            break;

        case PLAYER_INTERNAL_MSG_SET_SPEED:
            _player_set_speed(player);
            break;
        default:
            break;
    }
    if (msg->sync_event) {
        media_lib_sema_unlock(player->play_sema);
    }
    return running;
}

void player_thread(void *ctx)
{
    player_t *player = ctx;
    int ret;
    bool no_wait = false;
    player_msg_t recv_msg;
    while (1) {
        ret = player_get_msg(player, &recv_msg, no_wait);
        if (ret == 0) {
            int running = player_handle_msg(player, &recv_msg);
            if (running == 0) {
                break;
            }
        }
        no_wait = player_allow_reading(player);
        if (no_wait == false) {
            continue;
        }
        // TODO use 2 threads to handle data and command separately better
        if (player->write_pending) {
            measure_start("Pending");
            player_release_pending(player, false);
            _player_check_slow_done(player);
            media_lib_thread_sleep(10);
            measure_stop("Pending");
            continue;
        }
        extractor_frame_info_t frame_info;
        measure_start("Read_Frame");
        ret = esp_extractor_read_frame(player->extractor, &frame_info);
        measure_stop("Read_Frame");
        if (ret >= 0) {
            bool full = (ret == ESP_EXTR_ERR_WAITING_OUTPUT);
            measure_start("P_Frame");
            _frame_reached(&frame_info, player, full);
            measure_stop("P_Frame");
        }
        // For HLS new url not generated yet
        if (ret == ESP_EXTR_ERR_WAITING_NEW_URL || ret == ESP_EXTR_ERR_WAITING_OUTPUT) {
            media_lib_thread_sleep(50);
            continue;
        }
        if (ret != ESP_EXTR_ERR_OK && ret != ESP_EXTR_ERR_SKIPPED) {
            ESP_LOGI(TAG, "Quit Extractor for %d\n", ret);
            if (player->eos_sent == false) {
                notify_player_event(player, PLAYER_EVENT_EOS);
                _player_add_eos(player);
                player->eos_sent = true;
            }
        }
        _player_check_underflow(player);
    }
    media_lib_thread_destroy(NULL);
}
