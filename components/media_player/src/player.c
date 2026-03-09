#include "player_internal.h"

media_player_handle_t media_player_open(player_cfg_t *cfg)
{
    if (cfg == NULL || cfg->play_mask == 0 || (
        cfg->audio_render == NULL && cfg->video_render == NULL)) {
        return NULL;
    }
    player_t *player = (player_t *) media_lib_calloc(1, sizeof(player_t));
    if (player) {
        player->audio_render = cfg->audio_render;
        player->video_render = cfg->video_render;
        player->play_mask = cfg->play_mask;
        player->no_accurate_seek = cfg->no_accurate_seek;
        media_lib_mutex_create(&player->play_mutex);
        player->event_q = msg_q_create(PLAYER_EVENT_Q_ITEMS, sizeof(player_msg_t));
        media_lib_sema_create(&player->play_sema);
        media_lib_thread_create_from_scheduler(&player->play_thread, "Play", player_thread, player);
    }
    return (media_player_handle_t) player;
}

int media_player_set_source(media_player_handle_t p, media_src_type_t src_type, char *uri)
{
    // here need in sync
    player_t *player = (player_t *) p;
    int ret = 0;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    do {
        player->src_type = src_type;
        if (player->url) {
            media_lib_free(player->url);
        }
        player->url = media_lib_strdup(uri);
        printf("set new url to be player:%p %p:%s\n", p, player->url, player->url);
    } while (0);
    media_lib_mutex_unlock(player->play_mutex);
    return ret;
}

int media_player_set_source_by_callback(media_player_handle_t p, player_read_func read, player_seek_func seek, void *ctx){
    player_t *player = (player_t *) p;
    int ret = 0;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    player->src_by_callback = true;
    player->src_read_cb = read;
    player->src_seek_cb = seek;
    player->src_cb_ctx = ctx;
    media_lib_mutex_unlock(player->play_mutex);
    return ret;
}

int media_player_set_source_by_fifo(media_player_handle_t p, uint8_t* data, int size)
{
    player_t *player = (player_t *) p;
    int ret = 0;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    player->src_by_fifo = true;
    player->src_fifo = data;
    player->src_fifo_size = size;
    media_lib_mutex_unlock(player->play_mutex);
    return ret;
}

int media_player_set_callback(media_player_handle_t p, player_callback_func cb, void *ctx)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    player->cb = cb;
    player->ctx = ctx;
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}

int media_player_get_stream_number(media_player_handle_t p, extractor_stream_type_t stream_type)
{
    player_t *player = (player_t *) p;
    uint16_t count = 0;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    if (player->play_state >= PLAYER_STATE_PREPARED) {
        if (player->extractor) {
            esp_extractor_get_stream_num(player->extractor, stream_type, &count);
        }
    }
    media_lib_mutex_unlock(player->play_mutex);
    return count;
}

int media_player_get_stream_info(media_player_handle_t p, extractor_stream_type_t stream_type, int stream_idx, extractor_stream_info_t *stream_info)
{
    int ret = -1;
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    if (player->play_state >= PLAYER_STATE_PREPARED && player->extractor) {
        ret = esp_extractor_get_stream_info(player->extractor, stream_type, stream_idx, stream_info);
    }
    media_lib_mutex_unlock(player->play_mutex);
    return ret;
}

int media_player_play(media_player_handle_t p)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    player_msg_t msg;
    msg.player_event = PLAYER_INTERNAL_MSG_PLAY;
    player_send_msg(player, &msg);
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}

int media_player_set_speed(media_player_handle_t p, float speed)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    player->set_speed = speed;
    player_msg_t msg;
    msg.player_event = PLAYER_INTERNAL_MSG_SET_SPEED;
    player_send_msg(player, &msg);
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}

int media_player_get_position(media_player_handle_t p, uint32_t *pts)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    *pts = _player_position(player);
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}

static uint32_t _player_duration(player_t *player)
{
    uint32_t dur = 0;
    if (player->extractor) {
        extractor_stream_info_t stream_info;
        if (player->video_format) {
            stream_info.duration = 0;
            esp_extractor_get_stream_info(player->extractor, EXTRACTOR_STREAM_TYPE_VIDEO, 0, &stream_info);
            if (stream_info.duration > dur) {
                dur = stream_info.duration;
            }
        }
        if (player->audio_format) {
            stream_info.duration = 0;
            esp_extractor_get_stream_info(player->extractor, EXTRACTOR_STREAM_TYPE_AUDIO, 0, &stream_info);
            if (stream_info.duration > dur) {
                dur = stream_info.duration;
            }
        }
    }
    return dur;
}

int media_player_get_duration(media_player_handle_t p, uint32_t *duration)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    *duration = _player_duration(player);
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}

int media_player_seek_time(media_player_handle_t p, uint32_t ms)
{
    int ret;
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    // Not allow set speed when seeking, add not allow set speed condition
    if (player->play_state != PLAYER_STATE_PLAYED && player->speed != PLAYER_SPEED_1X &&
        player->speed != PLAYER_SPEED_PAUSE) {
        // during seek not allow seeing
        media_lib_mutex_unlock(player->play_mutex);
        return PLAYER_ERR_NOT_ALLOW;
    }
    player->seeking_time = ms;
    if (player->extractor) {
        // Cancel ongoing read
        esp_extractor_read_abort(player->extractor);
    }
    media_lib_mutex_unlock(player->play_mutex);
    player_msg_t msg;
    msg.player_event = PLAYER_INTERNAL_MSG_SEEK;
    ret = player_send_msg(player, &msg);
    flush_print("Seek ended\n");
    return ret;
}

int media_player_seek_position(media_player_handle_t player, uint64_t position)
{
    return 0;
}

int media_player_set_stream_info(media_player_handle_t player, extractor_stream_info_t *stream_array, int stream_num)
{
    return 0;
}

int media_player_push_stream_data(media_player_handle_t player, extractor_stream_type_t stream, extractor_frame_info_t *frame)
{
    return 0;
}

int media_player_stop(media_player_handle_t p)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    if (player->extractor) {
        esp_extractor_read_abort(player->extractor);
    }
    player_msg_t msg;
    msg.player_event = PLAYER_INTERNAL_MSG_STOP;
    media_lib_mutex_unlock(player->play_mutex);
    int ret = player_send_play_msg_sync(player, &msg);
    return ret;
}

int media_player_set_loop(media_player_handle_t p, bool loop)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    player->looping = loop;
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}

int media_player_close(media_player_handle_t p)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    // For network case close socket to fast quit
    flush_print("Start to do closing\n");
    player_set_state(player, PLAYER_STATE_CLOSING);
    // Abort ongoing read
    if (player->extractor) {
        esp_extractor_read_abort(player->extractor);
    }
    media_lib_mutex_unlock(player->play_mutex);
    player_msg_t msg;
    msg.player_event = PLAYER_INTERNAL_MSG_CLOSE;
    player_send_play_msg_sync(player, &msg);

    media_lib_mutex_destroy(player->play_mutex);
    media_lib_sema_destroy(player->play_sema);
    printf("Close player %p\n", p);
    if (player->url) {
        media_lib_free(player->url);
        flush_print("Free %p\n", player->url);
    }
    media_lib_free(player);
    flush_print("Close finished\n");
    return 0;
}

int media_player_interrupt(media_player_handle_t p)
{
    player_t *player = (player_t *) p;
    int ret = 0;
    do {
        media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
        if (player->play_state < PLAYER_STATE_PREPARED || player->extractor == NULL) {
            ret = PLAYER_ERR_NOT_ALLOW;
            break;
        }
        // Get resume info and then do stop action
        player->resume_info = (esp_extractor_resume_info_t *) malloc(sizeof(esp_extractor_resume_info_t));
        if (player->resume_info == NULL) {
            ret = PLAYER_ERR_NO_MEM;
            break;
        }
        ret = esp_extractor_get_resume_info(player->extractor, player->resume_info);
        if (ret != 0) {
            ESP_LOGE(TAG, "fail to get resume info ret %d", ret);
            break;
        }
        // Use current play position instead of extractor send pts
        player->resume_info->time = _player_position(player);
        ESP_LOGI(TAG, "Resume position %d", (int)player->resume_info->time);
        if (player->extractor) {
            esp_extractor_read_abort(player->extractor);
        }
        media_lib_mutex_unlock(player->play_mutex);
        player_msg_t msg;
        msg.player_event = PLAYER_INTERNAL_MSG_INTERRUPT;
        ret = player_send_play_msg_sync(player, &msg);
    } while (0);
    media_lib_mutex_unlock(player->play_mutex);
    return ret;
}

int media_player_set_fifo_size(media_player_handle_t p, player_fifo_cfg_t *fifo_cfg)
{
    player_t *player = (player_t *) p;
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    player->fifo_cfg = *fifo_cfg;
    player->fifo_by_user = true;
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}

int media_player_recover(media_player_handle_t p)
{
    player_t *player = (player_t *) p;
    if (player->resume_info == NULL) {
        return PLAYER_ERR_NOT_ALLOW;
    }
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    if (player->play_thread == NULL) {
        media_lib_thread_create_from_scheduler(&player->play_thread, "Play", player_thread, player);
    }
    media_lib_mutex_unlock(player->play_mutex);
    return media_player_play(p);
}

int media_player_dump(media_player_handle_t p, uint8_t mask)
{
    av_render_dump(NULL, mask);
    return 0;
}

#include <inttypes.h>

int media_player_query(media_player_handle_t p)
{
    player_t *player = (player_t *) p;
    if (player == NULL) {
        return PLAYER_ERR_NOT_ALLOW;
    }
    media_lib_mutex_lock(player->play_mutex, MEDIA_LIB_MAX_LOCK_TIME);
    ESP_LOGI(TAG, "Position:     %" PRIu32 "/%" PRIu32, _player_position(player), _player_duration(player));
    ESP_LOGI(TAG, "Audio format: %s", _player_get_audio_format_str(player->audio_format));
    ESP_LOGI(TAG, "Video format: %s", _player_get_video_format_str(player->video_format));
    ESP_LOGI(TAG, "Speed: %f Status:%d", player->speed, player->play_state);
    if (player->render_handle) {
        av_render_query(player->render_handle);
    }
    media_lib_mutex_unlock(player->play_mutex);
    return 0;
}
