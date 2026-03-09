/**
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * See LICENSE file for details.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_extractor_types.h"
#include "av_render_types.h"
#include "media_src.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PLAY_MASK_AUDIO       (1)
#define PLAY_MASK_VIDEO       (2)
#define PLAY_MASK_SUBTITLE    (4)

#define PLAYER_ERR_INVAID_ARG (-1)
#define PLAYER_ERR_NOT_ALLOW  (-2)
#define PLAYER_ERR_NO_MEM     (-3)

/**
 * @brief        Player configuration
 */
typedef struct {
    uint32_t              play_mask;        /*!< Play mask (to play audio or video or both) */
    audio_render_handle_t audio_render;     /*!< Audio render handle */
    video_render_handle_t video_render;     /*!< Video render handle */
    bool                  no_accurate_seek; /*!< When enable seek to nearest key frame instead */
} player_cfg_t;

/**
 * @brief        Player speed
 */
#define PLAYER_SPEED_PAUSE (0.0)
#define PLAYER_SPEED_1X    (1.0)

/**
 * @brief        Player event
 */
typedef enum {
    PLAYER_EVENT_NONE,               /*!< None event happen */
    PLAYER_EVENT_SRC_CONNECTING,     /*!< Data source connecting */
    PLAYER_EVENT_SRC_CONNECTED,      /*!< Data source connected */
    PLAYER_EVENT_SRC_DISCONNECTED,   /*!< Data source disconnected */
    PLAYER_EVENT_PREPARED,           /*!< Success to parse input data source */
    PLAYER_EVENT_PREPARE_ERROR,      /*!< Fail to parse input data source */
    PLAYER_EVENT_PLAYED_DONE,        /*!< Success to start play (response to `media_player_play`) */
    PLAYER_EVENT_PLAYED_PAUSE,       /*!< Success to pause play */
    PLAYER_EVENT_PLAYED_RESUME,      /*!< Success to resume play */
    PLAYER_EVENT_PLAY_ERROR,         /*!< Fail to play */
    PLAYER_EVENT_SEEK_DONE,          /*!< Seek finished and success (response to `media_player_seek_*`)*/
    PLAYER_EVENT_SEEK_ERROR,         /*!< Error to do seek */
    PLAYER_EVENT_EOS,                /*!< All streams are played to end */
    PLAYER_EVENT_AUDIO_EOS,          /*!< Audio played to end */
    PLAYER_EVENT_VIDEO_EOS,          /*!< Video played to end */
    PLAYER_EVENT_VIDEO_NOT_SUPPORT,  /*!< Video not supported, if have audio audio will continue to play */
    PLAYER_EVENT_AUDIO_NOT_SUPPORT,  /*!< Audio not supported, if have video video will continue to play */
    PLAYER_EVENT_STOPPED,            /*!< Player being stopped */
} player_event_t;

/**
 * @brief        Player fifo setting
 * @note         When fifo size is set, it will render or decoder in async
 *               If set to 0, it will render or decode in sync without extra middle fifo
 *               Set fifo can do things async, improve speed while waste thread and memory resource
 */
typedef struct {
    uint32_t adec_fifo_size;      /*!< Audio decoder input fifo size (if set will auto create audio decoder thread) */
    uint32_t vdec_fifo_size;      /*!< Video decoder input fifo size (if set will auto create video decoder thread) */
    uint32_t arender_fifo_size;   /*!< Audio render input fifo size (if set will auto create audio render thread) */
    uint32_t vrender_fifo_size;   /*!< Video render input fifo size (if set will auto create video render thread) */
    uint32_t extractor_pool_size; /*!< Pool size to hold extractor output frames */
} player_fifo_cfg_t;

/**
 * @brief        Player handle
 */
typedef void *media_player_handle_t;

/**
 * @brief        Player event callback
 */
typedef int (*player_callback_func)(player_event_t event, void *ctx);

/**
 * @brief        Player read callback
 */
typedef int (*player_read_func)(void *data, uint32_t size, void *ctx);

/**
 * @brief        Player seek callback
 */
typedef int (*player_seek_func)(uint32_t position, void *ctx);

/**
 * @brief        Open player
 *
 * @param        cfg: Player configuration
 *
 * @return
 *               - NULL: No enough memory
 *               - Others: Player instance
 */
media_player_handle_t media_player_open(player_cfg_t *cfg);

/**
 * @brief        Set player callback
 *
 * @param        player: Player instance
 * @param        cb: Callback to receive player event
 * @param        ctx: Callback context
 *
 * @return
 *               - 0: Success
 *               - Others: Invalid argument
 */
int media_player_set_callback(media_player_handle_t player, player_callback_func cb, void *ctx);

/**
 * @brief        Set customized fifo
 * @note         Need configuration before play, if not called, will allocated related memory internally
 *
 * @param        player: Player instance
 * @param        fifo_cfg: Fifo configuration
 *
 * @return
 *               - 0: Success
 *               - Others: Invalid argument
 */
int media_player_set_fifo_size(media_player_handle_t player, player_fifo_cfg_t *fifo_cfg);

/**
 * @brief        Set loop play
 *
 * @note         Player will auto replay when loop play is set
 *
 * @param        player: Player instance
 * @param        loop: Whether loop play after play to end
 *
 * @return
 *               - 0: On success
 *               - Others: Invalid argument
 */
int media_player_set_loop(media_player_handle_t player, bool loop);

/**
 * @brief        Set current data source
 *
 * @param        player: Player instance
 * @param        src_type: Fifo configuration
 *
 * @return
 *               - 0: Success
 *               - Others: Invalid argument
 */
int media_player_set_source(media_player_handle_t player, media_src_type_t src_type, char *uri);

/**
 * @brief        Set current data source
 *
 * @note         Not support seek backward
 *
 * @param        player: Player instance
 * @param        read:   Read callback to get source data
 * @param        seek:   Seek callback to real position
 * @param        ctx:    Context to read data
 *
 * @return
 *               - 0: Success
 *               - Others: Invalid argument
 */
int media_player_set_source_by_callback(media_player_handle_t player, player_read_func read, player_seek_func seek, void *ctx);

/**
 * @brief        Set current data source (Fifo source)
 *
 * @note         Fifo should be always valid during whole play lifetime
 *
 * @param        player: Player instance
 * @param        data:   Source data
 * @param        size:   Source data size
 *
 * @return
 *               - 0: Success
 *               - Others: Invalid argument
 */
int media_player_set_source_by_fifo(media_player_handle_t player, uint8_t* data, int size);

/**
 * @brief        Get stream number by stream type
 *
 * @note         Need called when after received `PLAYER_EVENT_PREPARED`
 *
 * @param        player: Player instance
 * @param        stream_type: Stream type
 *
 * @return
 *               - 0: No such stream
 *               - Others: Actual stream number
 */
int media_player_get_stream_number(media_player_handle_t player, extractor_stream_type_t stream_type);

/**
 * @brief        Get stream information for certain stream type and stream index
 *
 * @note         Need called when after received `PLAYER_EVENT_PREPARED`
 *
 * @param        player: Player instance
 * @param        stream_type: Stream type
 * @param        stream_idx: Stream index
 * @param[out]   stream_info: Stream information to be fetched
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to get stream information
 */
int media_player_get_stream_info(media_player_handle_t player, extractor_stream_type_t stream_type, int stream_idx,
                           extractor_stream_info_t *stream_info);

/**
 * @brief        Start to play
 *
 * @note         Play can be play before `PLAYER_EVENT_PREPARED` received and it will auto play after message reached
 *               Or manually called it after `PLAYER_EVENT_PREPARED`
 *               Play also can happen after call `media_player_stop`
 *
 * @param        player: Player instance
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to play
 */
int media_player_play(media_player_handle_t player);

/**
 * @brief        Set play speed
 *
 * @note         Speed 0.0 -- pause
 *                     1.0 -- resume
 *                     0.5 -- 0.5x
 *                     2.0 -- 2x
 *
 * @param        player: Player instance
 * @param        speed: Play speed setting
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to set speed
 */
int media_player_set_speed(media_player_handle_t player, float speed);

/**
 * @brief        Get current play time position
 *
 * @param        player: Player instance
 * @param        pts: Current playback time position (unit milliseconds)
 *
 * @return
 *               - 0: On success
 *               - Others: Fail get current play position
 */
int media_player_get_position(media_player_handle_t player, uint32_t *pts);


/**
 * @brief        Get current play time position
 *
 * @note         For VBR file, can not get duration from file header
 *               Duration is estimated and may change during play
 *
 * @param        player: Player instance
 * @param        pts: Current playback time position (unit milliseconds)
 *
 * @return
 *               - 0: On success
 *               - Others: Fail get file duration
 */
int media_player_get_duration(media_player_handle_t player, uint32_t *pts);

/**
 * @brief        Seek to certain time
 *
 * @param        player: Player instance
 * @param        time: Time position to be seek to (unit milliseconds)
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to do time seek
 */
int media_player_seek_time(media_player_handle_t player, uint32_t time);

/**
 * @brief        Seek to certain byte position
 *
 * @note         TODO: Not implemented yet
 * @param        player: Player instance
 * @param        position: Byte position to be seek to
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to do byte seek
 */
int media_player_seek_position(media_player_handle_t player, uint64_t position);

/**
 * @brief        Set stream information (Push mode only)
 *
 * @note         TODO: Not implemented yet
 * @param        player: Player instance
 * @param        stream_array: Stream information array
 * @param        stream_num: Number of streams
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to do set stream information
 */
int media_player_set_stream_info(media_player_handle_t player, extractor_stream_info_t *stream_array, int stream_num);

/**
 * @brief        Push stream data (Push mode only)
 *
 * @note         TODO: Not implemented yet
 * @param        player: Player instance
 * @param        stream: Stream type
 * @param        frame: Stream frame to be pushed
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to do push stream data
 */
int media_player_push_stream_data(media_player_handle_t player, extractor_stream_type_t stream, extractor_frame_info_t *frame);

/**
 * @brief        Stop player
 *
 * @note         Can be replayed by call `media_player_play`
 *
 * @param        player: Player instance
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to stop player
 */
int media_player_stop(media_player_handle_t player);

/**
 * @brief        Interrupt play (paired with `media_player_recover`)
 *
 * @note         Interrupt play keep only necessary information for play and release other resources
 *
 * @param        player: Player instance
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to interrupt
 */
int media_player_interrupt(media_player_handle_t player);

/**
 * @brief        Recover play from interrupted point (paired with `media_player_interrupt`)
 *
 * @param        player: Player instance
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to recover from interrupt point
 */
int media_player_recover(media_player_handle_t player);

/**
 * @brief        Query play status (For debug only)
 *
 * @param        player: Player instance
 *
 * @return
 *               - 0: On success
 *               - Others: Invalid player instance
 */
int media_player_query(media_player_handle_t player);

/**
 * @brief        Dump data from play pipeline (For debug only)
 *
 * @param        player: Player instance
 *
 * @return
 *               - 0: On success
 *               - Others: Invalid player instance
 */
int media_player_dump(media_player_handle_t player, uint8_t mask);

/**
 * @brief        Close player
 *
 * @note         After closed, player can not be operated anymore
 *
 * @param        player: Player instance
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to close player
 */
int media_player_close(media_player_handle_t player);

#ifdef __cplusplus
}
#endif
