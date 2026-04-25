#ifndef PLAYER_INTERNAL_H
#define PLAYER_INTERNAL_H

#include <stdio.h>
#include "player.h"
#include "media_src.h"
#include "esp_extractor.h"
#include "msg_q.h"
#include "media_lib_os.h"
#include "mem_pool.h"
#include "esp_log.h"
#include "av_render.h"

#define TAG "Player"

#define SAFE_FREE(a) \
    if (a) {         \
        free(a);     \
        a = NULL;    \
    }

#define AV_SYNC_TOLERANCE (3000)
#define PLAYER_EVENT_Q_ITEMS       (10)

#define REUSE_EXTRACTOR_DATA
// FLAC sometime framesize more than 10K

#ifndef REUSE_EXTRACTOR_DATA
#define VIDEO_FIFO_SIZE            (2*1024*1024)
#define AUDIO_FIFO_SIZE            (200 * 1024)
#define EXTRACTOR_POOL_SIZE        (300*1024)
#else
#define VIDEO_FIFO_SIZE            (2* 1024)
#define AUDIO_FIFO_SIZE            (8* 1024)
#define EXTRACTOR_POOL_SIZE        (2*1024*1024)
#endif

#define VIDEO_RENDER_FIFO_SIZE     (800 * 480 * 2 * 2)
#define AUDIO_RENDER_FIFO_SIZE     (20 * 1024)
#define AUDIO_RENDER_SAMPLES       (4096*2)
#define VIDEO_RENDER_FIFO_FPS      (2)

typedef enum {
    PLAYER_STATE_NONE,
    PLAYER_STATE_PREPARING = 1,
    PLAYER_STATE_PREPARED,
    PLAYER_STATE_PLAYING,
    PLAYER_STATE_PLAYED,
    PLAYER_STATE_PLAY_ERROR,
    PLAYER_STATE_PAUSED,
    PLAYER_STATE_SEEKING,
    PLAYER_STATE_STOPPING,
    PLAYER_STATE_STOPPED,
    PLAYER_STATE_CLOSING,
} player_state_t;

typedef enum {
    PLAYER_INTERNAL_MSG_CLOSE,
    PLAYER_INTERNAL_MSG_PREPARE,
    PLAYER_INTERNAL_MSG_PREPARED,
    PLAYER_INTERNAL_MSG_PLAY,
    PLAYER_INTERNAL_MSG_STOP,

    PLAYER_INTERNAL_MSG_SET_SPEED = 5,
    PLAYER_INTERNAL_MSG_SEEK,

    PLAYER_INTERNAL_MSG_AUDIO_REACHED,
    PLAYER_INTERNAL_MSG_VIDEO_REACHED,

    PLAYER_INTERNAL_MSG_AUDIO_DECODER_ERR,
    PLAYER_INTERNAL_MSG_VIDEO_DECODER_ERR,

    PLAYER_INTERNAL_MSG_SRC_PLAY = 0x10,
    PLAYER_INTERNAL_MSG_SRC_PAUSE,
    PLAYER_INTERNAL_MSG_SRC_CLOSE,
    PLAYER_INTERNAL_MSG_INTERRUPT,
} player_internal_msg_type_t;

typedef struct {
    // Configuration
    uint32_t                     play_mask;
    player_fifo_cfg_t            fifo_cfg;
    bool                         fifo_by_user;
    bool                         no_accurate_seek;
    audio_render_handle_t        audio_render;
    video_render_handle_t        video_render;
    bool                         auto_start;
    player_callback_func         cb;
    void                        *ctx;
    // Resource
    media_src_type_t             src_type;
    bool                         src_by_callback;
    bool                         src_by_fifo;
    uint8_t*                     src_fifo;
    int                          src_fifo_size;
    esp_extractor_config_t*      extractor_cfg;
    player_read_func             src_read_cb;
    player_seek_func             src_seek_cb;
    void                        *src_cb_ctx;

    esp_extractor_handle_t       extractor;
    msg_q_handle_t               event_q;
    av_render_handle_t           render_handle;

    // Thread and protect src
    media_lib_thread_handle_t    play_thread;
    media_lib_mutex_handle_t     play_mutex;
    media_lib_sema_handle_t      play_sema;

    // Status
    extractor_audio_format_t     audio_format;
    extractor_video_format_t     video_format;
    bool                         audio_rendered;
    bool                         video_rendered;
    float                        set_speed;
    bool                         audio_slow_down;
    uint8_t                      audio_enough_count;
    uint32_t                     seeking_time;
    uint32_t                     audio_send_pts;
    uint32_t                     video_send_pts;
    player_state_t               play_state;
    float                        speed;

    bool                         eos_sent;
    bool                         audio_eos_recv;
    bool                         video_eos_recv;
    uint32_t                     stream_playing;
    char                        *url;
    esp_extractor_resume_info_t *resume_info;
    bool                         underflowing;
    bool                         audio_locking;
    bool                         video_locking;
    bool                         write_pending;
    extractor_frame_info_t       pending_frame;
    bool                         looping;
    // PCM PTS fix: AVI extractor reports chunk index, not ms
    uint64_t                     pcm_bytes_sent;
    uint32_t                     pcm_bytes_per_ms; // sample_rate * channels * (bits/8) / 1000
} player_t;

typedef struct {
    player_internal_msg_type_t player_event;
    bool                       sync_event;
    int                       *ret_value;
} player_msg_t;

#define flush_print(...)     \
{                            \
    printf(__VA_ARGS__);     \
    fflush(stdout);          \
}

const char *_player_get_video_format_str(extractor_video_format_t format);

const char *_player_get_audio_format_str(extractor_audio_format_t format);

int player_send_msg(player_t *player, player_msg_t *msg);

int player_send_play_msg_sync(player_t *player, player_msg_t *msg);

uint32_t _player_position(player_t *player);

void player_set_state(player_t *player, player_state_t state);

void player_thread(void *ctx);

#endif
