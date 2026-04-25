/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdio.h>
#include <string.h>
#include "media_lib_adapter.h"
#include "media_lib_os.h"
#include "esp_log.h"
#include "codec_init.h"
#include "codec_board.h"
#include "player.h"
#include "esp_extractor_reg.h"
#include "esp_audio_dec_default.h"
#include "esp_video_dec_default.h"
#include "av_render_default.h"
#include "media_src_storage.h"
#include "media_src_network.h"
#include "settings.h"

static const char *TAG = "Player_Test";

#define PLAYER_EVENT_MASK_PREPARED (1)
#define PLAYER_EVENT_MASK_PLAYED   (2)
#define PLAYER_EVENT_MASK_EOS      (4)
#define PLAYER_EVENT_MASK_STOPPED  (8)
#define PLAYER_EVENT_MASK_SEEKED   (0x10)
#define PLAYER_EVENT_MASK_ERR      (0x20)

static uint32_t play_event_record;

static uint32_t loop_limits = 5;

static void unregister_components()
{
    esp_extractor_unregister_all();
    esp_audio_dec_unregister_default();
    esp_video_dec_unregister_default();
}

static void register_components()
{
    // Register media source
    media_src_register_storage();
    media_src_register_network();
    // Register extractor
    esp_extractor_register_all();
    // Register audio decoder
    esp_audio_dec_register_default();
    // Register implement for av_render
    esp_video_dec_register_default();
}

static void thread_manager(const char *thread_name, media_lib_thread_cfg_t *thread_cfg)
{
    if (strcmp(thread_name, "Vdec") == 0) {
        thread_cfg->core_id = 1;
        thread_cfg->priority = 15;
        thread_cfg->stack_size = 10 * 1024;
    }
    if (strcmp(thread_name, "ARender") == 0) {
        thread_cfg->core_id = 1;
        thread_cfg->priority = 21;
        thread_cfg->stack_size = 10 * 1024;
    }
    if (strcmp(thread_name, "Adec") == 0) {
        thread_cfg->core_id = 0;
        thread_cfg->priority = 10;
        thread_cfg->stack_size = 15 * 1024;
    }
}

static int player_event_cb(player_event_t event, void *ctx)
{
    ESP_LOGI(TAG, "Got player event:%d\n", event);
    switch (event) {
        case PLAYER_EVENT_PREPARED:
            play_event_record |= PLAYER_EVENT_MASK_PREPARED;
            break;
        case PLAYER_EVENT_PLAYED_DONE:
            play_event_record |= PLAYER_EVENT_MASK_PLAYED;
            break;
        // Workaround use audio EOS as play end
        case PLAYER_EVENT_AUDIO_EOS:
            play_event_record |= PLAYER_EVENT_MASK_EOS;
            break;
        case PLAYER_EVENT_STOPPED:
            play_event_record |= PLAYER_EVENT_MASK_STOPPED;
            play_event_record &= ~PLAYER_EVENT_MASK_PLAYED;
            break;
        case PLAYER_EVENT_SEEK_DONE:
            play_event_record |= PLAYER_EVENT_MASK_SEEKED;
            break;
        case PLAYER_EVENT_PREPARE_ERROR:
        case PLAYER_EVENT_PLAY_ERROR:
            play_event_record |= PLAYER_EVENT_MASK_ERR;
            break;
        default:
            break;
    }
    return 0;
}

static int play_file(char* file, uint32_t play_mask, bool loop)
{
    int ret = 0;
    player_cfg_t t_cfg = {
        .play_mask = play_mask,
    };
    audio_render_handle_t audio_render = NULL;
    audio_render_handle_t video_render = NULL;
    if (play_mask & PLAY_MASK_AUDIO) {
        if (audio_render == NULL) {
            i2s_render_cfg_t i2s_cfg = {
                .play_handle = get_playback_handle(),
            };
            audio_render = av_render_alloc_i2s_render(&i2s_cfg);
            if (audio_render == NULL) {
                ESP_LOGE(TAG, "Fail to create i2s render");
            }
        }
        t_cfg.audio_render = audio_render;
    }
    if (play_mask & PLAY_MASK_VIDEO) {
        lcd_render_cfg_t lcd_cfg = {
#if CONFIG_IDF_TARGET_ESP32P4
            .dsi_panel = true,
#endif
            .lcd_handle = board_get_lcd_handle(),
        };
        if (video_render == NULL) {
            video_render = av_render_alloc_lcd_render(&lcd_cfg);
            if (video_render == NULL) {
                ESP_LOGE(TAG, "Fail to create lcd render");
            }
        }
        t_cfg.video_render = video_render;
    }
    // Clear play event record
    play_event_record = 0;
    // Create player
    media_player_handle_t player = media_player_open(&t_cfg);
    if (player == NULL) {
        ESP_LOGE(TAG, "Fail to create player");
        return -1;
    }
    media_player_set_callback(player, player_event_cb, NULL);
    media_player_set_loop(player, loop);
    ret = media_player_set_source(player, MEDIA_SRC_TYPE_STORAGE, file);
    if (ret != 0) {
        ESP_LOGE(TAG, "Fail to set data source %s", file);
        goto _clear_up;
    }
    media_player_play(player);
    int replay_count = 0;
    // Simple logic to wait for player event
    while (1) {
        if (play_event_record & PLAYER_EVENT_MASK_ERR) {
            ESP_LOGE(TAG, "Fail to play file for evnet %x", (int)play_event_record);
            break;
        }
        if (play_event_record & PLAYER_EVENT_MASK_EOS) {
            if (loop == false) {
                ESP_LOGE(TAG, "Success to play to file end");
                break;
            }
            // Clear EOS flag
            play_event_record &= ~PLAYER_EVENT_MASK_EOS;
            replay_count++;
            ESP_LOGI(TAG, "Replay count %d", replay_count);
            if (replay_count >= loop_limits) {
                break;
            }
        }
        media_lib_thread_sleep(500);
    }
 _clear_up:
    // Clear up player and render resource
    if (player) {
        media_player_close(player);
    }
    if (audio_render) {
        audio_render_free_handle(audio_render);
    }
    if (video_render) {
        video_render_free_handle(audio_render);
    }
    return ret;
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Add OS glue layer
    media_lib_add_default_adapter();
    // Do board initialization
    set_codec_board_type(TEST_BOARD);
    mount_sdcard();

    codec_init_cfg_t init_cfg = {
        .reuse_dev = true,
    };
    init_codec(&init_cfg);
    board_lcd_init();

    // Set thread manager to control thread priority
    media_lib_thread_set_schedule_cb(thread_manager);
    // Register components
    register_components();
    //  Play both video and audio
    uint32_t play_mask = PLAY_MASK_AUDIO | PLAY_MASK_VIDEO;
    // Play test file to file end
    play_file(TEST_VIDEO_FILE, play_mask, false);
    // Loop to play test file loop times set to 5
    loop_limits = 5;
    play_file(TEST_VIDEO_FILE, play_mask, true);
    // Unregister components
    unregister_components();
}
