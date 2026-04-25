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
#include "settings.h"

static const char *TAG = "PLAYER_TEST";

int player_cli_run(char *file_name);

static int init_wifi(void)
{
    return 0;
}

static void thread_manager(const char *thread_name, media_lib_thread_cfg_t *thread_cfg)
{
    if (strcmp(thread_name, "Vdec") == 0) {
        thread_cfg->core_id = 1;
        thread_cfg->priority = 15;
        thread_cfg->stack_size = 10 * 1024;
        return;
    }
    if (strcmp(thread_name, "ARender") == 0) {
        thread_cfg->core_id = 0;
        thread_cfg->priority = 21;
        thread_cfg->stack_size = 10 * 1024;
        return;
    }
    if (strcmp(thread_name, "VRender") == 0) {
        thread_cfg->core_id = 0;
        thread_cfg->priority = 10;
        thread_cfg->stack_size = 10 * 1024;
        return;
    }
    if (strcmp(thread_name, "Play") == 0) {
        thread_cfg->core_id = 0;
        thread_cfg->priority = 20;
        thread_cfg->stack_size = 20 * 1024;
        printf("Player use stacksize %d\n", (int)thread_cfg->stack_size);
        return;
    }
    if (strcmp(thread_name, "Adec") == 0) {
        thread_cfg->core_id = 0;
        thread_cfg->priority = 10;
        thread_cfg->stack_size = 15 * 1024;
        return;
    }
}

#define RUN_ASYNC(thread, core, body)  \
void async_task##thread(void* arg) {   \
    body;                              \
    media_lib_thread_destroy(NULL);    \
}                                      \
media_lib_thread_create(NULL, #thread, async_task##thread, NULL, 4096, 3, core);


void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    media_lib_add_default_adapter();
    set_codec_board_type(TEST_BOARD);

    codec_init_cfg_t init_cfg = {
        .reuse_dev = true,
    };
    init_codec(&init_cfg);
    board_lcd_init();
    mount_sdcard();
    init_wifi();

    media_lib_thread_set_schedule_cb(thread_manager);
    player_cli_run(NULL);
}



