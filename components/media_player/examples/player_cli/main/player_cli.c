/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include "media_src.h"
#include "media_src_storage.h"
#include "media_src_network.h"
#include "player.h"
#include "esp_console.h"
#include "media_lib_os.h"
#include "folder_reader.h"
#include "media_lib_mem_trace.h"
#include "esp_extractor_reg.h"
#include "esp_mp4_extractor.h"
#include "av_render_default.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_audio_dec_default.h"
#include "esp_video_dec_default.h"
#include "esp_codec_dev.h"
#include "codec_init.h"
#include "sys_state.h"

#define TAG               "TEST"

#define MAX_PLAYER_NUM    (2)

#define SAME_STRING(a, b) (strcmp(a, b) == 0)

#define VERIFY_SEL(sel)                                  \
    if (sel >= MAX_PLAYER_NUM || players[sel] == NULL) { \
        printf("Player already closed\n");               \
        return 0;                                        \
    }

#define RETURN_ON_FAIL(ret)                                   \
    if (ret != 0) {                                           \
        ESP_LOGE(TAG, "Fail on %s:%d\n", __func__, __LINE__); \
        return ret;                                           \
    }

#define PLAYER_EVENT_MASK_PREPARED (1)
#define PLAYER_EVENT_MASK_PLAYED   (2)
#define PLAYER_EVENT_MASK_EOS      (4)
#define PLAYER_EVENT_MASK_STOPPED  (8)
#define PLAYER_EVENT_MASK_SEEKED   (0x10)
#define PLAYER_EVENT_MASK_ERR      (0x20)
#define MAX_FILE_PATH_LEN          (256)

typedef struct {
    char *cmd;
    char *help;
    int (*func)(int argc, char **argv);
} cli_entry_t;

static int cli_quit = 0;
static media_player_handle_t players[MAX_PLAYER_NUM];
static int players_mask[MAX_PLAYER_NUM];
static int filter_file_count = 0;
static int filter_file_idx = 0;
static char *filter_file_pattern;
static char *filter_ip;
// for filter ip
static char *path_url = NULL;
static audio_render_handle_t audio_render;
static video_render_handle_t video_render;

static int parse_cmd(char *buffer);

static int get_cur_time()
{
    return esp_timer_get_time() / 1000;
}

static void lc(char *s)
{
    while (*s) {
        if (*s >= 'A' && *s <= 'Z') {
            *s += 'a' - 'A';
        }
        s++;
    }
}

/*
 * @brief  Component register
 */
static void register_media_src()
{
    media_src_register_storage();
    media_src_register_network();
}

static int register_extractor()
{
    return esp_extractor_register_all();
}

static int unregister_component()
{
    esp_extractor_unregister_all();
    esp_audio_dec_unregister_default();
    return 0;
}

static int register_audio_decoder()
{
    return esp_audio_dec_register_default();
}

static int register_video_decoder()
{
    return esp_video_dec_register_default();
}


static int player_event_cb(player_event_t event, void *ctx)
{
    int sel = (int) (ctx);
    ESP_LOGI(TAG, "Got player event:%d\n", event);
    switch (event) {
        case PLAYER_EVENT_PREPARED:
            players_mask[sel] |= PLAYER_EVENT_MASK_PREPARED;
            break;
        case PLAYER_EVENT_PLAYED_DONE:
            players_mask[sel] |= PLAYER_EVENT_MASK_PLAYED;
            break;
        case PLAYER_EVENT_AUDIO_EOS:
        case PLAYER_EVENT_VIDEO_EOS:
            players_mask[sel] |= PLAYER_EVENT_MASK_EOS;
            break;
        case PLAYER_EVENT_STOPPED:
            players_mask[sel] |= PLAYER_EVENT_MASK_STOPPED;
            players_mask[sel] &= ~PLAYER_EVENT_MASK_PLAYED;
            break;
        case PLAYER_EVENT_SEEK_DONE:
            players_mask[sel] |= PLAYER_EVENT_MASK_SEEKED;
            break;
        case PLAYER_EVENT_PREPARE_ERROR:
        case PLAYER_EVENT_PLAY_ERROR:
            players_mask[sel] |= PLAYER_EVENT_MASK_ERR;
            break;
        default:
            break;
    }
    return 0;
}

static esp_err_t cli_open(int argc, char **argv)
{
    int sel = 0;
    int play_mask = 1;
    bool no_render = false;
    bool no_accurate_seek = false;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        }
        if (SAME_STRING(argv[i], "-mask")) {
            play_mask = atoi(argv[++i]);
        }
        if (SAME_STRING(argv[i], "-norender")) {
            no_render = (bool)atoi(argv[++i]);
        }
        if (SAME_STRING(argv[i], "-no_accurate_seek")) {
            no_accurate_seek = true;
        }
    }
    if (players[sel]) {
        media_player_close(players[sel]);
    }

    player_cfg_t t_cfg = {
        .play_mask = play_mask,
        .no_accurate_seek = no_accurate_seek,
    };
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
            .lcd_handle = (esp_lcd_panel_handle_t) board_get_lcd_handle(),
#if CONFIG_IDF_TARGET_ESP32P4
            .dsi_panel = true,
#endif
        };
        if (video_render == NULL) {
            video_render = av_render_alloc_lcd_render(&lcd_cfg);
            if (video_render == NULL) {
                ESP_LOGE(TAG, "Fail to create lcd render");
            }
        }
        if (no_render) {
            if (video_render) {
                video_render_free_handle(video_render);
                video_render = NULL;
            }
            lcd_cfg.use_frame_buffer = true;
            video_render = av_render_alloc_lcd_render(&lcd_cfg);
        }
        t_cfg.video_render = video_render;
    }
    media_player_handle_t player = media_player_open(&t_cfg);
    players_mask[sel] = 0;
    if (no_render) {
        player_fifo_cfg_t fifo_cfg = {
            .adec_fifo_size = 1024,
            .vdec_fifo_size = 1024,
            .arender_fifo_size = 20 * 1024,
            .vrender_fifo_size = 0,
            .extractor_pool_size = 2*1024*1024,
        };
        media_player_set_fifo_size(player, &fifo_cfg);
        ESP_LOGI(TAG, "Video render in sync mode");
    }
    media_player_set_callback(player, player_event_cb, (void *) sel);
    players[sel] = player;
    return 0;
}

static esp_err_t cli_close(int argc, char **argv)
{
    int sel = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        }
    }
    VERIFY_SEL(sel);
    media_player_close(players[sel]);
    players_mask[sel] = 0;
    players[sel] = NULL;
    //audio_mem_print(TAG, __LINE__, __func__);
    return 0;
}

static esp_err_t cli_stop(int argc, char **argv)
{
    int sel = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        }
    }
    VERIFY_SEL(sel);
    return (int) media_player_stop(players[sel]);
}

static esp_err_t cli_seek(int argc, char **argv)
{
    int sel = 0;
    int time = 0, i;
    for (i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        } else {
            break;
        }
    }
    if (i < argc) {
        time = atoi(argv[i]);
    }
    VERIFY_SEL(sel);
    int ret = (int) media_player_seek_time(players[sel], time);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_play(int argc, char **argv)
{
    int sel = 0;
    char *url = NULL;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        } else if (SAME_STRING(argv[i], "-uri")) {
            url = argv[++i];
        }
    }
    //audio_mem_print(TAG, __LINE__, __func__);
    if (players[sel] == NULL) {
        cli_open(argc, argv);
    }
    VERIFY_SEL(sel);
    int ret;
    if (url) {
        printf("set url to be %s\n", url);
        if (memcmp(url, "http", 4) == 0) {
            ret = media_player_set_source(players[sel], MEDIA_SRC_TYPE_NETWORK, url);
        } else {
            ret = media_player_set_source(players[sel], MEDIA_SRC_TYPE_STORAGE, url);
        }
        RETURN_ON_FAIL(ret);
    }
    ret = (int) media_player_play(players[sel]);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_pause(int argc, char **argv)
{
    int sel = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        }
    }
    VERIFY_SEL(sel);
    int ret;
    ret = (int) media_player_set_speed(players[sel], PLAYER_SPEED_PAUSE);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_resume(int argc, char **argv)
{
    int sel = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        }
    }
    VERIFY_SEL(sel);
    int ret;
    ret = (int) media_player_set_speed(players[sel], PLAYER_SPEED_1X);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_speed(int argc, char **argv)
{
    int sel = 0;
    int i = 0;
    for (i = 0; i < argc; i++) {
        if (i + 1 < argc && SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        } else {
            break;
        }
    }
    VERIFY_SEL(sel);
    float speed = 1.0;
    if (i < argc) {
        speed = atof(argv[i]);
    }
    int ret;
    ret = (int) media_player_set_speed(players[sel], speed);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_interrupt(int argc, char **argv)
{
    int sel = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        }
    }
    VERIFY_SEL(sel);
    int ret;
    ret = (int) media_player_interrupt(players[sel]);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_restore(int argc, char **argv)
{
    int sel = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        }
    }
    VERIFY_SEL(sel);
    int ret;
    ret = (int) media_player_recover(players[sel]);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_loop(int argc, char **argv)
{
    int sel = 0;
    bool loop = false;
    for (int i = 0; i < argc; i++) {
        if (i+1 < argc) {
            if (SAME_STRING(argv[i], "-sel")) {
                sel = atoi(argv[++i]);
                continue;
            }
        } else {
            loop = (bool) atoi(argv[i]);
        }
    }
    VERIFY_SEL(sel);
    int ret;
    ESP_LOGI(TAG, "Set loop to be %d", loop);
    ret = (int) media_player_set_loop(players[sel], loop);
    RETURN_ON_FAIL(ret);
    return ret;
}

static esp_err_t cli_param(int argc, char **argv)
{
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-dynamic")) {
            esp_mp4_extractor_use_dynamic_parser((bool) atoi(argv[++i]));
        }
    }
    return 0;
}

static int cli_read_src_data(void* data, uint32_t size, void* ctx) {
    FILE* fp = (FILE*)ctx;
    return fread(data, 1, size, fp);
}

static int cli_seek_src_data(uint32_t position, void* ctx) {
    FILE* fp = (FILE*)ctx;
    return fseek(fp, position, SEEK_SET);
}

static esp_err_t cli_playby(int argc, char **argv)
{
    char* uri = NULL;
    bool by_cb = false;
    bool by_buf = false;

    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-cb")) {
            by_cb = true;
        }
        else if (SAME_STRING(argv[i], "-buf")) {
            by_buf = true;
        }
        else if (SAME_STRING(argv[i], "-uri")) {
            uri  = argv[++i];
        }
    }
    if (uri == NULL) {
        ESP_LOGE(TAG, "File url not set use -uri");
        return -1;
    }
    FILE* fp = fopen(uri, "rb");
    uint8_t* data = NULL;
     int ret = 0;
    do {
        if (fp == NULL) {
            ESP_LOGE(TAG, "file %s note exists", uri);
            break;
        }
        cli_open(argc, argv);
        if (players[0] == NULL) {
            ESP_LOGE(TAG, "Fail to open player");
            break;
        }
        if (by_cb) {
            ret = media_player_set_source_by_callback(players[0], cli_read_src_data, cli_seek_src_data, fp);
        } else if (by_buf) {
            int size = 100*1024;
            data = media_lib_malloc(100*1024);
            if (data == NULL) {
                ESP_LOGE(TAG, "No buffer to hold source data");
                break;
            }
            size = fread(data, 1, size, fp);
            if (size < 0) {
                break;
            }
            ret = media_player_set_source_by_fifo(players[0], data, size);
        } else {
            fclose(fp);
            fp = NULL;
            ret = media_player_set_source(players[0], MEDIA_SRC_TYPE_STORAGE, uri);
        }
        ret = (int) media_player_play(players[0]);
        for (int i = 0; i < 20; i++) {
            uint32_t pos = 0;
            media_lib_thread_sleep(1000);
            media_player_get_position(players[0], &pos);
            printf("Pos %d\n", (int)pos);
        }
    } while (0);
    if (players[0]) {
        media_player_close(players[0]);
        players[0] = NULL;
    }
    if (data) {
        media_lib_free(data);
    }
    if (fp) {
        fclose(fp);
    }
    return ret;
}

static esp_err_t cli_wait(int argc, char **argv)
{
    int time = 1000;
    if (argc >= 1) {
        time = atoi(argv[0]);
        usleep(time * 1000);
    }
    return 0;
}

static int get_msg_flag(char *msg)
{
    int flag = 0;
    if (strstr(msg, "prepared")) {
        flag |= PLAYER_EVENT_MASK_PREPARED;
    } else if (strstr(msg, "played")) {
        flag |= PLAYER_EVENT_MASK_PLAYED;
    } else if (strstr(msg, "seeked")) {
        flag |= PLAYER_EVENT_MASK_SEEKED;
    } else if (strstr(msg, "stopped")) {
        flag |= PLAYER_EVENT_MASK_STOPPED;
    } else if (strstr(msg, "eos")) {
        flag |= PLAYER_EVENT_MASK_EOS;
    }
    return flag;
}

static esp_err_t wait_msg(int sel, int flag, int timeout)
{
    while (timeout > 0) {
        if (players_mask[sel] & PLAYER_EVENT_MASK_ERR) {
            ESP_LOGE(TAG, "State Error");
            return -2;
        }
        if (players_mask[sel] & flag) {
            return 0;
        }
        usleep(100000);
        timeout -= 100;
    }
    ESP_LOGE(TAG, "waitmsg timeout");
    return -1;
}

static esp_err_t cli_wait_msg(int argc, char **argv)
{
    int sel = 0;
    int i;
    int timeout = 0x7fffffff;
    char *msg = NULL;
    for (i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        } else if (SAME_STRING(argv[i], "-timeout")) {
            timeout = atoi(argv[++i]);
        } else if (SAME_STRING(argv[i], "-msg")) {
            msg = argv[++i];
        }
    }
    VERIFY_SEL(sel);
    if (msg) {
        int event = get_msg_flag(msg);
        if (event) {
            wait_msg(sel, event, timeout);
        }
    }
    return 0;
}

static esp_err_t cli_wait_eos(int argc, char **argv)
{
    int sel = 0;
    int timeout = 600000;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-sel")) {
            sel = atoi(argv[++i]);
        } else if (SAME_STRING(argv[i], "-timeout")) {
            timeout = atoi(argv[++i]);
        }
    }
    VERIFY_SEL(sel);
    if (wait_msg(sel, PLAYER_EVENT_MASK_PREPARED | PLAYER_EVENT_MASK_PLAYED, 2000) == 0) {
        ESP_LOGI(TAG, "Wait for play OK");
        wait_msg(sel, PLAYER_EVENT_MASK_EOS, timeout);
    }
    return 0;
}

static esp_err_t cli_quit_act(int argc, char **argv)
{
    printf("Enter quit\n");
    for (int i = 0; i < MAX_PLAYER_NUM; i++) {
        if (players[i]) {
            media_player_close(players[i]);
        }
    }
    unregister_component();
    cli_quit = 1;
    return 0;
}

static esp_err_t query_cli(int argc, char **argv)
{
    for (int i = 0; i < MAX_PLAYER_NUM; i++) {
        if (players[i]) {
            media_player_query(players[i]);
        }
    }
    return 0;
}

static esp_err_t dump_cli(int argc, char **argv)
{
    int mask = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-mask")) {
            mask = atoi(argv[++i]);
        }
    }
    media_player_dump(players[0], mask);
    return 0;
}

static esp_err_t vol_cli(int argc, char **argv)
{
    int vol = 0;
    if (argc > 0) {
        vol = atoi(argv[0]);
    }
    esp_codec_dev_set_out_vol(get_playback_handle(), vol);
    return 0;
}

static esp_err_t cli_dir(int argc, char **argv)
{
    const char *p;
    char *filter = NULL;
    if (argc == 0) {
        p = "/sdcard";
    } else {
        p = argv[0];
    }
    if (argc > 1) {
        filter = argv[1];
    }
    DIR *dir = opendir(p);
    int i = 0;
    if (dir) {
        struct dirent *c;
        while ((c = readdir(dir)) != NULL) {
            if (c->d_type != DT_DIR) {
                if (filter == NULL || strstr(c->d_name, filter) != NULL) {
                    printf("file %d: %s\n", i++, c->d_name);
                }
            } else {
                if (c->d_name[0] == '.') {
                    continue;
                }
                if (filter == NULL || strstr(c->d_name, filter) != NULL) {
                    printf("dir: %s\n", c->d_name);
                }
            }
        }
        closedir(dir);
    }
    return 0;
}

static esp_err_t cli_measure_fps(int argc, char **argv)
{
    if (argc == 0) {
        return -1;
    }
    return 0;
}

static esp_err_t cli_heap_detect(int argc, char **argv)
{
    if (argc < 1) {
        return -1;
    }
    if (SAME_STRING(argv[0], "stop")) {
        media_lib_stop_mem_trace();
        return 0;
    }
    if (SAME_STRING(argv[0], "start")) {
        char file_path[64];
        if (argc == 2) {
            snprintf(file_path, sizeof(file_path)-1, "/sdcard/%s", argv[1]);
        } else {
            strncpy(file_path, "/sdcard/mem.log", sizeof(file_path)-1);
        }
        media_lib_mem_trace_cfg_t trace_cfg = {
            .trace_type =
             MEDIA_LIB_MEM_TRACE_MODULE_USAGE | MEDIA_LIB_MEM_TRACE_LEAK | MEDIA_LIB_MEM_TRACE_SAVE_HISTORY,
            .save_cache_size = 30*1024,
            .save_path = file_path,
            .stack_depth = 3,
        };
        media_lib_start_mem_trace(&trace_cfg);
        return 0;
    }
    if (SAME_STRING(argv[0], "use")) {
        uint32_t used_size = 0, peak_size = 0;
        media_lib_get_mem_usage(NULL, &used_size, &peak_size);
        ESP_LOGI(TAG, "Library memory used %d peak %d", (int)used_size, (int)peak_size);
        return 0;
    }
    return 0;
}

static esp_err_t cli_filter_dir(int argc, char **argv)
{
    char *ext = "mp3;aac;mp4;m4a;wav;ts;ogg;opus;flac;avi;amr;m3u8";
    folder_deinit();
    if (argc == 0) {
        return 0;
    }
    char *p = argv[0];
    for (int i = 1; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-ext")) {
            ext = argv[++i];
        }
    }
    folder_init(p, ext);
    return 0;
}

typedef int (*net_parce_cb)(void *buffer, int len, void *ctx);

static int net_parse(char *url, net_parce_cb cb, void *ctx)
{
    media_src_t *src = media_src_open(MEDIA_SRC_TYPE_NETWORK);
    if (src == NULL) {
        return -1;
    }
    int ret = -1;
    uint64_t file_size = 0;
    void *data = NULL;
    media_src_connect(src, url);
    media_src_get_size(src, &file_size);
    printf("Read %s got size %lld\n", url, file_size);
    if (file_size < 1024 * 100) {
        data = media_lib_malloc((size_t) file_size);
    }
    if (data) {
        int n = media_src_read(src, data, (size_t) file_size);
        if (n > 0) {
            ret = cb(data, n, ctx);
        }
        media_lib_free(data);
    }
    media_src_close(src);
    return ret;
}

static inline char *read_to(char *s, char *p, int n)
{
    int len = strlen(p);
    while (n >= len) {
        if (*s == *p) {
            if (strncmp(s, p, len) == 0) {
                return s + len;
            }
        }
        n--;
        s++;
    }
    return NULL;
}

static int net_get_file_count_cb(void *buffer, int len, void *ctx)
{
    char *s = (char *) buffer;
    char *count = read_to(s, "\"filenum\": ", len);
    if (count) {
        *(int *) ctx = atoi(count);
        return 0;
    }
    return -1;
}

static int net_get_file_count(char *p)
{
    if (path_url == NULL) {
        path_url = (char *) media_lib_malloc(MAX_FILE_PATH_LEN);
    }
    if (path_url == NULL) {
        return 0;
    }
    int count = 0;
    snprintf(path_url, MAX_FILE_PATH_LEN, "http://%s/filter?contain=%s", filter_ip, p);
    net_parse(path_url, net_get_file_count_cb, &count);
    return count;
}

static int net_get_file_path_cb(void *buffer, int len, void *ctx)
{
    char *s = (char *) buffer;
    char *path = read_to(s, "\"filepath\": \"", len);
    if (path) {
        char *end = read_to(path, "\"", len - (int) (path - s));
        if (end) {
            int n = (end - 1 - path);
            if (n < MAX_FILE_PATH_LEN) {
                char *dst = (char *) ctx;
                memcpy(dst, path, n);
                dst[n] = 0;
                return 0;
            }
        }
    }
    return -1;
}

static char *net_get_file(char *p, int index)
{
    if (path_url == NULL) {
        return NULL;
    }
    snprintf(path_url, MAX_FILE_PATH_LEN, "http://%s/filter?contain=%s&index=%d", filter_ip, p, index);
    int ret = net_parse(path_url, net_get_file_path_cb, path_url);
    return (ret == 0) ? path_url : NULL;
}

static esp_err_t cli_list_file(int argc, char **argv)
{
    if (filter_file_pattern) {
        media_lib_free(filter_file_pattern);
        filter_file_pattern = NULL;
        if (filter_ip) {
            media_lib_free(filter_ip);
            filter_ip = NULL;
        }
        filter_file_count = filter_file_idx = 0;
    }
    if (argc == 0) {
        return 0;
    }
    for (int i = 1; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-ip")) {
            filter_ip = media_lib_strdup(argv[++i]);
        }
    }
    filter_file_pattern = media_lib_strdup(argv[0]);
    if (filter_ip) {
        filter_file_count = net_get_file_count(argv[0]);
    } else {
        filter_file_count = folder_get_file_count(argv[0]);
    }
    filter_file_idx = -1;
    printf("%s match %d files\n", argv[0], filter_file_count);
    return 0;
}

static void play_dir(int next)
{
    if (filter_file_pattern == NULL || filter_file_count == 0) {
        return;
    }
    if (next) {
        if (filter_file_idx < filter_file_count) {
            filter_file_idx++;
        }
    } else {
        if (filter_file_idx >= 0) {
            filter_file_idx--;
        }
    }
    printf("Play Reached %d/%d\n", filter_file_idx + 1, filter_file_count);
    if (filter_file_idx < 0 || filter_file_idx >= filter_file_count) {
        return;
    }
    static char cmd[256];
    snprintf(cmd, sizeof(cmd), "close");
    parse_cmd(cmd);
    char *f;
    if (filter_ip) {
        f = net_get_file(filter_file_pattern, filter_file_idx);
    } else {
        f = folder_get_file(filter_file_pattern, filter_file_idx);
    }
    if (f) {
        snprintf(cmd, sizeof(cmd), "play -mask 3 -uri \"%s\"", f);
        parse_cmd(cmd);
    }
}

static esp_err_t cli_play_pre(int argc, char **argv)
{
    play_dir(0);
    return 0;
}

static esp_err_t cli_play_nxt(int argc, char **argv)
{
    play_dir(1);
    return 0;
}

static esp_err_t cli_play_recursive(int argc, char **argv)
{
    int timeout = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-timeout")) {
            timeout = atoi(argv[++i]);
        }
    }
    while (filter_file_idx < filter_file_count) {
        play_dir(1);
        char cmd[256];
        if (timeout == 0) {
            snprintf(cmd, sizeof(cmd), "waiteos");
        } else {
            snprintf(cmd, sizeof(cmd), "waiteos -timeout %d", timeout);
        }
        parse_cmd(cmd);
    }
    return 0;
}

static esp_err_t cli_mem_leak(int argc, char **argv)
{
    media_lib_print_leakage(NULL);
    return 0;
}

static esp_err_t cli_speed_measure(int argc, char **argv)
{
    if (argc == 0) {
        return -1;
    }
    int timeout = 0;
    for (int i = 0; i + 1 < argc; i++) {
        if (SAME_STRING(argv[i], "-timeout")) {
            timeout = atoi(argv[++i]);
        }
    }
    media_src_t *src = media_src_open(MEDIA_SRC_TYPE_NETWORK);
    if (src == NULL) {
        return -1;
    }
    int ret = media_src_connect(src, argv[0]);
    if (ret == 0) {
        uint64_t file_size = 0;
        media_src_get_size(src, &file_size);
        printf("File size is:%lld\n", file_size);
        uint8_t *data = media_lib_malloc(100 * 1024);
        if (data) {
            int readed = 0;
            int start_time = get_cur_time();
            int pre = start_time;
            int pre_read = 0;
            while (readed < file_size) {
                int n = media_src_read(src, data, 100 * 1024);
                if (n <= 0) {
                    break;
                }
                pre_read += n;
                readed += n;
                int cur = get_cur_time();
                if (cur > pre + 1000) {
                    printf("Speed real:%lld aver:%lld\n", (uint64_t) pre_read * 1000 / (cur - pre),
                           (uint64_t) readed * 1000 / (cur - start_time));
                    pre = cur;
                    pre_read = 0;
                }
                if (timeout && cur >= start_time + timeout) {
                    break;
                }
            }
            media_lib_free(data);
        }
    }
    media_src_close(src);
    return 0;
}

static esp_err_t cli_assert(int argc, char **argv)
{
    *((int *) 0) = 0;
    return 0;
}

void measure_enable(bool enable);

static int cpu_cli(int argc, char **argv)
{
    measure_enable(true);
    sys_state_show();
    measure_enable(false);
    return 0;
}

static const cli_entry_t cli_entries[] = {
    {
     .cmd = "quit",
     .help = "",
     .func = cli_quit_act,
     },
    {
     .cmd = "open",
     .help = "-mask(?) 1/2/3 -sel(?) 0/1",
     .func = cli_open,
     },
    {
     .cmd = "close",
     .help = "-sel(?) 0/1",
     .func = cli_close,
     },
    {
     .cmd = "stop",
     .help = "-sel(?) 0/1",
     .func = cli_stop,
     },
    {
     .cmd = "seek",
     .help = "time(ms) -sel(?) 0/1",
     .func = cli_seek,
     },
    {
     .cmd = "play",
     .help = "-uri(?) path -sel(?) 0/1",
     .func = cli_play,
     },
    {
     .cmd = "pause",
     .help = "-sel(?) 0/1",
     .func = cli_pause,
     },
    {
     .cmd = "resume",
     .help = "-sel(?) 0/1",
     .func = cli_resume,
     },
     {
     .cmd = "speed",
     .help = "-sel(?) 0/1 0.5/1.0/2.0",
     .func = cli_speed,
     },
    {
     .cmd = "int",
     .help = "-sel(?) 0/1 interrupt current playback",
     .func = cli_interrupt,
     },
    {
     .cmd = "res",
     .help = "-sel(?) 0/1 restore playback",
     .func = cli_restore,
     },
    {
     .cmd = "loop",
     .help = "-sel(?) 0/1 Enable loop playback",
     .func = cli_loop,
     },
    {
     .cmd = "wait",
     .help = "time(ms)",
     .func = cli_wait,
     },
    {
     .cmd = "waitmsg",
     .help = "prepared|played|stopped|seeked|eos",
     .func = cli_wait_msg,
     },
    {
     .cmd = "param",
     .help = "-dynamic(?) 0/1 [mp4 dynamic load]",
     .func = cli_param,
     },
    {
     .cmd = "param",
     .help = "-dynamic(?) 0/1 [mp4 dynamic load]",
     .func = cli_param,
     },
     {
     .cmd = "playby",
     .help = "-cb/-buf -uri url",
     .func = cli_playby,
     },
    {
     .cmd = "waiteos",
     .help = "",
     .func = cli_wait_eos,
     },
    {
     .cmd = "dir",
     .help = "dirpath",
     .func = cli_dir,
     },
    {
     .cmd = "fps",
     .help = "0 normal; 1 measure only; 2 with render;",
     .func = cli_measure_fps,
     },
    {
     .cmd = "heap",
     .help = "start,stop,leak,use",
     .func = cli_heap_detect,
     },
    {
     .cmd = "filter",
     .help = "dirname -ext extension filter dir by extension, no arg close",
     .func = cli_filter_dir,
     },
    {
     .cmd = "file",
     .help = "name filter file by name",
     .func = cli_list_file,
     },
    {
     .cmd = "p",
     .help = "play previous file",
     .func = cli_play_pre,
     },
    {
     .cmd = "n",
     .help = "play next file",
     .func = cli_play_nxt,
     },
    {
     .cmd = "recur",
     .help = "Recursive play",
     .func = cli_play_recursive,
     },
    {
     .cmd = "leak",
     .help = "detect leakage",
     .func = cli_mem_leak,
     },
    {
     .cmd = "speed",
     .help = "test network speed",
     .func = cli_speed_measure,
     },
    {
     .cmd = "assert",
     .help = "assert system for gdb",
     .func = cli_assert,
     },
    {
     .func = cpu_cli,
     .help = "CPU usage",
     .cmd = "i",
     },
    {
     .func = query_cli,
     .help = "Query player status",
     .cmd = "q",
    },
    {
     .func = dump_cli,
     .help = "Dump data",
     .cmd = "dump",
    },
    {
     .func = vol_cli,
     .help = "Set volume",
     .cmd = "vol",
    },
};

static int parse_cmd(char *buffer)
{
    char *cmds[16];
    int char_cnt = 0;
    int cmd_idx = 0;
    char string_sep = 0;
    if (*buffer == '#') {
        return 0;
    }
    while (*buffer) {
        if (string_sep) {
            if (*buffer == string_sep) {
                string_sep = 0;
                buffer++;
                continue;
            }
            if (*buffer == '\\') {
                if (*(buffer + 1) == string_sep) {
                    buffer++;
                }
            }
        } else if (*buffer == '\'' || *buffer == '"' || *buffer == '\n') {
            string_sep = *(buffer++);
            continue;
        } else if (*buffer == ' ' || *buffer == '\t') {
            if (char_cnt) {
                cmds[cmd_idx][char_cnt] = 0;
                char_cnt = 0;
                cmd_idx++;
                if (cmd_idx == 16) {
                    break;
                }
            }
            buffer++;
            continue;
        }
        if (char_cnt == 0) {
            if (cmd_idx < 16) {
                cmds[cmd_idx] = buffer;
            }
        } else {
            cmds[cmd_idx][char_cnt] = *buffer;
        }
        char_cnt++;
        buffer++;
    }
    if (char_cnt) {
        cmds[cmd_idx][char_cnt] = 0;
        char_cnt = 0;
        cmd_idx++;
    }
    if (cmd_idx == 0) {
        return 0;
    }
    lc(cmds[0]);
    for (int i = 0; i < cmd_idx; i++) {
        printf("%d: %s\n", i, cmds[i]);
    }
    for (int i = 0; i < sizeof(cli_entries) / sizeof(cli_entry_t); i++) {
        if (SAME_STRING(cmds[0], cli_entries[i].cmd)) {
            cli_entries[i].func(cmd_idx - 1, &cmds[1]);
            break;
        }
    }
    return 0;
}

static int console_action(int argc, char **argv)
{
    for (int i = 0; i < sizeof(cli_entries) / sizeof(cli_entry_t); i++) {
        if (argc > 0 && strcmp(argv[0], cli_entries[i].cmd) == 0) {
            argc--;
            argv++;
            cli_entries[i].func(argc, argc ? argv : NULL);
            break;
        }
    }
    return 0;
}

static int init_console()
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "esp>";
    repl_config.task_priority = 22;
    repl_config.max_cmdline_length = 1024;
    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif
    esp_console_cmd_t console_cmd = {};
    for (int i = 0; i < sizeof(cli_entries) / sizeof(cli_entry_t); i++) {
        console_cmd.command = cli_entries[i].cmd;
        console_cmd.help = cli_entries[i].help;
        console_cmd.func = console_action;
        ESP_ERROR_CHECK(esp_console_cmd_register(&console_cmd));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    return 0;
}

int player_cli_run(char *file_name)
{
    register_media_src();
    register_extractor();
    register_audio_decoder();
    register_video_decoder();

#if 0
    //test your files can use gdb to debug
#define RUN_CMD(c)      \
    {                   \
        strcpy(cmd, c); \
        parse_cmd(cmd); \
    }
    while (1) {
        char cmd[64];
        printf("Start to wait\n");
        RUN_CMD("wait 60000");
        RUN_CMD("play -uri /sdcard/audio/alac.m4a");
        RUN_CMD("waiteos");
        break;
        RUN_CMD("file avi -ip 192.168.4.1");
        RUN_CMD("recur");
        break;
        RUN_CMD("play -mask 3 -uri /sdcard/out.mp4");
        RUN_CMD("waiteos");
        RUN_CMD("close");
    }
    return 0;
#endif
    init_console();

    if (file_name) {
        FILE *fp = fopen(file_name, "r");
        if (fp == NULL) {
            printf("File %s not exists\n", file_name);
            return 0;
        }
        static char line_buffer[1024];
        while (!feof(fp) && cli_quit == 0) {
            fgets(line_buffer, sizeof(line_buffer), fp);
            parse_cmd(line_buffer);
        }
        if (!cli_quit) {
            cli_quit_act(0, NULL);
        }
        fclose(fp);
    }
    return 0;
}
