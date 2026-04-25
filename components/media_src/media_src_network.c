
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "media_src_network.h"
#include "esp_http_client.h"
#include "media_lib_os.h"
#include "esp_log.h"
#include "errno.h"
#include "lwip/sockets.h"
//#include "esp_transport.h"
#include "esp_timer.h"
#include "media_src.h"
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

#define SEEK_ACTION_LENGTH (500 * 1024)

#define TAG                "NW"
typedef struct {
    int                          buffer_size;
    int                          filled;
    int                          pos;
    int                          rp;
    int                          need;
    int                          used;
    void                        *buffer;
    bool                         eos;
    bool                         reset;
    bool                         quit;
    media_lib_event_grp_handle_t event;
    media_lib_mutex_handle_t     protect;
} file_buffer_t;

typedef struct {
    esp_http_client_handle_t  http_handle;
    int                       file_size;
    uint32_t                  position;
    char                     *url;
    file_buffer_t             cache_buffer;
    media_lib_thread_handle_t read_thread;
    media_lib_mutex_handle_t  api_lock;
    bool                      connected;
} media_src_network_t;

#define BUFFER_SIZE         (500 * 1024)

#define READ_EVENT_RESET_OK (1)
#define READ_EVENT_DATA_OK  (2)
#define READ_EVENT_QUIT_OK  (4)
#define READ_EVENT_GO       (8)

static int _open_url(media_src_network_t *nw, char *url, uint32_t position);

static void file_buffer_reset(file_buffer_t *f)
{
    f->filled = 0;
    f->used = 0;
    f->pos = f->rp;
    f->need = 0;
    f->eos = false;
}

int file_buffer_close(file_buffer_t *b)
{
    if (b->buffer) {
        media_lib_free(b->buffer);
        ESP_LOGI(TAG, "fifo free %p", b->buffer);
    }
    if (b->event) {
        media_lib_event_group_destroy(b->event);
    }
    if (b->protect) {
        media_lib_mutex_destroy(b->protect);
    }
    memset(b, 0, sizeof(file_buffer_t));
    return 0;
}

static inline int in_cache(file_buffer_t *f, int p)
{
    return (p >= f->pos && (p <= f->pos + f->filled || f->eos));
}

static inline int get_left_size(file_buffer_t *f, int size)
{
    if (f->eos && f->rp + size > f->pos + f->filled) {
        int n = f->pos + f->filled - f->rp;
        return (n >= 0) ? n : 0;
    }
    return size;
}

int file_buffer_open(file_buffer_t *b, int pos)
{
    do {
        memset(b, 0, sizeof(file_buffer_t));
        b->buffer = media_lib_malloc(BUFFER_SIZE);
        if (b->buffer == NULL) {
            ESP_LOGE(TAG, "Fail to alloc buffer");
            break;
        }
        ESP_LOGI(TAG, "fifo malloc %p", b->buffer);
        b->buffer_size = BUFFER_SIZE;
        if (media_lib_event_group_create(&b->event) != 0) {
            break;
        }
        if (media_lib_mutex_create(&b->protect) != 0) {
            break;
        }
        file_buffer_reset(b);
        return 0;
    } while (0);
    file_buffer_close(b);
    return -1;
}

int file_buffer_seek(file_buffer_t *b, int pos)
{
    media_lib_event_group_clr_bits(b->event, READ_EVENT_GO);
    media_lib_event_group_clr_bits(b->event, READ_EVENT_RESET_OK);
    b->rp = pos;
    b->reset = true;
    media_lib_mutex_unlock(b->protect);
    media_lib_event_group_wait_bits(b->event, READ_EVENT_RESET_OK, 10000);
    return 0;
}

int file_buffer_go(file_buffer_t *b)
{
    media_lib_event_group_set_bits(b->event, READ_EVENT_GO);
    return 0;
}

int file_buffer_read(file_buffer_t *f, void *buffer, int size)
{
    // if in cache
    while (f->quit == false) {
        media_lib_mutex_lock(f->protect, 10000);
        if (in_cache(f, f->rp) && in_cache(f, f->rp + size)) {
            int s = get_left_size(f, size);
            if (s) {
                if (buffer) {
                    memcpy(buffer, (f->buffer + f->rp - f->pos), s);
                }
                f->rp += s;
            }
            f->need = 0;
            media_lib_mutex_unlock(f->protect);
            return s;
        } else {
            f->need = size;
            media_lib_mutex_unlock(f->protect);
            media_lib_event_group_clr_bits(f->event, READ_EVENT_DATA_OK);
            ESP_LOGI(TAG, "waiting for data rp %d\n", f->rp);
            media_lib_event_group_wait_bits(f->event, READ_EVENT_DATA_OK, 1000);
        }
    }
    return -1;
}

static void file_cache_thread(void *ctx)
{
    media_src_network_t *src = (media_src_network_t *) ctx;
    file_buffer_t *f = &src->cache_buffer;
    int half = f->buffer_size >> 1;
    int total_read = 0, pre_read = 0;
    uint64_t total_time = 0;
    uint64_t aver_time = 0;
    while (!f->quit) {
        if (f->reset) {
            file_buffer_reset(f);
            f->reset = false;
            media_lib_event_group_set_bits(f->event, READ_EVENT_RESET_OK);
            media_lib_event_group_wait_bits(f->event, READ_EVENT_GO, 1000);
            continue;
        }
        int size = 20 * 1024;
        // half used
        media_lib_mutex_lock(f->protect, 10000);
        if (in_cache(f, f->rp) && f->rp >= f->pos + half) {
            memcpy(f->buffer, f->buffer + half, half);
            f->pos += half;
            // ESP_LOGI(TAG, "consume half to %d size:%d\n", f->pos, f->filled);
            f->filled -= half;
        }
        media_lib_mutex_unlock(f->protect);
        
        bool eos = f->eos;
        if (f->eos == false && f->filled + size <= f->buffer_size) {
            uint64_t start_time = esp_timer_get_time();
            int got = esp_http_client_read(src->http_handle, (char *) f->buffer + f->filled, size);
            if (got < size && errno != 0 && src->connected) {
                int ret = _open_url(src, src->url, (uint32_t) f->pos + f->filled);
                ESP_LOGI(TAG, "Reconnect ret %d", ret);
                if (ret == 0) {
                    continue;
                }
            }
            if (got < size) {
                if (got < 0 || esp_http_client_is_complete_data_received(src->http_handle)) {
                    eos = true;
                }
                ESP_LOGI(TAG, "Read end got %d pos:%d eos:%d", got, f->pos + f->filled, eos);
            }
            if (got < 0) {
                media_lib_event_group_set_bits(f->event, READ_EVENT_DATA_OK);
                ESP_LOGE(TAG, "Fail to read data ret %d", got);
                continue;
            }
            media_lib_mutex_lock(f->protect, 10000);
            f->filled += got;
            f->eos = eos;
            if (f->need && in_cache(f, f->rp + f->need)) {
                media_lib_event_group_set_bits(f->event, READ_EVENT_DATA_OK);
            }
            media_lib_mutex_unlock(f->protect);
            uint64_t elapse = esp_timer_get_time() - start_time;
            aver_time += elapse;
            total_time += elapse;
            total_read += size;
            pre_read += size;
            // printf("Buff to %d size:%d rp:%d\n", f->pos, f->filled, f->rp);
            if (aver_time >= 2 * 1000 * 1000) {
                uint32_t aver = (uint32_t) ((uint64_t) total_read * 1000 / (total_time / 1000));
                uint32_t real = (uint32_t) ((uint64_t) pre_read * 1000 / (aver_time / 1000));
                pre_read = 0;
                aver_time = 0;
                printf("Buff to %d size:%d rp:%d\n", f->pos, f->filled, f->rp);
                ESP_LOGI(TAG, "Read speed real %d aver %d", (int)real, (int)aver);
            }
        } else {
            // buffer whole
            media_lib_thread_sleep(20);
        }
    }
    media_lib_event_group_set_bits(f->event, READ_EVENT_DATA_OK);
    media_lib_event_group_set_bits(f->event, READ_EVENT_QUIT_OK);
    media_lib_thread_destroy(NULL);
}

int file_buffer_quit(file_buffer_t *b)
{
    b->quit = true;
    if (b->event) {
        media_lib_event_group_clr_bits(b->event, READ_EVENT_QUIT_OK);
        media_lib_event_group_wait_bits(b->event, READ_EVENT_QUIT_OK, 10000);
    }
    if (b->protect) {
        media_lib_mutex_lock(b->protect, 10000);
        media_lib_mutex_unlock(b->protect);
    }
    b->quit = false;
    return -1;
}

int __attribute__((weak)) esp_http_client_get_socket(esp_http_client_handle_t client)
{
    return -1;
}

static int fast_close_client(esp_http_client_handle_t client)
{
    struct timeval tv;
    // hacking code
    int fd = esp_http_client_get_socket(client);
    printf("Get socket fd %d\n", fd);
    if (fd >= 0) {
        tv.tv_sec = 0;
        tv.tv_usec = 1000;
        if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
            ESP_LOGE(TAG, "Fail to setsockopt SO_RCVTIMEO");
        }
        if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) != 0) {
            ESP_LOGE(TAG, "Fail to setsockopt SO_SNDTIMEO");
        }
        lwip_close(fd);
    }
    return 0;
}

int media_src_network_disconnect(media_src_t *src)
{
    if (src == NULL || src->sub_src == NULL) {
        return -1;
    }
    media_src_network_t *nw = (media_src_network_t *) src->sub_src;
    nw->connected = false;
    if (nw->http_handle) {
        fast_close_client(nw->http_handle);
    }
    media_lib_mutex_lock(nw->api_lock, 10000);
    printf("Quit cache start %p\n", &nw->cache_buffer);
    if (nw->read_thread) {
        file_buffer_quit(&nw->cache_buffer);
        nw->read_thread = NULL;
    }
    printf("close cache start\n");
    file_buffer_close(&nw->cache_buffer);
    if (nw->http_handle) {
        esp_http_client_cleanup(nw->http_handle);
        free(nw->url);
        nw->url = NULL;
        nw->http_handle = NULL;
    }
    media_lib_mutex_unlock(nw->api_lock);
    printf("Disconenct finished\n");
    return 0;
}

int media_src_network_close(media_src_t *src)
{
    if (src && src->sub_src) {
        media_src_network_t *nw = (media_src_network_t *) src->sub_src;
        media_src_network_disconnect(src);
        media_lib_mutex_destroy(nw->api_lock);
        free(src->sub_src);
        src->sub_src = NULL;
    }
    return 0;
}

int media_src_network_open(media_src_t *src)
{
    media_src_network_t *nw = (media_src_network_t *) malloc(sizeof(media_src_network_t));
    while (nw) {
        memset(nw, 0, sizeof(media_src_network_t));
        if (media_lib_mutex_create(&nw->api_lock) != 0) {
            break;
        }
        src->sub_src = nw;
        return 0;
    }
    if (nw) {
        free(nw);
    }
    return -1;
}

static int _open_url(media_src_network_t *nw, char *url, uint32_t position)
{
    esp_http_client_handle_t http_client = nw->http_handle;
    // TODO not cleanup use prev one
    if (http_client) {
        esp_http_client_cleanup(nw->http_handle);
        http_client = NULL;
    }
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 5 * 1000,
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
    };
    if (strncmp(url, "https://", 8) == 0) {
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
       config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
    }
    http_client = esp_http_client_init(&config);
    if (http_client == NULL) {
        return -1;
    }
    nw->http_handle = http_client;
    do {
        esp_http_client_set_header(http_client, "User-Agent", "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/117.0.0.0 Safari/537.36");
        if (position > 0) {
            char rang_header[32];
            snprintf(rang_header, 32, "bytes=%d-", (int) position);
            esp_http_client_set_header(http_client, "Range", rang_header);
            printf("request header:%s\n", rang_header);
        }
        if (esp_http_client_open(http_client, 0) != ESP_OK) {
            break;
        }
        int size = esp_http_client_fetch_headers(http_client);
        printf("Got file size %d\n", size);
        if (size == 0) {
            if (esp_http_client_is_complete_data_received(http_client) == false) {
                size = 0x7fffffff;
            }
        }
        if (size >= 0) {
            nw->file_size = size;
        }
        int status_code = esp_http_client_get_status_code(http_client);
        printf("Got Status code %d\n", status_code);
        if (status_code < 0) {
            break;
        }
        if (status_code == 302) {
            esp_http_client_set_redirection(http_client);
            esp_http_client_close(http_client);
            continue;
        }
        // error status
        if (status_code > 400) {
            nw->position = 0xFFFFFFFF;
            nw->file_size = 0;
            break;
        }
        nw->position = position;
        printf("HTTP open ok\n");
        return 0;
    } while (1);
    return -1;
}

int media_src_network_connect(media_src_t *src, char *uri)
{
    if (src == NULL || uri == NULL || src->sub_src == NULL) {
        return -1;
    }
    media_src_network_disconnect(src);

    media_src_network_t *nw = (media_src_network_t *) src->sub_src;
    media_lib_mutex_lock(nw->api_lock, 10000);
    nw->url = strdup(uri);
    printf("Start connect url %s\n", uri);
    int ret = _open_url(nw, uri, 0);
    printf("connect open url %s ret %d\n", uri, ret);
    if (ret == 0) {
        ret = file_buffer_open(&nw->cache_buffer, 0);
        if (ret == 0) {
            media_lib_thread_create(&nw->read_thread, "Read Thread", file_cache_thread, nw, 8 * 1024, 23, 1);
            nw->connected = true;
            media_lib_mutex_unlock(nw->api_lock);
            return 0;
        }
    }
    media_lib_mutex_unlock(nw->api_lock);
    // never access src resource after fail
    return -1;
}

int media_src_network_read(media_src_t *src, void *data, size_t len)
{
    if (src == NULL || src->sub_src == NULL || data == NULL || len == 0) {
        return -1;
    }
    media_src_network_t *nw = (media_src_network_t *) src->sub_src;
    media_lib_mutex_lock(nw->api_lock, 10000);
    if (nw->file_size == 0) {
        media_lib_mutex_unlock(nw->api_lock);
        return 0;
    }
    int s = file_buffer_read(&nw->cache_buffer, data, len);
    if (s > 0) {
        if (s < len && nw->position + len <= nw->file_size) {
            ESP_LOGE(TAG, "Read less data at %d + %d < %d", (int)nw->position, (int)len, (int)nw->file_size);
        }
        nw->position += s;
    }
    media_lib_mutex_unlock(nw->api_lock);
    return s;
}

int media_src_network_seek(media_src_t *src, uint64_t position)
{
    if (src == NULL || src->sub_src == NULL) {
        return -1;
    }
    media_src_network_t *nw = (media_src_network_t *) src->sub_src;
    if (position == nw->position) {
        return 0;
    }
    media_lib_mutex_lock(nw->api_lock, 10000);
    if (in_cache(&nw->cache_buffer, (int) position)) {
        media_lib_mutex_lock(nw->cache_buffer.protect, 10000);
        nw->cache_buffer.rp = (int) position;
        nw->position = (uint32_t) position;
        media_lib_mutex_unlock(nw->cache_buffer.protect);
        media_lib_mutex_unlock(nw->api_lock);
        return 0;
    }
    // read not do seek
    if (position > nw->position && position <= nw->position + SEEK_ACTION_LENGTH) {
        int len = position - nw->position;
        int each_len = len > 1024 ? 1024 : len;
        while (len > 0) {
            if (len < each_len) {
                each_len = len;
            }
            int readed = file_buffer_read(&nw->cache_buffer, NULL, each_len);
            if (readed <= 0) {
                break;
            }
            nw->position += readed;
            len -= readed;
        }
        media_lib_mutex_unlock(nw->api_lock);
        return 0;
    }
    file_buffer_seek(&nw->cache_buffer, (int) position);
    int ret = _open_url(nw, nw->url, (uint32_t) position);
    if (ret == 0) {
        nw->position = (uint32_t) position;
        file_buffer_go(&nw->cache_buffer);
    }
    media_lib_mutex_unlock(nw->api_lock);
    return ret;
}

int media_src_network_get_position(media_src_t *src, uint64_t *position)
{
    if (src == NULL || src->sub_src == NULL || position == NULL) {
        return -1;
    }
    media_src_network_t *nw = (media_src_network_t *) src->sub_src;
    media_lib_mutex_lock(nw->api_lock, 10000);
    *position = nw->position;
    media_lib_mutex_unlock(nw->api_lock);
    return 0;
}

int media_src_network_set_info(media_src_t *src, media_src_set_type_t set_type, void *data, size_t len)
{
    return 0;
}

int media_src_network_get_size(media_src_t *src, uint64_t *size)
{
    if (src == NULL || src->sub_src == NULL) {
        return -1;
    }
    media_src_network_t *nw = (media_src_network_t *) src->sub_src;
    *size = nw->file_size;
    return 0;
}

int media_src_register_network()
{
    media_src_func_t src_func = {
        .open = media_src_network_open,
        .connect = media_src_network_connect,
        .disconnect = media_src_network_disconnect,
        .read = media_src_network_read,
        .seek = media_src_network_seek,
        .get_pos = media_src_network_get_position,
        .close = media_src_network_close,
        .set = media_src_network_set_info,
        .file_size = media_src_network_get_size,
    };
    return media_src_register(MEDIA_SRC_TYPE_NETWORK, &src_func);
}
