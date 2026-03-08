
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/unistd.h>
#include "esp_heap_caps.h"
#include "media_src_storage.h"
#include "esp_timer.h"
#include "esp_log.h"

#define CACHE_SIZE (32*1024)

#define USE_ALIGN_CACHE
#define TAG "Src_Storage"

typedef struct {
#ifdef USE_ALIGN_CACHE
    uint8_t* align_buffer;
    int      readed;
    int      filled;
    bool     eof;
    int      seek_pos;
    int      align_pos;
    int      buffer_pos;
#endif
    FILE*    fp;
} storage_src_t;

#define ALIGN_TO(pos, align) (pos & (~((align)-1)))

static void media_src_storage_flush(storage_src_t* m)
{
#ifdef USE_ALIGN_CACHE
    m->filled = m->readed = 0;
    m->eof = false;
    m->align_pos = m->seek_pos = 0;
    m->buffer_pos = 0;
#endif
}

#ifdef USE_ALIGN_CACHE
static int cache_data(storage_src_t* m, void *data, int n) {
    int sent = 0;
    static int copy_used = 0;
    if (m->filled > m->readed) {
        sent = m->filled - m->readed;
        if (m->align_pos < m->seek_pos) {
            int need_skip = m->seek_pos - m->align_pos;
            if (need_skip > sent) {
                m->readed += sent;
                sent = 0;
                m->align_pos += sent;
            } else {
                sent -= need_skip;
                m->readed += need_skip;
                m->align_pos += need_skip;
            }
        }
        if (sent > n) {
            sent = n;
        }
        if (sent) {
            int start = (int)(esp_timer_get_time()/1000);
            memcpy(data, m->align_buffer + m->readed, sent);
            copy_used += (int)(esp_timer_get_time()/1000) - start;
            m->readed += sent;
        }
    }
    if (m->readed >= m->filled) {
        m->buffer_pos += m->filled;
        m->readed = m->filled = 0;
    }
    if (m->filled == 0) {
        static int total_size = 0;
        static int used_time = 0;
        int start = (int)(esp_timer_get_time()/1000);
        int n = read(fileno(m->fp), m->align_buffer, CACHE_SIZE);
        int end = (int)(esp_timer_get_time()/1000);
        total_size += n;
        used_time += end - start;
        if (used_time > 5000) {
            ESP_LOGI(TAG, "Readspeed %d %d copy:%d", total_size, (int)((uint64_t)total_size * 1000 / used_time), 
                    copy_used*100/used_time);
            used_time = 0;
            total_size = 0;
            copy_used = 0;
        }
        //int n = fread(m->align_buffer, 1, CACHE_SIZE, m->fp);
        if (n < 0) {
            return n;
        }
        if (n < CACHE_SIZE) {
            m->eof = true;
        }
        m->filled = n;
    }
    return sent;

}

static int read_from_cache(storage_src_t* m, void *data, size_t len) {
    int read = 0;
    while (len > 0) {
        if (m->eof && m->filled == 0) {
            break;
        }
        int n = cache_data(m, data, len);
        if (n < 0) {
            return n;
        }
        data += n;
        len -= n;
        read += n;
    }
    return read;
}
#endif

int media_src_storage_open(media_src_t *src)
{
    storage_src_t* m = calloc(1, sizeof(storage_src_t));
    if (m == NULL) {
        return -1;
    }
#ifdef USE_ALIGN_CACHE
    m->align_buffer = heap_caps_aligned_alloc(64, CACHE_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (m->align_buffer == NULL) {
        free(m);
        return -1;
    }
#endif
    src->sub_src = m;
    return 0;
}

int media_src_storage_connect(media_src_t *src, char *uri)
{
    storage_src_t* m = (storage_src_t*)src->sub_src;
    if (m->fp) {
        fclose(m->fp);
    }
    media_src_storage_flush(m);
    m->fp = fopen(uri, "rb");
    if (m->fp) {
        return 0;
    }
    return -1;
}

int media_src_storage_disconnect(media_src_t *src)
{
    storage_src_t* m = (storage_src_t*)src->sub_src;
    if (m->fp) {
        fclose(m->fp);
    }
    media_src_storage_flush(m);
    return 0;
}

int media_src_storage_read(media_src_t *src, void *data, size_t len)
{
    storage_src_t* m = (storage_src_t*)src->sub_src;
    if (m->fp) {
#ifdef USE_ALIGN_CACHE
        return read_from_cache(m, data, len);
#else
        return fread(data, 1, len, m->fp);
#endif
    }
    return -1;
}

int media_src_storage_seek(media_src_t *src, uint64_t position)
{
    storage_src_t* m = (storage_src_t*)src->sub_src;
    if (m->fp) {
#ifdef USE_ALIGN_CACHE
        if (m->filled && position >= m->buffer_pos && position <= m->buffer_pos + m->filled) {
            // Still in cached memory
            m->readed = (position - m->buffer_pos);
            return 0;
        }
#endif
        media_src_storage_flush(m);
#ifdef USE_ALIGN_CACHE
        m->align_pos = (int) ALIGN_TO(position, 1024);
        m->seek_pos = (int)position;
        m->buffer_pos = m->align_pos;
        position = m->align_pos;
#endif
        return fseek(m->fp, position, SEEK_SET);
    }
    return -1;
}

int media_src_storage_get_position(media_src_t *src, uint64_t *position)
{
    storage_src_t* m = (storage_src_t*)src->sub_src;
    if (m->fp) {
        *position = ftell(m->fp);
        return 0;
    }
    return -1;
}

int media_src_storage_get_size(media_src_t *src, uint64_t *size)
{
    storage_src_t* m = (storage_src_t*)src->sub_src;
    if (m->fp) {
        uint32_t old = ftell(m->fp);
        fseek(m->fp, 0, 2);
        long end = ftell(m->fp);
        fseek(m->fp, old, 0);
        *size = (end <= 0 ? 0 : (uint32_t) end);
        return 0;
    }
    return -1;
}

int media_src_storage_set_info(media_src_t *src, media_src_set_type_t set_type, void *data, size_t len)
{
    return 0;
}

int media_src_storage_close(media_src_t *src)
{
    storage_src_t* m = (storage_src_t*)src->sub_src;
    if (m->fp) {
        fclose((FILE *) m->fp);
        m->fp = NULL;
    }
#ifdef USE_ALIGN_CACHE
    if (m->align_buffer) {
        free(m->align_buffer);
    }
#endif
    free(m);
    return 0;
}

int media_src_register_storage()
{
    media_src_func_t src_func = {
        .open = media_src_storage_open,
        .connect = media_src_storage_connect,
        .disconnect = media_src_storage_disconnect,
        .read = media_src_storage_read,
        .seek = media_src_storage_seek,
        .get_pos = media_src_storage_get_position,
        .file_size = media_src_storage_get_size,
        .close = media_src_storage_close,
        .set = media_src_storage_set_info,
    };
    return media_src_register(MEDIA_SRC_TYPE_STORAGE, &src_func);
}
