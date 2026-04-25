#include "media_src.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"

static media_src_func_t media_src_table[MEDIA_SRC_TYPE_MAX];

int media_src_register(media_src_type_t src_type, media_src_func_t *src_table)
{
    if (src_type >= MEDIA_SRC_TYPE_MAX) {
        return -1;
    }
    memcpy(&media_src_table[src_type], src_table, sizeof(media_src_func_t));
    return 0;
}

static media_src_func_t *get_src_table(media_src_t *src)
{
    if (src->src_type >= MEDIA_SRC_TYPE_MAX) {
        return NULL;
    }
    return &media_src_table[src->src_type];
}

media_src_t *media_src_open(media_src_type_t src_type)
{
    if (src_type >= MEDIA_SRC_TYPE_MAX) {
        return NULL;
    }

    media_src_t *src = (media_src_t *) malloc(sizeof(media_src_t));
    if (src == NULL) {
        return NULL;
    }
    memset(src, 0, sizeof(media_src_t));
    src->src_type = src_type;

    media_src_func_t *src_table = get_src_table(src);
    if (src_table && src_table->open) {
        int ret = src_table->open(src);
        if (ret == 0) {
            return src;
        }
    }
    media_src_close(src);
    return NULL;
}

int media_src_set_event_callback(media_src_t *src, media_src_event_func cb, void *ctx)
{
    src->cb = cb;
    src->ctx = ctx;
    return 0;
}

#if 0
int media_src_set_pcm_output(media_src_t* src, media_src_pcm_output pcm_output, void* ctx) {
    src->pcm_output = pcm_output;
    src->pcm_ctx = ctx;
    return 0;
}
#endif

int media_src_close(media_src_t *src)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->close) {
            ret = src_table->close(src);
        }
        free(src);
    }
    return ret;
}

int media_src_reload(media_src_t *src)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->reload) {
            ret = src_table->reload(src);
        }
    }
    return ret;
}

int media_src_connect(media_src_t *src, char *uri)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->connect) {
            ret = src_table->connect(src, uri);
        }
    }
    return ret;
}

int media_src_disconnect(media_src_t *src)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->disconnect) {
            ret = src_table->disconnect(src);
        }
    }
    return ret;
}

int media_src_read(media_src_t *src, void *data, size_t len)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->read) {
            ret = src_table->read(src, data, len);
        }
    }
    return ret;
}

int media_src_seek(media_src_t *src, uint64_t position)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->seek) {
            ret = src_table->seek(src, position);
        }
    }
    return ret;
}

int media_src_get_size(media_src_t *src, uint64_t *size)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->file_size) {
            ret = src_table->file_size(src, size);
        }
    }
    return ret;
}

int media_src_get_position(media_src_t *src, uint64_t *position)
{
    int ret = -1;
    if (src) {
        media_src_func_t *src_table = get_src_table(src);
        if (src_table && src_table->get_pos) {
            ret = src_table->get_pos(src, position);
        }
    }
    return ret;
}

media_src_type_t media_src_get_src_type(media_src_t *src)
{
    return src->src_type;
}
