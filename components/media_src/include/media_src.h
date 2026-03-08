/**
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * See LICENSE file for details.
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief        Media source type
 */
typedef enum {
    MEDIA_SRC_TYPE_NONE,     /*!< None source */
    MEDIA_SRC_TYPE_STORAGE,  /*!< Local storage source type */
    MEDIA_SRC_TYPE_NETWORK,  /*!< Network source type */
    MEDIA_SRC_TYPE_MEMORY,   /*!< Memory source type */
    MEDIA_SRC_TYPE_BT,       /*!< Bluetooth source type */
    MEDIA_SRC_TYPE_RTMP,     /*!< RTMP source type */
    MEDIA_SRC_TYPE_USER,     /*!< Customized source type */
    MEDIA_SRC_TYPE_MAX
} media_src_type_t;

/**
 * @brief        Media source setting
 */
typedef enum {
    MEDIA_SRC_SET_TYPE_COOKIE,
    MEDIA_SRC_SET_TYPE_STATIC_BUFFER,
} media_src_set_type_t;

/**
 * @brief        Media source event
 */
typedef enum {
    MEDIA_SRC_EVENT_CONNECTED,
    MEDIA_SRC_EVENT_DISCONNECTED,
    MEDIA_SRC_EVENT_EOS,
} media_src_event_t;

typedef int (*media_src_event_func)(media_src_event_t src_event, void *ctx);

/**
 * @brief        Media source structure
 */
typedef struct {
    media_src_type_t     src_type;  /*!< Source type */
    media_src_event_func cb;        /*!< Callback */
    void                *ctx;       /*!< Callback context */
    void                *sub_src;   /*!< Sub source to keep media source extra data */
} media_src_t;

/**
 * @brief        Media source interface
 */
typedef int (*media_src_open_func)(media_src_t *src);
typedef int (*media_src_connect_func)(media_src_t *src, char *uri);
typedef int (*media_src_disconnect_func)(media_src_t *src);
typedef int (*media_src_read_func)(media_src_t *src, void *data, size_t len);
typedef int (*media_src_seek_func)(media_src_t *src, uint64_t position);
typedef int (*media_src_get_size_func)(media_src_t *src, uint64_t *size);
typedef int (*media_src_get_position_func)(media_src_t *src, uint64_t *position);
typedef int (*media_src_set_info)(media_src_t *src, media_src_set_type_t set_type, void *data, size_t len);
typedef int (*media_src_reload_func)(media_src_t *src);
typedef int (*media_src_close_func)(media_src_t *src);

/**
 * @brief        Media source instance functions
 */
typedef struct {
    media_src_open_func         open;       /*!< Open source callback */
    media_src_connect_func      connect;    /*!< Connect source callback */
    media_src_disconnect_func   disconnect; /*!< Disconnect source callback */
    media_src_read_func         read;       /*!< Read from source callback */
    media_src_seek_func         seek;       /*!< Seek source callback */
    media_src_get_size_func     file_size;  /*!< Get file size callback */
    media_src_get_position_func get_pos;    /*!< Get current read position callback */
    media_src_set_info          set;        /*!< Source setting callback */
    media_src_reload_func       reload;     /*!< Reload source callback */
    media_src_close_func        close;      /*!< Close source callback*/
} media_src_func_t;

/**
 * @brief        Register media source
 *
 * @param        src_type: Source type
 * @param        src_table: Media source realization function table
 *
 * @return
 *               - 0: Success
 *               - Others: Fail to register
 */
int media_src_register(media_src_type_t src_type, media_src_func_t *src_table);

/**
 * @brief        Open media source
 *
 * @param        src_type: Source type
 *
 * @return
 *               - 0: Success to open
 *               - Others: Such source type not registered yet
 */
media_src_t *media_src_open(media_src_type_t src_type);

/**
 * @brief        Set event callback
 *
 * @param        src: Media source instance
 * @param        cb: Event callback function
 * @param        ctx: Event callback context
 *
 * @return
 *               - 0: Success to set event callback
 *               - Others: Fail to set callback
 */
int media_src_set_event_callback(media_src_t *src, media_src_event_func cb, void *ctx);

/**
 * @brief        Connect media source
 *
 * @param        src: Media source instance
 * @param        url: Data source URI
 *
 * @return
 *               - 0: Success to connect
 *               - Others: Fail to connect
 */
int media_src_connect(media_src_t *src, char *uri);

/**
 * @brief        Reload of media source
 *
 * @note         Keep some internal resource and reload same url
 *
 * @param        src: Media source instance
 *
 * @return
 *               - 0: Success to reload
 *               - Others: Fail to reload
 */
int media_src_reload(media_src_t *src);

/**
 * @brief        Read data from media source
 *
 * @param        src: Media source instance
 * @param        data: Data to store read data
 * @param        size: Data size to read
 *
 * @return
 *               - 0: Success to seek
 *               - Others: Fail to seek
 */
int media_src_read(media_src_t *src, void *data, size_t size);

/**
 * @brief        Seek to byte position
 *
 * @param        src: Media source instance
 * @param        position: Position to seek
 *
 * @return
 *               - 0: Success to seek
 *               - Others: Fail to seek
 */
int media_src_seek(media_src_t *src, uint64_t position);

/**
 * @brief        Get file size of current source
 *
 * @param        src: Media source instance
 * @param        size: File size
 *
 * @return
 *               - 0: Success to get
 *               - Others: Fail to get
 */
int media_src_get_size(media_src_t *src, uint64_t *size);

/**
 * @brief        Get byte position of media source
 *
 * @param        src: Media source instance
 * @param        position: Current position
 *
 * @return
 *               - 0: Success to get
 *               - Others: Fail to get
 */
int media_src_get_position(media_src_t *src, uint64_t *position);

/**
 * @brief        Get source type
 *
 * @param        src: Media source instance
 *
 * @return       Source type of current media source
 */
media_src_type_t media_src_get_src_type(media_src_t *src);

/**
 * @brief        Disconnect media source
 *
 * @param        src: Media source instance
 *
 * @return
 *               - 0: Success to disconnect
 *               - Others: Fail to disconnect
 */
int media_src_disconnect(media_src_t *src);

/**
 * @brief        Close media source
 *
 * @param        src: Media source instance
 *
 * @return
 *               - 0: Success to close
 *               - Others: Fail to close
 */
int media_src_close(media_src_t *src);

#ifdef __cplusplus
}
#endif
