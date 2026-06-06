#ifndef FILE_BROWSER_ROW_H
#define FILE_BROWSER_ROW_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// File item metadata (one row)
// ============================================================================

typedef enum {
    FILE_TYPE_FOLDER,
    FILE_TYPE_AUDIO,
    FILE_TYPE_IMAGE,
    FILE_TYPE_VIDEO,
    FILE_TYPE_DOCUMENT,
    FILE_TYPE_COMPRESSED,
    FILE_TYPE_EXECUTABLE,
    FILE_TYPE_UNSUPPORTED,
} file_type_t;

typedef struct {
    char name[256];          // Filename (truncated on display)
    file_type_t type;
    uint32_t size_bytes;     // Raw size for formatting
    uint32_t date_unix;      // Unix timestamp for formatting
    char full_path[512];     // For reference, not displayed
    bool is_accessible;      // Disabled if unsupported or permission denied
} file_item_t;

// ============================================================================
// Row object (visual container for one file/folder)
// ============================================================================

/**
 * @brief Create a file browser row object
 * Returns a container with icon + name + size + date labels
 */
lv_obj_t *file_browser_row_create(lv_obj_t *parent, const file_item_t *item);

/**
 * @brief Update row visual state based on focus/select/disable
 */
void file_browser_row_set_focus(lv_obj_t *row, bool focused);
void file_browser_row_set_selected(lv_obj_t *row, bool selected);
void file_browser_row_set_disabled(lv_obj_t *row, bool disabled);

/**
 * @brief Get file item metadata from row object
 */
const file_item_t *file_browser_row_get_item(lv_obj_t *row);

/**
 * @brief Format size to human-readable string
 */
void file_browser_format_size(uint32_t size_bytes, char *buf, size_t buf_size);

/**
 * @brief Format date to YYYY-MM-DD HH:mm string
 */
void file_browser_format_date(uint32_t unix_time, char *buf, size_t buf_size);

/**
 * @brief Get icon ID for file type (maps to icon bitmap index)
 */
uint16_t file_browser_get_icon_id(file_type_t type, bool is_accessible);

#ifdef __cplusplus
}
#endif

#endif // FILE_BROWSER_ROW_H
