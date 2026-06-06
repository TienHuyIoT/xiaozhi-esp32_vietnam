#ifndef FILE_BROWSER_SCAN_H
#define FILE_BROWSER_SCAN_H

#include "file_browser_row.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Async folder scan task
// Enumerates files on SD card without blocking UI
// ============================================================================

#define FILE_BROWSER_SCAN_BATCH_SIZE    10  // Files per yield
#define FILE_BROWSER_SCAN_YIELD_MS      10  // ms between batches

typedef enum {
    SCAN_EVT_PROGRESS,    // Intermediate batch loaded
    SCAN_EVT_COMPLETE,    // All files loaded
    SCAN_EVT_ERROR,       // I/O or other error
} file_scan_event_type_t;

typedef struct {
    file_scan_event_type_t type;
    uint32_t total_files;
    uint32_t loaded_files;
    int error_code;  // 0 if no error
    char error_msg[128];
} file_scan_event_t;

/**
 * @brief Initialize folder scan task
 * Must be called once during app init
 */
void file_browser_scan_init(void);

/**
 * @brief Start async scan of folder at path
 * Posts events to g_scan_event_queue
 */
void file_browser_scan_folder(const char *folder_path);

/**
 * @brief Get scan result (file list)
 */
const file_item_t *file_browser_scan_get_results(uint32_t *out_count);

/**
 * @brief Cancel ongoing scan
 */
void file_browser_scan_cancel(void);

/**
 * @brief Get event queue for polling/listening to scan progress
 */
QueueHandle_t file_browser_scan_get_event_queue(void);

#ifdef __cplusplus
}
#endif

#endif // FILE_BROWSER_SCAN_H
