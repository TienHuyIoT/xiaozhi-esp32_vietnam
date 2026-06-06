/**
 * @file file_browser_integration_example.cc
 * @brief Example of integrating File Browser module into main application
 * 
 * This file shows:
 * 1. How to initialize the file browser module
 * 2. How to create and load the file browser screen
 * 3. How to monitor async folder scan events
 * 4. How to handle state transitions
 */

#include "features/file_browser/file_browser.h"
#include "esp_log.h"

static const char *TAG = "FILE_BROWSER_INTEGRATION";

// Global reference to file browser screen
static lv_obj_t *g_file_browser_screen = NULL;

// ============================================================================
// Example: Initialize File Browser Module
// ============================================================================

void example_file_browser_init(void) {
    ESP_LOGI(TAG, "Initializing file browser module");
    
    // Initialize module (encoder keymap + scan task)
    file_browser_init();
    
    // Create file browser screen
    g_file_browser_screen = file_browser_create_screen();
    
    ESP_LOGI(TAG, "File browser module ready");
}

// ============================================================================
// Example: Load File Browser Screen
// ============================================================================

void example_show_file_browser(const char *folder_path) {
    ESP_LOGI(TAG, "Showing file browser for: %s", folder_path);
    
    if (!g_file_browser_screen) {
        ESP_LOGE(TAG, "File browser screen not initialized");
        return;
    }
    
    // Load screen
    lv_scr_load(g_file_browser_screen);
    
    // Open folder (triggers async scan)
    file_browser_open_folder(folder_path);
}

// ============================================================================
// Example: Monitor Folder Scan Progress
// (Call this from a timer or UI update task, ~100ms interval)
// ============================================================================

void example_file_browser_poll_scan(void) {
    QueueHandle_t scan_queue = file_browser_scan_get_event_queue();
    file_scan_event_t evt;
    
    // Non-blocking check for scan events
    if (xQueueReceive(scan_queue, &evt, 0) != pdTRUE) {
        return;  // No event pending
    }
    
    switch (evt.type) {
        case SCAN_EVT_PROGRESS:
            // Intermediate progress
            ESP_LOGD(TAG, "Scan progress: %lu files loaded", evt.loaded_files);
            // TODO: Update UI spinner or progress bar
            break;
        
        case SCAN_EVT_COMPLETE:
            ESP_LOGI(TAG, "Scan complete: %lu total files", evt.total_files);
            
            // Get scan results
            uint32_t count;
            const file_item_t *files = file_browser_scan_get_results(&count);
            
            // TODO: Populate UI list with results
            // for (uint32_t i = 0; i < count && i < 5; i++) {
            //     lv_obj_t *row = file_browser_row_create(list, &files[i]);
            //     file_browser_row_set_focus(row, (i == 0));
            // }
            
            // Transition state machine from LOADING to BROWSING
            file_browser_state_loading_exit();
            break;
        
        case SCAN_EVT_ERROR:
            ESP_LOGE(TAG, "Scan error: %s (code=%d)", evt.error_msg, evt.error_code);
            // TODO: Show error message in UI
            // TODO: Offer Retry button or go back
            break;
    }
}

// ============================================================================
// Example: Monitor File Browser State Changes
// ============================================================================

void example_file_browser_poll_state(void) {
    static file_browser_state_t last_state = FB_STATE_BROWSING;
    file_browser_state_t current_state = file_browser_get_state();
    
    // Only log state changes
    if (current_state == last_state) {
        return;
    }
    
    last_state = current_state;
    
    switch (current_state) {
        case FB_STATE_BROWSING:
            ESP_LOGI(TAG, "State: BROWSING");
            // List is active, normal interaction enabled
            break;
        
        case FB_STATE_MENU_OPEN:
            ESP_LOGI(TAG, "State: MENU_OPEN");
            // Context menu is displayed and focused
            break;
        
        case FB_STATE_CONFIRM_DELETE:
            ESP_LOGI(TAG, "State: CONFIRM_DELETE");
            // Confirm dialog is active
            break;
        
        case FB_STATE_LOADING:
            ESP_LOGI(TAG, "State: LOADING");
            // Async folder scan in progress
            break;
        
        case FB_STATE_ERROR:
            ESP_LOGI(TAG, "State: ERROR");
            // I/O error occurred
            break;
        
        default:
            break;
    }
}

// ============================================================================
// Example: Integration into Main Application Loop
// ============================================================================

/**
 * @brief Suggested integration point in application task/main loop
 * 
 * Call this function periodically (e.g., every 100 ms) to process
 * file browser events and state changes.
 */
void example_file_browser_update(void) {
    // Poll for scan events
    example_file_browser_poll_scan();
    
    // Poll for state changes
    example_file_browser_poll_state();
}

// ============================================================================
// Example: Main Application Integration
// ============================================================================

/**
 * @brief Example of calling file browser from app init
 */
void example_app_init(void) {
    // ... other app initialization ...
    
    // Initialize file browser module
    example_file_browser_init();
    
    // ... other app initialization ...
}

/**
 * @brief Example of launching file browser from menu
 */
void example_user_selected_file_browser(void) {
    // Show file browser, starting from SD card root
    example_show_file_browser("/sdcard");
}

/**
 * @brief Example app task that monitors file browser
 */
void example_app_task(void *arg) {
    // Initialize
    example_app_init();
    
    // Main loop
    while (1) {
        // Update file browser (poll for events)
        example_file_browser_update();
        
        // ... other app logic ...
        
        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
// Example: Custom Event Handlers
// ============================================================================

/**
 * @brief Example handler for when user selects a file
 * (To be called from file_browser when row is pressed in BROWSING state)
 */
void example_on_file_selected(const file_item_t *file) {
    if (!file) return;
    
    ESP_LOGI(TAG, "User selected: %s (type=%d, size=%u)", 
             file->name, file->type, file->size_bytes);
    
    // Handle based on file type
    switch (file->type) {
        case FILE_TYPE_FOLDER:
            // Open folder
            example_show_file_browser(file->full_path);
            break;
        
        case FILE_TYPE_AUDIO:
            // Play audio
            ESP_LOGI(TAG, "Playing audio: %s", file->full_path);
            // TODO: Trigger music player
            break;
        
        case FILE_TYPE_IMAGE:
            // Show image
            ESP_LOGI(TAG, "Displaying image: %s", file->full_path);
            // TODO: Trigger image viewer
            break;
        
        case FILE_TYPE_VIDEO:
            // Play video
            ESP_LOGI(TAG, "Playing video: %s", file->full_path);
            // TODO: Trigger video player
            break;
        
        default:
            ESP_LOGW(TAG, "Unsupported file type: %d", file->type);
            break;
    }
}

/**
 * @brief Example handler for delete action (from menu)
 * (To be called from file_browser when user confirms delete)
 */
void example_on_file_delete(const file_item_t *file) {
    if (!file) return;
    
    ESP_LOGI(TAG, "Deleting file: %s", file->full_path);
    
    // TODO: Perform actual file deletion
    // int result = remove(file->full_path);
    // if (result == 0) {
    //     ESP_LOGI(TAG, "File deleted successfully");
    //     // Refresh folder view
    // } else {
    //     ESP_LOGE(TAG, "Failed to delete file: %s", strerror(errno));
    // }
}

// ============================================================================
// Notes for Integration
// ============================================================================

/*
 * 1. ENCODER DRIVER INTEGRATION:
 *    - Ensure your rotary_encoder driver is initialized before file_browser_init()
 *    - Replace encoder_val = 0 in file_browser_keymap.c:file_browser_encoder_read()
 *      with actual encoder API call:
 *      encoder_val = rotary_encoder_get_counter(encoder_handle);
 *
 * 2. BUTTON / BACK KEY INTEGRATION:
 *    - Connect back button GPIO to file_browser_encoder_button_handler()
 *    - Set up periodic timer to call file_browser_encoder_long_press_check()
 *      every 50 ms to detect long-press
 *
 * 3. FOCUS GROUP:
 *    - Ensure LVGL has a default group created before file_browser_init():
 *      lv_group_t *g = lv_group_create();
 *      lv_group_set_default(g);
 *
 * 4. EVENT QUEUE POLLING:
 *    - Call example_file_browser_poll_scan() periodically (~100 ms)
 *      from a timer, task, or main loop to process async scan events
 *
 * 5. STATE TRANSITIONS:
 *    - Use file_browser_get_state() to check current state
 *    - State machine handles most transitions automatically
 *    - Modal dialogs (menu, confirm) manage focus group internally
 *
 * 6. TOUCH INTEGRATION:
 *    - File browser screen is LVGL-based, supports touch natively
 *    - Touch events are handled by LVGL's event system
 *    - Ensure lcd_touch driver is properly initialized for your board
 *
 * 7. MEMORY CONSIDERATIONS:
 *    - Folder scan allocates file_item_t array (256 bytes per item)
 *    - Initial capacity: 100 files; doubles on overflow
 *    - Total RAM for 100 files: ~25 KiB (acceptable for ESP32-S3)
 *    - Adjust SCAN_BATCH_SIZE and SCAN_YIELD_MS if needed
 */
