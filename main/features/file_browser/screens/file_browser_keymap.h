#ifndef FILE_BROWSER_KEYMAP_H
#define FILE_BROWSER_KEYMAP_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Encoder Input Device Handler
// Maps physical encoder rotation/press to LVGL key events
// ============================================================================

typedef struct {
    int32_t last_encoder_val;
    uint32_t press_start_time;
    bool is_long_press_triggered;
} encoder_state_t;

extern encoder_state_t g_encoder_state;

// Keymap definitions
#define ENCODER_SHORT_PRESS_THRESHOLD_MS    350
#define ENCODER_LONG_PRESS_THRESHOLD_MS     450
#define ENCODER_VERY_LONG_PRESS_MS          1000
#define ENCODER_DEBOUNCE_MS                 20

// Custom app events (beyond LV_KEY_*)
enum {
    APP_EVT_CTX_MENU_OPEN = LV_EVENT_LAST + 1,
    APP_EVT_CTX_MENU_CLOSE,
    APP_EVT_FOLDER_CHANGED,
};

// ============================================================================
// Function declarations
// ============================================================================

/**
 * @brief Initialize encoder input device driver for file browser
 * Call once during app init
 */
void file_browser_keymap_init(void);

/**
 * @brief Read encoder rotation and press state
 * Called by LVGL input device task (typically 20 ms tick)
 * 
 * @param indev Pointer to input device
 * @param data Output structure filled with key events
 */
void file_browser_encoder_read(lv_indev_t *indev, lv_indev_data_t *data);

/**
 * @brief Handle encoder rotation event
 * Translates encoder delta to LV_KEY_NEXT or LV_KEY_PREV
 */
void file_browser_encoder_rotation_handler(int32_t delta);

/**
 * @brief Handle encoder button press/release
 * Detects short vs long press; posts APP_EVT_CTX_MENU_OPEN if long
 */
void file_browser_encoder_button_handler(bool pressed);

/**
 * @brief Periodic check for long-press threshold
 */
void file_browser_encoder_long_press_check(void);

/**
 * @brief Get current encoder press duration (ms)
 * Useful for detecting long-press without state machine
 */
uint32_t file_browser_get_encoder_press_duration(void);

/**
 * @brief Cancel pending long-press detection (e.g., on modal close)
 */
void file_browser_encoder_press_reset(void);

#ifdef __cplusplus
}
#endif

#endif // FILE_BROWSER_KEYMAP_H
