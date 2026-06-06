#ifndef FILE_BROWSER_STATE_MACHINE_H
#define FILE_BROWSER_STATE_MACHINE_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// File Browser State Machine
// ============================================================================

typedef enum {
    FB_STATE_BROWSING,        // Normal list browsing, no modal
    FB_STATE_MENU_OPEN,       // Context menu is displayed and focused
    FB_STATE_CONFIRM_DELETE,  // Confirm delete dialog active
    FB_STATE_CONFIRM_MOVE,    // Confirm move/copy dialog active
    FB_STATE_LOADING,         // Folder scan in progress
    FB_STATE_ERROR,           // I/O error; retry mode
} file_browser_state_t;

typedef enum {
    FB_ACTION_NONE = 0,
    FB_ACTION_OPEN_FOLDER,
    FB_ACTION_SELECT_FILE,
    FB_ACTION_RENAME,
    FB_ACTION_COPY,
    FB_ACTION_MOVE,
    FB_ACTION_DELETE,
    FB_ACTION_PROPERTIES,
} file_browser_action_t;

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *list;
    lv_obj_t *menu_panel;
    lv_obj_t *confirm_dialog;
    file_browser_state_t current_state;
    file_browser_action_t pending_action;
    int32_t focused_row_index;
    int32_t focused_row_before_modal;  // Save row focus when menu opens
} file_browser_context_t;

extern file_browser_context_t g_fb_ctx;

// ============================================================================
// State machine transitions
// ============================================================================

/**
 * @brief Transition to BROWSING state
 * Setup: List group is active, menu hidden, interactions enabled
 */
void file_browser_state_browsing_enter(void);

/**
 * @brief Transition to MENU_OPEN state
 * Setup: Menu group is active, list frozen, focused row saved
 */
void file_browser_state_menu_open_enter(void);

/**
 * @brief Transition back to BROWSING from MENU_OPEN
 * Setup: Menu hidden, list focus restored, list group active
 */
void file_browser_state_menu_open_exit(void);

/**
 * @brief Transition to CONFIRM_DELETE state
 * Setup: Confirm dialog group active, list/menu frozen
 */
void file_browser_state_confirm_delete_enter(void);

/**
 * @brief Handle confirm dialog result
 */
void file_browser_state_confirm_delete_result(bool confirmed);

/**
 * @brief Transition to LOADING state
 */
void file_browser_state_loading_enter(void);

/**
 * @brief Transition from LOADING to BROWSING after scan complete
 */
void file_browser_state_loading_exit(void);

/**
 * @brief Global event handler for state transitions
 */
void file_browser_state_handle_event(int event_type, void *event_data);

/**
 * @brief Get current state
 */
file_browser_state_t file_browser_state_get(void);

#ifdef __cplusplus
}
#endif

#endif // FILE_BROWSER_STATE_MACHINE_H
