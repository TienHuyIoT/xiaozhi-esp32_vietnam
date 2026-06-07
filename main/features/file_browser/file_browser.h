#ifndef FILE_BROWSER_H
#define FILE_BROWSER_H

#include "lvgl.h"
#include "screens/file_browser_state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

void file_browser_init(void);
lv_obj_t* file_browser_create_screen(void);
void file_browser_open_folder(const char* folder_path);
file_browser_state_t file_browser_get_state(void);
void file_browser_close(void);

/**
 * @brief Register exit callback for file browser UI.
 *
 * Callback is called when user requests exit from file browser (e.g. Exit button).
 */
void file_browser_set_exit_callback(void (*callback)(void));

/**
 * @brief Show or hide the Exit button on file browser header.
 */
void file_browser_show_exit_button(bool show);

#ifdef __cplusplus
}
#endif

#endif  // FILE_BROWSER_H