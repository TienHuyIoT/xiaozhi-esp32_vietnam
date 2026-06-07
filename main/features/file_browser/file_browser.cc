#include "file_browser.h"

#include <functional>
#include <stdio.h>
#include <string.h>

#include "screens/file_browser_keymap.h"
#include "screens/file_browser_row.h"
#include "screens/file_browser_scan.h"
#include "screens/file_browser_state_machine.h"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
}

namespace {
constexpr const char* kTag = "FILE_BROWSER";
constexpr const char* kRootPath = "/sdcard";
constexpr int kScreenW = 320;
constexpr int kScreenH = 240;
constexpr int kHeaderH = 40;
constexpr int kListH = 160;
constexpr int kFooterH = 40;
constexpr int kMenuItemCount = 5;

class FileBrowser final {
 public:
  /**
   * @brief Singleton accessor for FileBrowser controller.
   */
  static FileBrowser& Instance() {
    static FileBrowser instance;
    return instance;
  }

  /**
   * @brief Initialize file-browser dependencies (keymap + scanner).
   */
  void Init() {
    ESP_LOGI(kTag, "Initializing file browser module");
    file_browser_keymap_init();
    file_browser_scan_init();
    ESP_LOGI(kTag, "File browser module initialized successfully");
  }

  /**
   * @brief Build and return file-browser screen.
   *
   * Creates the screen once, wires all UI callbacks, and starts opening root folder.
   */
  lv_obj_t* CreateScreen() {
    if (g_fb_ctx.screen != nullptr) {
      return g_fb_ctx.screen;
    }

    // Touch hitbox map (absolute screen coordinates, 320x240):
    // 1) Header container: x1=0, x2=319, y1=0, y2=39, pad_hor=8, pad_ver=4.
    //    Header content box: x1=8, x2=311, y1=4, y2=35 (w=304, h=32).
    //    With SPACE_BETWEEN and child widths [36, 180, 36]: gap=26 px.
    //    - Back button expected:    x1=8,   x2=43,  y1=4, y2=35
    //    - Path label expected:     x1=70,  x2=249, y1=4, y2=35 (non-clickable)
    //    - Refresh button expected: x1=276, x2=311, y1=4, y2=35
    // 2) List container: x1=0, x2=319, y1=40, y2=199, pad_all=2.
    //    List content box: x1=2, x2=317, y1=42, y2=197.
    //    Row i (h=32) expected: y1=42+i*32, y2=73+i*32.
    // 3) Footer container: x1=0, x2=319, y1=200, y2=239 (labels only).
    // 4) Menu panel (visible): x1=60, x2=259, y1=40, y2=199, pad_all=6.
    // 5) Confirm dialog (visible): x1=60, x2=259, y1=72, y2=167, pad_all=8.

    lv_obj_t* screen = lv_obj_create(nullptr);
    lv_obj_set_size(screen, kScreenW, kScreenH);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    lv_obj_set_style_pad_all(screen, 0, 0);

    lv_obj_t* header = lv_obj_create(screen);
    lv_obj_set_size(header, kScreenW, kHeaderH);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x222222), 0);
    lv_obj_set_style_pad_hor(header, 8, 0);
    lv_obj_set_style_pad_ver(header, 4, 0);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    back_btn_ = lv_btn_create(header);
    // Back button local hitbox in header container: pos=(0,0), size=(36x32)
    // Expected absolute hitbox: x1=8, x2=43, y1=4, y2=35
    lv_obj_set_size(back_btn_, 36, 32);
    lv_obj_t* back_lbl = lv_label_create(back_btn_);
    lv_label_set_text(back_lbl, "<");
    lv_obj_center(back_lbl);

    path_label_ = lv_label_create(header);
    lv_label_set_text(path_label_, kRootPath);
    lv_obj_set_width(path_label_, 180);
    lv_obj_set_style_text_color(path_label_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(path_label_, LV_TEXT_ALIGN_CENTER, 0);

    refresh_btn_ = lv_btn_create(header);
    // Refresh button local hitbox in header container: after back+label in row flex,
    // size=(36x32), expected absolute hitbox: x1=276, x2=311, y1=4, y2=35.
    lv_obj_set_size(refresh_btn_, 36, 32);
    lv_obj_t* refresh_lbl = lv_label_create(refresh_btn_);
    lv_label_set_text(refresh_lbl, "R");
    lv_obj_center(refresh_lbl);

    exit_btn_ = lv_btn_create(header);
    // Exit button: hide by default, shown only when needed
    // size=(36x32), position after refresh button
    lv_obj_set_size(exit_btn_, 36, 32);
    lv_obj_t* exit_lbl = lv_label_create(exit_btn_);
    lv_label_set_text(exit_lbl, "X");
    lv_obj_center(exit_lbl);
    lv_obj_add_flag(exit_btn_, LV_OBJ_FLAG_HIDDEN);

    list_obj_ = lv_obj_create(screen);
    lv_obj_set_size(list_obj_, kScreenW, kListH);
    lv_obj_set_pos(list_obj_, 0, kHeaderH);
    lv_obj_set_style_bg_color(list_obj_, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_width(list_obj_, 0, 0);
    lv_obj_set_style_pad_all(list_obj_, 2, 0);
    lv_obj_set_flex_flow(list_obj_, LV_FLEX_FLOW_COLUMN);

    lv_obj_t* footer = lv_obj_create(screen);
    lv_obj_set_size(footer, kScreenW, kFooterH);
    lv_obj_set_pos(footer, 0, kHeaderH + kListH);
    lv_obj_set_style_bg_color(footer, lv_color_hex(0x222222), 0);
    lv_obj_set_style_pad_hor(footer, 8, 0);
    lv_obj_set_style_pad_ver(footer, 4, 0);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    count_label_ = lv_label_create(footer);
    lv_label_set_text(count_label_, "0/0");
    lv_obj_set_style_text_color(count_label_, lv_color_hex(0xDDDDDD), 0);

    hint_label_ = lv_label_create(footer);
    lv_label_set_text(hint_label_, "ENC: move/ok, hold: menu");
    lv_obj_set_width(hint_label_, 200);
    lv_obj_set_style_text_color(hint_label_, lv_color_hex(0xBBBBBB), 0);
    lv_obj_set_style_text_align(hint_label_, LV_TEXT_ALIGN_CENTER, 0);

    menu_panel_ = lv_obj_create(screen);
    // Context menu panel absolute hitbox: x1=60, x2=259, y1=40, y2=199
    lv_obj_set_size(menu_panel_, 200, 160);
    lv_obj_set_pos(menu_panel_, 60, 40);
    lv_obj_set_style_bg_color(menu_panel_, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_border_color(menu_panel_, lv_color_hex(0x00CCFF), 0);
    lv_obj_set_style_border_width(menu_panel_, 2, 0);
    lv_obj_set_style_pad_all(menu_panel_, 6, 0);
    lv_obj_set_flex_flow(menu_panel_, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(menu_panel_, LV_OBJ_FLAG_HIDDEN);

    const char* menu_items[] = {"Open", "Rename", "Copy", "Delete", "Properties"};
    for (int i = 0; i < kMenuItemCount; ++i) {
      lv_obj_t* item_btn = lv_btn_create(menu_panel_);
      // Menu item hitbox (local to menu panel):
      // width=100% of menu content width (=200-2*6=188), height=26.
      // Menu panel abs content box: x1=66, x2=253, y1=46, y2=193.
      // So each item x expected: x1=66, x2=253.
      // For index i (0..4): y1~=46+i*26, y2~=71+i*26 (+ LVGL row-gap if configured).
      lv_obj_set_width(item_btn, lv_pct(100));
      lv_obj_set_height(item_btn, 26);
      lv_obj_t* item_lbl = lv_label_create(item_btn);
      lv_label_set_text(item_lbl, menu_items[i]);
      lv_obj_center(item_lbl);
      menu_buttons_[i] = item_btn;
      lv_obj_add_event_cb(item_btn, OnMenuItemClicked, LV_EVENT_CLICKED, this);
      lv_obj_add_event_cb(item_btn, OnDebugInputEvent, LV_EVENT_ALL, this);
    }

    confirm_dialog_ = lv_obj_create(screen);
    // Confirm dialog absolute hitbox: x1=60, x2=259, y1=72, y2=167
    lv_obj_set_size(confirm_dialog_, 200, 96);
    lv_obj_set_pos(confirm_dialog_, 60, 72);
    lv_obj_set_style_bg_color(confirm_dialog_, lv_color_hex(0x1F1F1F), 0);
    lv_obj_set_style_border_color(confirm_dialog_, lv_color_hex(0xCC5555), 0);
    lv_obj_set_style_border_width(confirm_dialog_, 2, 0);
    lv_obj_set_style_pad_all(confirm_dialog_, 8, 0);
    lv_obj_set_flex_flow(confirm_dialog_, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(confirm_dialog_, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t* confirm_lbl = lv_label_create(confirm_dialog_);
    lv_label_set_text(confirm_lbl, "Delete selected item?");

    lv_obj_t* btn_row = lv_obj_create(confirm_dialog_);
    // Button row local to confirm dialog: pos by flex flow, size=(100% x 36)
    // Confirm dialog content box abs: x1=68, x2=251, y1=80, y2=159.
    // btn_row width=184 (100% content), height=36.
    // btn_row x1=68, x2=251; y1/y2 depend on confirm_lbl height (font-dependent).
    // Use runtime logs to confirm final y1/y2.
    lv_obj_set_width(btn_row, lv_pct(100));
    lv_obj_set_height(btn_row, 36);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_style_border_width(btn_row, 0, 0);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    cancel_btn_ = lv_btn_create(btn_row);
    // Cancel button local hitbox: size=(76x30), left side by SPACE_AROUND.
    // With row width=184 and 2 buttons:
    // free=184-(76+76)=32, SPACE_AROUND slot=8 => cancel x(local): x1~=8, x2~=83.
    // absolute x expected: x1~=76, x2~=151; y1/y2 follow btn_row y and h=30.
    lv_obj_set_size(cancel_btn_, 76, 30);
    lv_obj_t* cancel_lbl = lv_label_create(cancel_btn_);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);

    delete_btn_ = lv_btn_create(btn_row);
    // Delete button local hitbox: size=(76x30), right side by SPACE_AROUND.
    // expected local x: x1~=100, x2~=175.
    // absolute x expected: x1~=168, x2~=243; y1/y2 follow btn_row y and h=30.
    lv_obj_set_size(delete_btn_, 76, 30);
    lv_obj_t* delete_lbl = lv_label_create(delete_btn_);
    lv_label_set_text(delete_lbl, "Delete");
    lv_obj_center(delete_lbl);

    lv_obj_add_event_cb(back_btn_, OnBackClicked, LV_EVENT_CLICKED, this);
    lv_obj_add_event_cb(refresh_btn_, OnRefreshClicked, LV_EVENT_CLICKED, this);
    lv_obj_add_event_cb(exit_btn_, OnExitClicked, LV_EVENT_CLICKED, this);
    lv_obj_add_event_cb(cancel_btn_, OnCancelDeleteClicked, LV_EVENT_CLICKED, this);
    lv_obj_add_event_cb(delete_btn_, OnConfirmDeleteClicked, LV_EVENT_CLICKED, this);
    lv_obj_add_event_cb(back_btn_, OnDebugInputEvent, LV_EVENT_ALL, this);
    lv_obj_add_event_cb(refresh_btn_, OnDebugInputEvent, LV_EVENT_ALL, this);
    lv_obj_add_event_cb(exit_btn_, OnDebugInputEvent, LV_EVENT_ALL, this);
    lv_obj_add_event_cb(cancel_btn_, OnDebugInputEvent, LV_EVENT_ALL, this);
    lv_obj_add_event_cb(delete_btn_, OnDebugInputEvent, LV_EVENT_ALL, this);
    lv_obj_add_event_cb(list_obj_, OnDebugInputEvent, LV_EVENT_ALL, this);
    lv_obj_add_event_cb(menu_panel_, OnDebugInputEvent, LV_EVENT_ALL, this);
    lv_obj_add_event_cb(confirm_dialog_, OnDebugInputEvent, LV_EVENT_ALL, this);

    g_fb_ctx.screen = screen;
    g_fb_ctx.list = list_obj_;
    g_fb_ctx.menu_panel = menu_panel_;
    g_fb_ctx.confirm_dialog = confirm_dialog_;
    g_fb_ctx.focused_row_index = 0;
    g_fb_ctx.focused_row_before_modal = 0;
    snprintf(current_path_, sizeof(current_path_), "%s", kRootPath);
    selected_item_ = nullptr;
    file_browser_state_browsing_enter();

    if (scan_timer_ == nullptr) {
      scan_timer_ = lv_timer_create(OnScanTimer, 120, this);
    }

    OpenFolder(current_path_);

    return g_fb_ctx.screen;
  }

  /**
   * @brief Request opening a folder path and trigger async scan.
   */
  void OpenFolder(const char* folder_path) {
    if (folder_path == nullptr) {
      return;
    }
    ESP_LOGI(kTag, "Opening folder: %s", folder_path);
    snprintf(current_path_, sizeof(current_path_), "%s", folder_path);
    if (path_label_ != nullptr) {
      lv_label_set_text(path_label_, current_path_);
    }
    if (hint_label_ != nullptr) {
      lv_label_set_text(hint_label_, "Loading...");
    }
    file_browser_state_loading_enter();
    file_browser_scan_folder(folder_path);
  }

  /**
   * @brief Close file browser state (UI cleanup only).
   */
  void Close() {
    // Hide exit button when closing
    if (exit_btn_ != nullptr) {
      lv_obj_add_flag(exit_btn_, LV_OBJ_FLAG_HIDDEN);
    }
    ESP_LOGI(kTag, "File browser closed");
  }

  /**
   * @brief Get current file-browser state machine state.
   */
  file_browser_state_t GetState() const {
    return file_browser_state_get();
  }

  /**
   * @brief Set callback invoked when user exits file browser.
   */
  void SetExitCallback(std::function<void()> callback) {
    on_exit_callback_ = callback;
  }

  /**
   * @brief Show or hide Exit button in header.
   */
  void ShowExitButton(bool show) {
    if (exit_btn_ != nullptr) {
      if (show) {
        lv_obj_clear_flag(exit_btn_, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(exit_btn_, LV_OBJ_FLAG_HIDDEN);
      }
    }
  }

 private:
  /**
   * @brief Convert LVGL input event code to readable string for logs.
   */
    static const char* EventCodeToString(lv_event_code_t code) {
      switch (code) {
        case LV_EVENT_PRESSED:
          return "PRESSED";
        case LV_EVENT_PRESSING:
          return "PRESSING";
        case LV_EVENT_RELEASED:
          return "RELEASED";
        case LV_EVENT_CLICKED:
          return "CLICKED";
        case LV_EVENT_LONG_PRESSED:
          return "LONG_PRESSED";
        default:
          return "OTHER";
      }
    }

    /**
     * @brief Generic debug hook for touch/key events on target widgets.
     */
    static void OnDebugInputEvent(lv_event_t* e) {
      FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
      if (self == nullptr) {
        return;
      }

      lv_event_code_t code = lv_event_get_code(e);
      if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING &&
          code != LV_EVENT_RELEASED && code != LV_EVENT_CLICKED &&
          code != LV_EVENT_LONG_PRESSED) {
        return;
      }

      self->LogTouchTarget(lv_event_get_target_obj(e), code);
    }

    /**
     * @brief Log the target object and hitbox for input-debugging.
     */
    void LogTouchTarget(lv_obj_t* obj, lv_event_code_t code) {
      if (obj == nullptr) {
        return;
      }

      const char* name = "UNKNOWN";
      if (obj == back_btn_) {
        name = "BACK_BUTTON";
      } else if (obj == refresh_btn_) {
        name = "REFRESH_BUTTON";
      } else if (obj == cancel_btn_) {
        name = "CANCEL_BUTTON";
      } else if (obj == delete_btn_) {
        name = "DELETE_BUTTON";
      } else if (obj == menu_panel_) {
        name = "MENU_PANEL";
      } else if (obj == confirm_dialog_) {
        name = "CONFIRM_DIALOG";
      } else if (obj == list_obj_) {
        name = "FILE_LIST";
      } else {
        for (int i = 0; i < kMenuItemCount; ++i) {
          if (obj == menu_buttons_[i]) {
            name = "MENU_ITEM_BUTTON";
            break;
          }
        }

        if (name[0] == 'U' && lv_obj_get_parent(obj) == list_obj_) {
          name = "FILE_ROW";
        }
      }

      lv_area_t area;
      lv_obj_get_coords(obj, &area);
      ESP_LOGI(kTag,
               "Input evt=%s target=%s obj=%p hitbox=[%d,%d]-[%d,%d] size=%dx%d",
               EventCodeToString(code), name, obj, area.x1, area.y1, area.x2, area.y2,
               (area.x2 - area.x1 + 1), (area.y2 - area.y1 + 1));
    }

  /**
   * @brief Event handler for Back button click.
   */
  static void OnBackClicked(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGD(kTag, "Back button clicked");
      self->HandleBack();
    }
  }

  /**
   * @brief Event handler for Refresh button click.
   *
   * Reloads current folder content.
   */
  static void OnRefreshClicked(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGD(kTag, "Refresh button clicked");
      self->HandleRefresh();
    }
  }

  /**
   * @brief Event handler for Exit button click.
   *
   * Closes file browser and triggers registered exit callback.
   */
  static void OnExitClicked(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGI(kTag, "Exit button clicked");
      self->Close();
      if (self->on_exit_callback_) {
        self->on_exit_callback_();
      }
    }
  }

  /**
   * @brief Event handler for row click.
   */
  static void OnRowClicked(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGD(kTag, "Row clicked");
      self->HandleRowFocus(lv_event_get_target_obj(e));
    }
  }

  /**
   * @brief Event handler for row long-press to open context menu.
   */
  static void OnRowLongPressed(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGI(kTag, "Row long pressed -> open context menu");
      self->HandleRowFocus(lv_event_get_target_obj(e));
      self->ShowMenu();
    }
  }

  /**
   * @brief Event handler for opening context menu from keymap event.
   */
  static void OnRowMenuEvent(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGI(kTag, "Context menu event received from keymap");
      self->HandleRowFocus(lv_event_get_target_obj(e));
      self->ShowMenu();
    }
  }

  /**
   * @brief Event handler for context-menu item click.
   */
  static void OnMenuItemClicked(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGD(kTag, "Menu item clicked");
      self->HandleMenuItem(lv_event_get_target_obj(e));
    }
  }

  /**
   * @brief Event handler for canceling delete confirmation.
   */
  static void OnCancelDeleteClicked(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGI(kTag, "Delete canceled");
      self->HideConfirmDelete();
    }
  }

  /**
   * @brief Event handler for confirming delete action.
   */
  static void OnConfirmDeleteClicked(lv_event_t* e) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_event_get_user_data(e));
    if (self != nullptr) {
      ESP_LOGI(kTag, "Delete confirmed");
      self->HandleDeleteSelected();
    }
  }

  /**
   * @brief Periodic timer callback to poll async scan events.
   */
  static void OnScanTimer(lv_timer_t* timer) {
    FileBrowser* self = static_cast<FileBrowser*>(lv_timer_get_user_data(timer));
    if (self != nullptr) {
      self->PollScanEvents();
    }
  }

  /**
   * @brief Handle back-navigation behavior in current UI context.
   */
  void HandleBack() {
    if (menu_panel_ != nullptr && !lv_obj_has_flag(menu_panel_, LV_OBJ_FLAG_HIDDEN)) {
      ESP_LOGD(kTag, "Back closes menu panel");
      HideMenu();
      return;
    }
    if (confirm_dialog_ != nullptr &&
        !lv_obj_has_flag(confirm_dialog_, LV_OBJ_FLAG_HIDDEN)) {
      ESP_LOGD(kTag, "Back closes confirm dialog");
      HideConfirmDelete();
      return;
    }

    char parent[sizeof(current_path_)] = {0};
    if (!GetParentPath(parent, sizeof(parent))) {
      ESP_LOGI(kTag, "Already at root: %s", current_path_);
      return;
    }
    OpenFolder(parent);
  }

  /**
   * @brief Refresh current folder listing.
   */
  void HandleRefresh() {
    OpenFolder(current_path_);
  }

  /**
   * @brief Focus/select a file row and update footer counter.
   */
  void HandleRowFocus(lv_obj_t* row) {
    if (row == nullptr || list_obj_ == nullptr) {
      return;
    }

    uint32_t index = 0;
    uint32_t total = static_cast<uint32_t>(lv_obj_get_child_cnt(list_obj_));
    for (uint32_t i = 0; i < total; ++i) {
      lv_obj_t* child = lv_obj_get_child(list_obj_, static_cast<int32_t>(i));
      bool focused = (child == row);
      file_browser_row_set_focus(child, focused);
      file_browser_row_set_selected(child, focused);
      if (focused) {
        index = i;
      }
    }

    selected_item_ = file_browser_row_get_item(row);
    g_fb_ctx.focused_row_index = static_cast<int32_t>(index);
    UpdateCounterLabel(index + 1, total);

    if (selected_item_ != nullptr) {
      ESP_LOGD(kTag, "Focused row=%lu name=%s type=%d path=%s",
               static_cast<unsigned long>(index), selected_item_->name,
               static_cast<int>(selected_item_->type), selected_item_->full_path);
    } else {
      ESP_LOGD(kTag, "Focused row=%lu (no item)", static_cast<unsigned long>(index));
    }
  }

  /**
   * @brief Execute selected menu action.
   */
  void HandleMenuItem(lv_obj_t* target) {
    int action = -1;
    for (int i = 0; i < kMenuItemCount; ++i) {
      if (menu_buttons_[i] == target) {
        action = i;
        break;
      }
    }
    if (action < 0) {
      return;
    }

    ESP_LOGI(kTag, "Menu action index=%d", action);

    if (selected_item_ == nullptr) {
      HideMenu();
      return;
    }

    if (action == 0) {
      if (selected_item_->type == FILE_TYPE_FOLDER) {
        OpenFolder(selected_item_->full_path);
      }
      HideMenu();
      return;
    }
    if (action == 3) {
      ShowConfirmDelete();
      return;
    }

    if (hint_label_ != nullptr) {
      lv_label_set_text(hint_label_, "Action placeholder");
    }
    HideMenu();
  }

  /**
   * @brief Delete currently selected item then refresh folder.
   */
  void HandleDeleteSelected() {
    if (selected_item_ == nullptr) {
      HideConfirmDelete();
      return;
    }

    int rc = remove(selected_item_->full_path);
    if (rc == 0) {
      ESP_LOGI(kTag, "Deleted: %s", selected_item_->full_path);
      if (hint_label_ != nullptr) {
        lv_label_set_text(hint_label_, "Delete success");
      }
    } else {
      ESP_LOGW(kTag, "Delete failed: %s", selected_item_->full_path);
      if (hint_label_ != nullptr) {
        lv_label_set_text(hint_label_, "Delete failed");
      }
    }

    HideConfirmDelete();
    HideMenu();
    OpenFolder(current_path_);
  }

  /**
   * @brief Show context menu panel and switch state.
   */
  void ShowMenu() {
    if (menu_panel_ == nullptr) {
      return;
    }
    ESP_LOGD(kTag, "Show menu panel");
    lv_obj_clear_flag(menu_panel_, LV_OBJ_FLAG_HIDDEN);
    file_browser_state_menu_open_enter();
  }

  /**
   * @brief Hide context menu panel and restore browsing state.
   */
  void HideMenu() {
    if (menu_panel_ == nullptr) {
      return;
    }
    ESP_LOGD(kTag, "Hide menu panel");
    lv_obj_add_flag(menu_panel_, LV_OBJ_FLAG_HIDDEN);
    file_browser_state_menu_open_exit();
  }

  /**
   * @brief Show delete confirmation dialog.
   */
  void ShowConfirmDelete() {
    if (confirm_dialog_ == nullptr) {
      return;
    }
    ESP_LOGD(kTag, "Show confirm delete dialog");
    lv_obj_clear_flag(confirm_dialog_, LV_OBJ_FLAG_HIDDEN);
    file_browser_state_confirm_delete_enter();
  }

  /**
   * @brief Hide delete confirmation dialog.
   */
  void HideConfirmDelete() {
    if (confirm_dialog_ == nullptr) {
      return;
    }
    ESP_LOGD(kTag, "Hide confirm delete dialog");
    lv_obj_add_flag(confirm_dialog_, LV_OBJ_FLAG_HIDDEN);
    file_browser_state_confirm_delete_result(false);
  }

  /**
   * @brief Poll and process folder-scan events from queue.
   */
  void PollScanEvents() {
    QueueHandle_t queue = file_browser_scan_get_event_queue();
    if (queue == nullptr) {
      return;
    }

    file_scan_event_t evt;
    while (xQueueReceive(queue, &evt, 0) == pdTRUE) {
      if (evt.type == SCAN_EVT_PROGRESS) {
        ESP_LOGD(kTag, "Scan progress event");
        if (hint_label_ != nullptr) {
          lv_label_set_text(hint_label_, "Scanning...");
        }
      } else if (evt.type == SCAN_EVT_COMPLETE) {
        ESP_LOGI(kTag, "Scan complete event");
        RefreshListFromScanResult();
        file_browser_state_loading_exit();
        if (hint_label_ != nullptr) {
          lv_label_set_text(hint_label_, "ENC: move/ok, hold: menu");
        }
      } else if (evt.type == SCAN_EVT_ERROR) {
        ESP_LOGW(kTag, "Scan error event");
        if (hint_label_ != nullptr) {
          lv_label_set_text(hint_label_, "Scan error");
        }
        file_browser_state_browsing_enter();
      }
    }
  }

  /**
   * @brief Rebuild visible file rows from latest scan result.
   */
  void RefreshListFromScanResult() {
    if (list_obj_ == nullptr) {
      return;
    }

    while (lv_obj_get_child_cnt(list_obj_) > 0) {
      lv_obj_del(lv_obj_get_child(list_obj_, 0));
    }

    uint32_t count = 0;
    const file_item_t* items = file_browser_scan_get_results(&count);
    selected_item_ = nullptr;

    if (items == nullptr || count == 0) {
      ESP_LOGI(kTag, "Folder empty: %s", current_path_);
      lv_obj_t* empty = lv_label_create(list_obj_);
      lv_label_set_text(empty, "No files");
      lv_obj_set_style_text_color(empty, lv_color_hex(0xAAAAAA), 0);
      UpdateCounterLabel(0, 0);
      return;
    }

    ESP_LOGI(kTag, "Loaded %lu items for %s", static_cast<unsigned long>(count),
             current_path_);

    for (uint32_t i = 0; i < count; ++i) {
      lv_obj_t* row = file_browser_row_create(list_obj_, &items[i]);
      // Row touch hitbox (local list coordinates):
      // row i occupies y=[i*32 .. i*32+31] with full row width from row component.
      // Absolute y starts from list content top y=42 (because list pad_all=2).
      // So expected row i absolute y: y1~=42+i*32, y2~=73+i*32.
      lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_add_event_cb(row, OnRowClicked, LV_EVENT_CLICKED, this);
      lv_obj_add_event_cb(row, OnRowLongPressed, LV_EVENT_LONG_PRESSED, this);
      lv_obj_add_event_cb(row, OnRowMenuEvent,
                          static_cast<lv_event_code_t>(APP_EVT_CTX_MENU_OPEN), this);
      lv_obj_add_event_cb(row, OnDebugInputEvent, LV_EVENT_ALL, this);
      if (i == 0) {
        file_browser_row_set_focus(row, true);
        file_browser_row_set_selected(row, true);
        selected_item_ = &items[0];
      }
    }

    g_fb_ctx.focused_row_index = 0;
    UpdateCounterLabel(1, count);
  }

  /**
   * @brief Update footer item counter text.
   */
  void UpdateCounterLabel(uint32_t current, uint32_t total) {
    if (count_label_ == nullptr) {
      return;
    }
    char buf[24];
    snprintf(buf, sizeof(buf), "%lu/%lu", static_cast<unsigned long>(current),
             static_cast<unsigned long>(total));
    lv_label_set_text(count_label_, buf);
  }

  /**
   * @brief Resolve parent path of current folder.
   */
  bool GetParentPath(char* out, size_t out_size) const {
    if (out == nullptr || out_size == 0) {
      return false;
    }
    if (strcmp(current_path_, kRootPath) == 0) {
      return false;
    }

    char tmp[sizeof(current_path_)];
    snprintf(tmp, sizeof(tmp), "%s", current_path_);

    size_t len = strlen(tmp);
    while (len > 1 && tmp[len - 1] == '/') {
      tmp[len - 1] = '\0';
      --len;
    }

    char* slash = strrchr(tmp, '/');
    if (slash == nullptr || slash == tmp) {
      snprintf(out, out_size, "%s", kRootPath);
      return true;
    }
    *slash = '\0';

    if (strlen(tmp) < strlen(kRootPath)) {
      snprintf(out, out_size, "%s", kRootPath);
      return true;
    }

    snprintf(out, out_size, "%s", tmp);
    return true;
  }

  FileBrowser()
      : list_obj_(nullptr),
        back_btn_(nullptr),
        refresh_btn_(nullptr),
        exit_btn_(nullptr),
        menu_panel_(nullptr),
        confirm_dialog_(nullptr),
        cancel_btn_(nullptr),
        delete_btn_(nullptr),
        path_label_(nullptr),
        count_label_(nullptr),
        hint_label_(nullptr),
        selected_item_(nullptr),
        scan_timer_(nullptr),
        on_exit_callback_(nullptr) {
    memset(menu_buttons_, 0, sizeof(menu_buttons_));
    memset(current_path_, 0, sizeof(current_path_));
    snprintf(current_path_, sizeof(current_path_), "%s", kRootPath);
  }

  lv_obj_t* list_obj_;
  lv_obj_t* back_btn_;
  lv_obj_t* refresh_btn_;
  lv_obj_t* exit_btn_;
  lv_obj_t* menu_panel_;
  lv_obj_t* confirm_dialog_;
  lv_obj_t* cancel_btn_;
  lv_obj_t* delete_btn_;
  lv_obj_t* path_label_;
  lv_obj_t* count_label_;
  lv_obj_t* hint_label_;
  lv_obj_t* menu_buttons_[kMenuItemCount];
  const file_item_t* selected_item_;
  lv_timer_t* scan_timer_;
  std::function<void()> on_exit_callback_;
  char current_path_[512];
};

}  // namespace

extern "C" void file_browser_init(void) { FileBrowser::Instance().Init(); }

/**
 * @brief Create/get file browser screen.
 */
extern "C" lv_obj_t* file_browser_create_screen(void) {
  return FileBrowser::Instance().CreateScreen();
}

/**
 * @brief Open a folder path in file browser.
 */
extern "C" void file_browser_open_folder(const char* folder_path) {
  FileBrowser::Instance().OpenFolder(folder_path);
}

/**
 * @brief Get current state of file browser.
 */
extern "C" file_browser_state_t file_browser_get_state(void) {
  return FileBrowser::Instance().GetState();
}

/**
 * @brief Close file browser UI state.
 */
extern "C" void file_browser_close(void) {
  FileBrowser::Instance().Close();
}

/**
 * @brief Register C-style exit callback for file browser.
 */
extern "C" void file_browser_set_exit_callback(void (*callback)(void)) {
  FileBrowser::Instance().SetExitCallback([callback]() {
    if (callback) callback();
  });
}

/**
 * @brief Show or hide the file browser exit button.
 */
extern "C" void file_browser_show_exit_button(bool show) {
  FileBrowser::Instance().ShowExitButton(show);
}