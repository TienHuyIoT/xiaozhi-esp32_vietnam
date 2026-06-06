#include "file_browser_state_machine.h"
#include "file_browser_keymap.h"

extern "C" {
#include "esp_log.h"
}

namespace {
constexpr const char* kTag = "FB_STATE";

class BrowserStateMachine final {
 public:
  static BrowserStateMachine& Instance() {
    static BrowserStateMachine instance;
    return instance;
  }

  void EnterBrowsing() {
    g_fb_ctx.current_state = FB_STATE_BROWSING;

    if (g_fb_ctx.menu_panel != nullptr) {
      lv_obj_add_flag(g_fb_ctx.menu_panel, LV_OBJ_FLAG_HIDDEN);
    }
    if (g_fb_ctx.confirm_dialog != nullptr) {
      lv_obj_add_flag(g_fb_ctx.confirm_dialog, LV_OBJ_FLAG_HIDDEN);
    }
    if (g_fb_ctx.list != nullptr) {
      lv_obj_clear_state(g_fb_ctx.list, LV_STATE_DISABLED);
    }

    ESP_LOGD(kTag, "State -> BROWSING");
  }

  void EnterMenuOpen() {
    g_fb_ctx.current_state = FB_STATE_MENU_OPEN;
    g_fb_ctx.focused_row_before_modal = g_fb_ctx.focused_row_index;

    if (g_fb_ctx.list != nullptr) {
      lv_obj_add_state(g_fb_ctx.list, LV_STATE_DISABLED);
    }
    if (g_fb_ctx.menu_panel != nullptr) {
      lv_obj_clear_flag(g_fb_ctx.menu_panel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_t* first = lv_obj_get_child(g_fb_ctx.menu_panel, 0);
      if (first != nullptr) {
        lv_group_focus_obj(first);
      }
    }

    ESP_LOGD(kTag, "State -> MENU_OPEN");
  }

  void ExitMenuOpen() {
    if (g_fb_ctx.menu_panel != nullptr) {
      lv_obj_add_flag(g_fb_ctx.menu_panel, LV_OBJ_FLAG_HIDDEN);
    }
    EnterBrowsing();
  }

  void EnterLoading() {
    g_fb_ctx.current_state = FB_STATE_LOADING;
    if (g_fb_ctx.list != nullptr) {
      lv_obj_add_state(g_fb_ctx.list, LV_STATE_DISABLED);
    }

    ESP_LOGD(kTag, "State -> LOADING");
  }

  void ExitLoading() {
    if (g_fb_ctx.list != nullptr) {
      lv_obj_clear_state(g_fb_ctx.list, LV_STATE_DISABLED);
    }
    EnterBrowsing();
  }

  void EnterConfirmDelete() {
    g_fb_ctx.current_state = FB_STATE_CONFIRM_DELETE;
    g_fb_ctx.pending_action = FB_ACTION_DELETE;

    if (g_fb_ctx.confirm_dialog != nullptr) {
      lv_obj_clear_flag(g_fb_ctx.confirm_dialog, LV_OBJ_FLAG_HIDDEN);
    }
  }

  void ConfirmDeleteResult(bool confirmed) {
    if (confirmed) {
      ESP_LOGI(kTag, "Delete confirmed");
    }

    if (g_fb_ctx.confirm_dialog != nullptr) {
      lv_obj_add_flag(g_fb_ctx.confirm_dialog, LV_OBJ_FLAG_HIDDEN);
    }
    g_fb_ctx.pending_action = FB_ACTION_NONE;
    g_fb_ctx.current_state = FB_STATE_MENU_OPEN;
  }

  void HandleEvent(int event_type, void* event_data) {
    switch (g_fb_ctx.current_state) {
      case FB_STATE_BROWSING:
        if (event_type == APP_EVT_CTX_MENU_OPEN) {
          EnterMenuOpen();
        }
        break;
      case FB_STATE_MENU_OPEN:
        if (event_type == APP_EVT_CTX_MENU_CLOSE) {
          ExitMenuOpen();
        } else if (event_type == FB_ACTION_DELETE) {
          EnterConfirmDelete();
        }
        break;
      case FB_STATE_CONFIRM_DELETE:
        if (event_type == FB_ACTION_DELETE) {
          ConfirmDeleteResult(event_data != nullptr);
        }
        break;
      case FB_STATE_LOADING:
        if (event_type == FB_ACTION_NONE) {
          ExitLoading();
        }
        break;
      case FB_STATE_CONFIRM_MOVE:
      case FB_STATE_ERROR:
      default:
        break;
    }
  }

  file_browser_state_t GetState() const {
    return g_fb_ctx.current_state;
  }
};
}  // namespace

file_browser_context_t g_fb_ctx = {
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    FB_STATE_BROWSING,
    FB_ACTION_NONE,
    0,
    0,
};

extern "C" void file_browser_state_browsing_enter(void) {
  BrowserStateMachine::Instance().EnterBrowsing();
}

extern "C" void file_browser_state_menu_open_enter(void) {
  BrowserStateMachine::Instance().EnterMenuOpen();
}

extern "C" void file_browser_state_menu_open_exit(void) {
  BrowserStateMachine::Instance().ExitMenuOpen();
}

extern "C" void file_browser_state_loading_enter(void) {
  BrowserStateMachine::Instance().EnterLoading();
}

extern "C" void file_browser_state_loading_exit(void) {
  BrowserStateMachine::Instance().ExitLoading();
}

extern "C" void file_browser_state_confirm_delete_enter(void) {
  BrowserStateMachine::Instance().EnterConfirmDelete();
}

extern "C" void file_browser_state_confirm_delete_result(bool confirmed) {
  BrowserStateMachine::Instance().ConfirmDeleteResult(confirmed);
}

extern "C" void file_browser_state_handle_event(int event_type, void* event_data) {
  BrowserStateMachine::Instance().HandleEvent(event_type, event_data);
}

extern "C" file_browser_state_t file_browser_state_get(void) {
  return BrowserStateMachine::Instance().GetState();
}