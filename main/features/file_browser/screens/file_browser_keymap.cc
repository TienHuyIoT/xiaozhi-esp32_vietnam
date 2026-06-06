#include "file_browser_keymap.h"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {
constexpr const char* kTag = "FB_KEYMAP";

const char* KeyToString(uint32_t key) {
  switch (key) {
    case LV_KEY_NEXT:
      return "NEXT";
    case LV_KEY_PREV:
      return "PREV";
    case LV_KEY_ENTER:
      return "ENTER";
    default:
      return "UNKNOWN";
  }
}

class KeymapController final {
 public:
  static KeymapController& Instance() {
    static KeymapController instance;
    return instance;
  }

  void Init() {
    indev_ = lv_indev_create();
    lv_indev_set_type(indev_, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(indev_, file_browser_encoder_read);

    lv_group_t* group = lv_group_get_default();
    if (group == nullptr) {
      group = lv_group_create();
      lv_group_set_default(group);
    }
    if (indev_ != nullptr) {
      lv_indev_set_group(indev_, group);
    }

    ESP_LOGI(kTag, "Keymap initialized");
  }

  void Read(lv_indev_t* indev, lv_indev_data_t* data) {
    LV_UNUSED(indev);

    if (has_pending_key_) {
      data->key = static_cast<uint32_t>(pending_key_);
      data->state = LV_INDEV_STATE_PRESSED;
      ESP_LOGD(kTag, "Emit key=%s(%lu)", KeyToString(data->key),
               static_cast<unsigned long>(data->key));
      has_pending_key_ = false;
      return;
    }

    data->key = 0;
    data->state = LV_INDEV_STATE_RELEASED;
  }

  void OnRotate(int32_t delta) {
    if (delta > 0) {
      pending_key_ = LV_KEY_NEXT;
      has_pending_key_ = true;
      g_encoder_state.last_encoder_val += delta;
      ESP_LOGD(kTag, "Rotate delta=%ld -> key=NEXT", static_cast<long>(delta));
    } else if (delta < 0) {
      pending_key_ = LV_KEY_PREV;
      has_pending_key_ = true;
      g_encoder_state.last_encoder_val += delta;
      ESP_LOGD(kTag, "Rotate delta=%ld -> key=PREV", static_cast<long>(delta));
    }
  }

  void OnButton(bool pressed) {
    uint32_t now = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (pressed) {
      ESP_LOGD(kTag, "Encoder button pressed");
      g_encoder_state.press_start_time = now;
      g_encoder_state.is_long_press_triggered = false;
      return;
    }

    if (g_encoder_state.press_start_time == 0) {
      return;
    }

    uint32_t duration = now - g_encoder_state.press_start_time;
    ESP_LOGD(kTag, "Encoder button released, duration=%lu ms",
             static_cast<unsigned long>(duration));
    if (duration < ENCODER_LONG_PRESS_THRESHOLD_MS &&
        !g_encoder_state.is_long_press_triggered) {
      pending_key_ = LV_KEY_ENTER;
      has_pending_key_ = true;
      ESP_LOGI(kTag, "Short press -> key=ENTER");
    }

    g_encoder_state.press_start_time = 0;
    g_encoder_state.is_long_press_triggered = false;
  }

  void LongPressCheck() {
    if (g_encoder_state.press_start_time == 0 || g_encoder_state.is_long_press_triggered) {
      return;
    }

    uint32_t now = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t duration = now - g_encoder_state.press_start_time;
    if (duration < ENCODER_LONG_PRESS_THRESHOLD_MS) {
      return;
    }

    g_encoder_state.is_long_press_triggered = true;
    ESP_LOGI(kTag, "Long press detected -> send APP_EVT_CTX_MENU_OPEN");
    lv_group_t* group = lv_group_get_default();
    lv_obj_t* focused = (group != nullptr) ? lv_group_get_focused(group) : nullptr;
    if (focused != nullptr) {
      lv_obj_send_event(focused, static_cast<lv_event_code_t>(APP_EVT_CTX_MENU_OPEN), nullptr);
    }
  }

  uint32_t PressDurationMs() const {
    if (g_encoder_state.press_start_time == 0) {
      return 0;
    }
    uint32_t now = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    return now - g_encoder_state.press_start_time;
  }

  void ResetPress() {
    g_encoder_state.press_start_time = 0;
    g_encoder_state.is_long_press_triggered = false;
  }

 private:
  KeymapController() : indev_(nullptr), pending_key_(0), has_pending_key_(false) {}

  lv_indev_t* indev_;
  int pending_key_;
  bool has_pending_key_;
};

}  // namespace

encoder_state_t g_encoder_state = {
    0,
    0,
    false,
};

extern "C" void file_browser_keymap_init(void) {
  KeymapController::Instance().Init();
}

extern "C" void file_browser_encoder_read(lv_indev_t* indev,
                                           lv_indev_data_t* data) {
  KeymapController::Instance().Read(indev, data);
}

extern "C" void file_browser_encoder_rotation_handler(int32_t delta) {
  KeymapController::Instance().OnRotate(delta);
}

extern "C" void file_browser_encoder_button_handler(bool pressed) {
  KeymapController::Instance().OnButton(pressed);
}

extern "C" void file_browser_encoder_long_press_check(void) {
  KeymapController::Instance().LongPressCheck();
}

extern "C" uint32_t file_browser_get_encoder_press_duration(void) {
  return KeymapController::Instance().PressDurationMs();
}

extern "C" void file_browser_encoder_press_reset(void) {
  KeymapController::Instance().ResetPress();
}