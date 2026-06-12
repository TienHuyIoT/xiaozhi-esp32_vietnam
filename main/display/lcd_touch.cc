#include "lcd_touch.h"
#include <esp_log.h>
#include <esp_timer.h>

static const char* TAG = "LcdTouch";

#define TOUCH_POLLING_DELAY_MS 10

// ============================================================================
// LcdTouch Base Class Implementation
// ============================================================================

LcdTouch::LcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
                   uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y, TouchInterruptCallback callback)
    : touch_handle_(touch_handle)
    , panel_io_(panel_io)
    , swap_xy_(swap_xy)
    , mirror_x_(mirror_x)
    , mirror_y_(mirror_y)
    , width_(width)
    , height_(height)
    , interrupt_callback_(callback)
{
    ESP_LOGI(TAG, "LcdTouch initialized: %dx%d, swap_xy=%d, mirror_x=%d, mirror_y=%d",
             width_, height_, swap_xy_, mirror_x_, mirror_y_);

#ifdef LVGL_PORT_TOUCH_DRIVER_CALLBACK
    const lvgl_port_touch_cfg_t touch_cfg = {
      .disp = lv_display_get_default(),
      .handle = touch_handle_,
    };
    lvgl_port_add_touch(&touch_cfg);
#else
    ESP_LOGI(TAG, "Adding custom touch driver to LVGL...");
    touch_indev_ = lv_indev_create();
    lv_indev_set_type(touch_indev_, LV_INDEV_TYPE_POINTER);
    lv_indev_set_driver_data(touch_indev_, touch_handle_);
    lv_indev_set_user_data(touch_indev_, this);

    // Create mutex protecting the shared snapshot
    point_mutex_ = xSemaphoreCreateMutex();

    // LVGL read_cb only copies the latest snapshot — no I2C, no blocking
    lv_indev_set_read_cb(touch_indev_, [](lv_indev_t *drv, lv_indev_data_t *data) {
        LcdTouch* instance = (LcdTouch*)lv_indev_get_user_data(drv);
        instance->ReadFromBuffer(data);
    });

    // touch_event_task is the sole hardware reader and snapshot producer
    xTaskCreatePinnedToCore(touch_event_task, "touch_task", 4 * 1024, this, 5, NULL, 0);
#endif
}

LcdTouch::~LcdTouch() {
    if (touch_handle_) {
        ESP_LOGI(TAG, "LcdTouch destroyed");
    }
}

void LcdTouch::touch_event_task(void* arg)
{
    LcdTouch *touch = static_cast<LcdTouch*>(arg);
    if (touch == nullptr) {
        ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "touch_event_task started");

    lv_indev_data_t data;
    vTaskDelay(pdMS_TO_TICKS(100)); // Initial delay
    while (true) {
        touch->touch_driver_read(touch->touch_indev_, &data);
    }
}

void LcdTouch::touch_driver_read(lv_indev_t* drv, lv_indev_data_t* data) {
    constexpr uint8_t TOUCH_MAX_POINT = 1;
    esp_lcd_touch_point_data_t point_data[TOUCH_MAX_POINT];
    uint8_t touch_cnt = 0;

    if (interrupt_callback_) {
        bool has_interrupt = interrupt_callback_();
        if (!has_interrupt && !was_touching_) {
            data->state = LV_INDEV_STATE_RELEASED;
            data->continue_reading = true;
            return;
        }
    } else {
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLLING_DELAY_MS));
    }

    esp_lcd_touch_handle_t touch_ctx =
        static_cast<esp_lcd_touch_handle_t>(lv_indev_get_driver_data(drv));
    esp_err_t err = esp_lcd_touch_read_data(touch_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read touch data: %s", esp_err_to_name(err));
        return;
    }

    err = esp_lcd_touch_get_data(touch_ctx, point_data, &touch_cnt, TOUCH_MAX_POINT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get touch data: %s", esp_err_to_name(err));
        return;
    }

    if (touch_cnt > 0 && touch_cnt <= TOUCH_MAX_POINT) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = point_data[0].x;
        data->point.y = point_data[0].y;
        was_touching_ = true;
        touch_end_time_ = esp_timer_get_time();
        ESP_LOGD(TAG, "Touch PRESSED at (%d, %d)", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        if (was_touching_) {
            int64_t time_since_release = esp_timer_get_time() - touch_end_time_;
            if (time_since_release > TOUCH_RELEASE_TIMEOUT) {
                was_touching_ = false;
            }
        }
        ESP_LOGD(TAG, "Touch RELEASED");
    }

    // Write to shared snapshot so LVGL read_cb (ReadFromBuffer) has fresh data
    if (point_mutex_ && xSemaphoreTake(point_mutex_, 0) == pdTRUE) {
        latest_point_.state = data->state;
        if (data->state == LV_INDEV_STATE_PRESSED) {
            latest_point_.x = data->point.x;
            latest_point_.y = data->point.y;
        }
        xSemaphoreGive(point_mutex_);
    } else {
        ESP_LOGW(TAG, "Snapshot mutex busy while producing touch data");
    }

    data->continue_reading = true;
}

void LcdTouch::ReadFromBuffer(lv_indev_data_t* data) {
    if (point_mutex_ && xSemaphoreTake(point_mutex_, pdMS_TO_TICKS(2)) == pdTRUE) {
        data->point.x = latest_point_.x;
        data->point.y = latest_point_.y;
        data->state   = latest_point_.state;
        xSemaphoreGive(point_mutex_);

        static lv_indev_state_t last_consumer_state = LV_INDEV_STATE_RELEASED;
        if (data->state != last_consumer_state) {
            if (data->state == LV_INDEV_STATE_PRESSED) {
                ESP_LOGD(TAG, "LVGL read_cb state -> PRESSED (%d, %d)", data->point.x,
                         data->point.y);
            } else {
                ESP_LOGD(TAG, "LVGL read_cb state -> RELEASED");
            }
            last_consumer_state = data->state;
        }
    } else {
        // Mutex timeout (very rare) — report released so LVGL does not hang
        data->state = LV_INDEV_STATE_RELEASED;
        ESP_LOGW(TAG, "Snapshot mutex timeout in LVGL read_cb");
    }
    data->continue_reading = false;
}

void LcdTouch::SetInterruptCallback(TouchInterruptCallback callback) {
    interrupt_callback_ = callback;
}

esp_lcd_touch_handle_t LcdTouch::GetTouchHandle() const {
    return touch_handle_;
}

// ============================================================================
// I2cLcdTouch Implementation
// ============================================================================

I2cLcdTouch::I2cLcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
                         uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y,
                         TouchInterruptCallback callback)
    : LcdTouch(touch_handle, panel_io, width, height, swap_xy, mirror_x, mirror_y, callback) {}

I2cLcdTouch::~I2cLcdTouch() {
    ESP_LOGI(TAG, "I2cLcdTouch destroyed");
}

// ============================================================================
// SpiLcdTouch Implementation
// ============================================================================

SpiLcdTouch::SpiLcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
                         uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y,
                         TouchInterruptCallback callback)
    : LcdTouch(touch_handle, panel_io, width, height, swap_xy, mirror_x, mirror_y, callback) {}

SpiLcdTouch::~SpiLcdTouch() {
    ESP_LOGI(TAG, "SpiLcdTouch destroyed");
}


