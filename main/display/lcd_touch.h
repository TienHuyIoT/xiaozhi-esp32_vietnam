#ifndef LCD_TOUCH_H
#define LCD_TOUCH_H

#include <lvgl.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_touch.h>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/** Touch release debounce timeout (µs).
 * After the last detected touch point, wait this long before declaring released.
 */
#define TOUCH_RELEASE_TIMEOUT 5000  // 5 ms

using TouchInterruptCallback = std::function<bool()>;

// Base LcdTouch class
class LcdTouch {
protected:
    esp_lcd_touch_handle_t touch_handle_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    lv_indev_t* touch_indev_ = nullptr;

    // Display transformation settings
    bool swap_xy_;
    bool mirror_x_;
    bool mirror_y_;
    uint16_t width_;
    uint16_t height_;
    TouchInterruptCallback interrupt_callback_ = nullptr;

    // Minimal state needed for interrupt-driven release debounce
    bool was_touching_ = false;
    int64_t touch_end_time_ = 0;

    // Producer-consumer snapshot: written by touch_event_task, read by LVGL read_cb
    SemaphoreHandle_t point_mutex_ = nullptr;
    struct {
        int16_t x = 0;
        int16_t y = 0;
        lv_indev_state_t state = LV_INDEV_STATE_RELEASED;
    } latest_point_;

public:
    LcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
             uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y,
             TouchInterruptCallback callback);
    virtual ~LcdTouch();

    virtual void SetInterruptCallback(TouchInterruptCallback callback);
    virtual esp_lcd_touch_handle_t GetTouchHandle() const;

    // Called by touch_event_task: reads hardware, writes snapshot
    void touch_driver_read(lv_indev_t* drv, lv_indev_data_t* data);

    // Called by LVGL indev read_cb: copies snapshot, no I2C, no blocking
    void ReadFromBuffer(lv_indev_data_t* data);

    static void touch_event_task(void* arg);
};

// I2C LCD Touch (FT6x36, CST816S, etc.)
class I2cLcdTouch : public LcdTouch {
public:
    I2cLcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
                uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y,
                TouchInterruptCallback callback = nullptr);
    virtual ~I2cLcdTouch();
};

// SPI LCD Touch (XPT2046, etc.)
class SpiLcdTouch : public LcdTouch {
public:
    SpiLcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
                uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y,
                TouchInterruptCallback callback = nullptr);
    virtual ~SpiLcdTouch();
};

#endif  // LCD_TOUCH_H
