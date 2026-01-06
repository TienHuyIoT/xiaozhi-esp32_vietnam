#ifndef LCD_TOUCH_H
#define LCD_TOUCH_H

#include <lvgl.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_touch.h>
#include <functional>

/** Touch release debounce timeout
 * If touch using interrupt, after the interrupt is triggered, wait for this time
 * to confirm that the touch is really released. Time has been turned down to 5ms
 * 
 * If touch using polling, this depends on the polling interval.
*/

#define TOUCH_RELEASE_TIMEOUT 5000 // 5ms
#define TOUCH_SWIPE_RELEASE_TIMEOUT 500000 // 500ms

// Touch gesture types
enum TouchGesture {
    TOUCH_GESTURE_NONE = 0,
    TOUCH_GESTURE_SWIPE_UP,
    TOUCH_GESTURE_SWIPE_DOWN,
    TOUCH_GESTURE_SWIPE_LEFT,
    TOUCH_GESTURE_SWIPE_RIGHT,
    TOUCH_GESTURE_TAP,
    TOUCH_GESTURE_DOUBLE_TAP,
    TOUCH_GESTURE_LONG_PRESS
};

typedef struct {
    float   ratio_xy;                 // Ratio of X to Y movement
    int16_t swipe_threshold;          // Minimum pixels for swipe (reduced for better sensitivity)
    int32_t swipe_timeout_us;         // Maximum time for swipe
    int32_t tap_timeout_us;           // Maximum time for tap
    int32_t double_tap_window_us;     // Window for double tap
    int32_t long_press_time_us;       // Time for long press
    int32_t release_timeout_us;       // Time to confirm release
} touch_gesture_t;

#define LCD_TOUCH_GESTURE_CONFIG()                      \
    {                                                   \
        .ratio_xy = 1.5f,                               \
        .swipe_threshold = 50,                          \
        .swipe_timeout_us = 2000000,                    \
        .tap_timeout_us = 170000,                       \
        .double_tap_window_us = 500000,                 \
        .long_press_time_us = 800000,                   \
        .release_timeout_us = TOUCH_RELEASE_TIMEOUT     \
    }

// Touch event callback type
using TouchEventCallback = std::function<void(TouchGesture gesture, int16_t x, int16_t y)>;
using TouchInterruptCallback = std::function<bool()>;

// Base LcdTouch class
class LcdTouch {
protected:
    esp_lcd_touch_handle_t touch_handle_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    lv_indev_t *touch_indev_ = nullptr;
    
    // Touch event callback
    TouchEventCallback gesture_callback_ = nullptr;
    touch_gesture_t gesture_config_ = LCD_TOUCH_GESTURE_CONFIG();
    int32_t release_timeout_us_ = TOUCH_RELEASE_TIMEOUT;

    // Touch state tracking
    bool is_touching_ = false;
    bool was_touching_ = false;
    int16_t touch_start_x_ = 0;
    int16_t touch_start_y_ = 0;
    int16_t touch_current_x_ = 0;
    int16_t touch_current_y_ = 0;
    int64_t touch_start_time_ = 0;
    int64_t touch_end_time_ = 0;
    int64_t last_tap_time_ = 0;
    bool gesture_detected_ = false;
    bool long_press_detected_ = false;
    bool is_swiping_ = false;  // Flag to prevent tap when swiping
    
    // Display transformation settings
    bool swap_xy_;
    bool mirror_x_;
    bool mirror_y_;
    uint16_t width_;
    uint16_t height_;
    TouchInterruptCallback interrupt_callback_ = nullptr;

public:
    LcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
             uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y, TouchInterruptCallback callback);
    virtual ~LcdTouch();
    
    // Register callback for gesture events
    virtual void SetGestureCallback(TouchEventCallback callback);
    virtual void SetInterruptCallback(TouchInterruptCallback callback);
    
    // Configure gesture detection parameters
    virtual void SetRatioXY(float ratio);
    virtual void SetSwipeThreshold(int16_t threshold);
    virtual void SetSwipeTimeout(int32_t timeout_us);
    virtual void SetTapTimeout(int32_t timeout_us);
    virtual void SetDoubleTapWindow(int32_t window_us);
    virtual void SetLongPressTime(int32_t time_us);
    virtual void SetReleaseTimeout(int32_t time_us);
    
    // Get touch handle
    virtual esp_lcd_touch_handle_t GetTouchHandle() const;

    // LVGL touch driver callback
    void touch_driver_read(lv_indev_t *drv, lv_indev_data_t *data);

    static void touch_event_task(void* arg);
    
protected:
    // Internal gesture detection methods
    virtual TouchGesture DetectSwipeGesture();
    virtual void HandleTouchPress(int16_t x, int16_t y);
    virtual void HandleTouchRefresh(int16_t x, int16_t y);
    virtual void HandleTouchRelease();
};

// I2C LCD Touch (FT6x36, etc.)
class I2cLcdTouch : public LcdTouch {
public:
    I2cLcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
                uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y, TouchInterruptCallback callback = nullptr);
    virtual ~I2cLcdTouch();
};

// SPI LCD Touch (XPT2046, etc.)
class SpiLcdTouch : public LcdTouch {
public:
    SpiLcdTouch(esp_lcd_touch_handle_t touch_handle, esp_lcd_panel_io_handle_t panel_io,
                uint16_t width, uint16_t height, bool swap_xy, bool mirror_x, bool mirror_y, TouchInterruptCallback callback = nullptr);
    virtual ~SpiLcdTouch();
};

#endif // LCD_TOUCH_H
