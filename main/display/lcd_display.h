#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "lvgl_display.h"
#include "gif/lvgl_gif.h"
#include "features/weather/weather_ui.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <font_emoji.h>

#include <atomic>
#include <memory>

#define PREVIEW_IMAGE_DURATION_MS 5000

// Theme color structure
struct ThemeColors {
    lv_color_t background;
    lv_color_t text;
    lv_color_t chat_background;
    lv_color_t user_bubble;
    lv_color_t assistant_bubble;
    lv_color_t system_bubble;
    lv_color_t system_text;
    lv_color_t border;
    lv_color_t low_battery;
};

class LcdDisplay : public LvglDisplay {
protected:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    
    lv_draw_buf_t draw_buf_;
    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;
    lv_obj_t* preview_image_ = nullptr;
    lv_obj_t* emoji_label_ = nullptr;
    lv_obj_t* emoji_image_ = nullptr;
    std::unique_ptr<LvglGif> gif_controller_ = nullptr;
    lv_obj_t* emoji_box_ = nullptr;
    lv_obj_t* chat_message_label_ = nullptr;
    esp_timer_handle_t preview_timer_ = nullptr;
    std::unique_ptr<LvglImage> preview_image_cached_ = nullptr;

    // Weather UI component
    std::unique_ptr<WeatherUI> weather_ui_;

    void InitializeLcdThemes();
    void SetupUI();
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    // Media overlay state (true = emoji/chat hidden for media content)
    bool media_overlay_active_ = false;

    // Rotate and offset settings
    int rotation_degree_ = 0;
    void SetRotationAndOffset(lv_display_rotation_t rotation, int offset_x, int offset_y);

protected:
    // Add protected constructor
    LcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width, int height);
    
public:
    ~LcdDisplay();
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetChatMessage(const char* role, const char* content) override; 
    virtual void SetPreviewImage(std::unique_ptr<LvglImage> image) override;

    // Theme switching
    virtual void SetTheme(Theme* theme) override;

    /** Show/hide the media overlay (hides emoji + chat for FFT/video). */
    virtual void SetMediaOverlayActive(bool active) override;

    // Rotate lcd display
    virtual bool SetRotation(int rotation_degree, bool save_setting) override;

    /** Get the raw LCD panel handle (for direct drawing, e.g. video player) */
    esp_lcd_panel_handle_t GetPanelHandle() const { return panel_; }
    
#ifdef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    virtual void ShowIdleCard(const IdleCardInfo& info) override;
    virtual void HideIdleCard() override;
#endif
};

// SPI LCD Display
class SpiLcdDisplay : public LcdDisplay {
public:
    SpiLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy);
};

// RGB LCD Display
class RgbLcdDisplay : public LcdDisplay {
public:
    RgbLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy);
};

// MIPI LCD Display
class MipiLcdDisplay : public LcdDisplay {
public:
    MipiLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                   int width, int height, int offset_x, int offset_y,
                   bool mirror_x, bool mirror_y, bool swap_xy,
                   lv_color_format_t color_format = LV_COLOR_FORMAT_RGB565);
};

#endif // LCD_DISPLAY_H
