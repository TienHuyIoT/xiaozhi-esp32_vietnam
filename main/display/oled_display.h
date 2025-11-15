#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "lvgl_display.h"
#include "fft_display.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

// Forward declarations
class FFTDisplay;
#if defined(HAVE_LVGL) || __has_include(<lvgl.h>)
class OLEDDisplayAdapter;
#endif


class OledDisplay : public LvglDisplay {
private:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* content_left_ = nullptr;
    lv_obj_t* content_right_ = nullptr;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;
    lv_obj_t *emotion_label_ = nullptr;
    lv_obj_t* chat_message_label_ = nullptr;

    // FFT Display integration
    std::unique_ptr<FFTDisplay> fft_display_;
#if defined(HAVE_LVGL) || __has_include(<lvgl.h>)
    std::unique_ptr<LCDDisplayAdapter> fft_adapter_;  // Using LCD adapter for LVGL-based OLED
#endif

    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    void SetupUI_128x64();
    void SetupUI_128x32();

public:
    OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width, int height, bool mirror_x, bool mirror_y);
    ~OledDisplay();

    virtual void SetMusicInfo(const char* song_name) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetTheme(Theme* theme) override;
    
    // FFT Display methods
    virtual void start() override;
    virtual void clearScreen() override;
    virtual void stopFft() override;
    virtual void updateAudioDataBuffer(int16_t* data, size_t sample_count) override;
    virtual int16_t* createAudioDataBuffer(size_t sample_count) override;
    virtual void releaseAudioDataBuffer(int16_t* buffer = nullptr) override;
};

#endif // OLED_DISPLAY_H
