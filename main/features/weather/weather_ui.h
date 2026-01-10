// --- [DIENBIEN MOD] ---
#ifndef WEATHER_UI_H
#define WEATHER_UI_H

#include "weather_model.h"
#include "lvgl_display.h"
#if HAVE_LVGL
#include <lvgl.h>
#endif
#include <string>

class WeatherUI {
public:
    WeatherUI();
    ~WeatherUI();

    void SetupIdleUI(lv_obj_t* parent, int screen_width, int screen_height);
    void ShowIdleCard(const IdleCardInfo& info);
    void HideIdleCard();

    // Get weather icon from code
    static const char* GetWeatherIcon(const std::string& code);
    static const char* GetBatteryIcon(int level, bool charging);
    static const char* GetWifiIcon(int rssi);

    bool IsInitialized() const { return container_ != nullptr; }

private:
    lv_obj_t* container_;
    int screen_width_;
    int screen_height_;

    // --- CÁC THÀNH PHẦN GIAO DIỆN ---
    lv_obj_t* label_wifi_icon_;
    lv_obj_t* label_bat_icon_;
    
    // Hàng 1
    lv_obj_t* label_full_date_;
    
    // Hàng 2: Đồng hồ (Chia nhỏ thành 8 thành phần)
    lv_obj_t* cont_clock_;       // Container chứa cả hàng đồng hồ
    lv_obj_t* lbl_clock_digits_[8]; // 0,1: Giờ | 2: : | 3,4: Phút | 5: : | 6,7: Giây
    
    // Hàng 3 (Weather Group)
    lv_obj_t* group_weather_; 
    lv_obj_t* label_main_temp_;
    lv_obj_t* label_main_icon_;
    lv_obj_t* label_main_desc_; 
    
    // Hàng 4
    lv_obj_t* label_city_;
    
    // Hàng 5 (Detail Group)
    lv_obj_t* group_details_; 
    lv_obj_t* arc_humid_;
    lv_obj_t* label_humid_val_;
    lv_obj_t* arc_press_;
    lv_obj_t* label_press_val_;
    lv_obj_t* arc_wind_;
    lv_obj_t* label_wind_val_;
    
    // Hàng 6 (Forecast)
    lv_obj_t* obj_forecast_cont_;

    // Helpers
    void CreateGradientBars(lv_obj_t* parent);
    void CreateDetailArc(lv_obj_t* parent, lv_obj_t** arc_out, lv_obj_t** label_out, lv_color_t color);
};

#endif // WEATHER_UI_H