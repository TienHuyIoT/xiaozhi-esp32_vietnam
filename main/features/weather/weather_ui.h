
// --- [DIENBIEN MOD] ---
#ifndef WEATHER_UI_H
#define WEATHER_UI_H

#include "weather_model.h"
#include "weather_config.h"
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
    void UpdateIdleDisplay(const WeatherInfo& weather_info);
    static const char* GetWeatherIcon(const std::string& code);
    bool IsInitialized() const { return container_ != nullptr; }

private:
    lv_obj_t* container_;
    
    // UI Elements
    lv_obj_t* label_city_;      
    lv_obj_t* label_time_;      
    lv_obj_t* label_date_;      
    
    // Analog Clock Elements (Center)
    lv_obj_t* clock_arc_;       
    lv_obj_t* hand_hour_;
    lv_obj_t* hand_min_;
    lv_obj_t* hand_sec_;
    lv_obj_t* center_point_;

    // Sides
    lv_obj_t* icon_weather_main_; 
    lv_obj_t* label_temp_;        
    
    // Bottom Info
    lv_obj_t* label_brand_;       
    lv_obj_t* label_humidity_;

    int screen_width_;
    int screen_height_;

    // Helpers
    // Hàm tính toán tọa độ vạch bezel chính xác theo góc
    void GetBezelXY(int minute_idx, int w, int h, int margin, int& out_x, int& out_y);
    void CreateBezelTicksAndNumbers(lv_obj_t* parent, int w, int h);
    
    void CreateCenterClock(lv_obj_t* parent, int w, int h, float ratio);
    
    // Layout sections
    void CreateTopSection(lv_obj_t* parent);    
    void CreateMiddleSection(lv_obj_t* parent); // Icon (Trái) + Temp (Phải)
    void CreateCitySection(lv_obj_t* parent);   // City (Dưới vòng tròn)
    void CreateBottomSection(lv_obj_t* parent); // Brand (Trái) + Humid (Phải)
};

#endif // WEATHER_UI_H