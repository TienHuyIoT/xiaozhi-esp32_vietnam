// --- [DIENBIEN MOD] ---
#ifndef WEATHER_MODEL_H
#define WEATHER_MODEL_H

#include <string>
#include <vector>

// Cấu trúc lưu trữ 1 mốc dự báo
struct ForecastItem {
    std::string day_name; // Ví dụ: "T2", "T3"
    std::string icon_code; // Mã icon
    float temp;           // Nhiệt độ dự báo
};

// Cấu trúc thông tin thời tiết (Lấy từ API về)
struct WeatherInfo {
    std::string city;
    std::string description;
    std::string icon_code;
    float temp = 0.0f;
    int humidity = 0;
    float feels_like = 0.0f;
    int pressure = 0;
    float wind_speed = 0.0f;
    bool valid = false;

    std::vector<ForecastItem> forecast; 
};

// Cấu trúc hiển thị ra màn hình
struct IdleCardInfo {
    std::string city;
    std::string time_text;
    std::string date_text;
    std::string day_text;
    std::string temperature_text;
    std::string humidity_text;
    std::string description_text;
    std::string feels_like_text;
    std::string wind_text;
    std::string pressure_text;
    std::string battery_icon;
    std::string network_icon;
    const char* icon = nullptr;
    
    // System Info
    int battery_level = 100;
    bool is_charging = false;
    int wifi_rssi = 0;
    
    std::vector<ForecastItem> forecast;
};

#endif // WEATHER_MODEL_H