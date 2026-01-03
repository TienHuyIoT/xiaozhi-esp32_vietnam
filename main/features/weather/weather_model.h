// --- [DIENBIEN MOD] ---
#ifndef WEATHER_MODEL_H
#define WEATHER_MODEL_H

#include <string>

// Weather information structure
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
};

// Idle card display information
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
    const char* icon = nullptr;
    int battery_level = 100; // [NEW] Thêm biến pin
};

#endif // WEATHER_MODEL_H