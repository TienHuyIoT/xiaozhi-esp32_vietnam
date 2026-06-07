#ifndef WEATHER_SERVICE_H
#define WEATHER_SERVICE_H

#include "weather_model.h"
#include "weather_config.h"
#include <string>
#include <memory>
#include <ctime>

class WeatherService {
public:
    enum class FetchStatus {
        kNever = 0,
        kSuccess,
        kOffline,
        kError,
    };

    static WeatherService& GetInstance() {
        static WeatherService instance;
        return instance;
    }

    WeatherService(const WeatherService&) = delete;
    WeatherService& operator=(const WeatherService&) = delete;

    // Fetch weather data from API
    bool FetchWeatherData();

    // Fetch current weather data from API
    bool FetchCurrentWeatherData(const std::string& city, const std::string& api_key);

    // Fetch 5-day forecast data from API
    bool FetchForecastData(const std::string& city, const std::string& api_key);

    // Get current weather info
    const WeatherInfo& GetWeatherInfo() const { return weather_info_; }

    // Check if weather data needs update
    bool NeedsUpdate() const;

    // Returns age of latest successful weather data in milliseconds.
    uint32_t GetDataAgeMs() const;

    // Returns true when data age exceeds stale threshold.
    bool IsStale(uint32_t stale_threshold_ms = WEATHER_STALE_THRESHOLD_MS) const;

    // Returns last fetch status for UI state decisions.
    FetchStatus GetLastFetchStatus() const { return last_fetch_status_; }

    // Get city from IP address
    static std::string GetCityFromIP();

    // Set API key
    void SetApiKey(const std::string& api_key) { api_key_ = api_key; }

    // Set city manually
    void SetCity(const std::string& city) { city_ = city; }

private:
    WeatherService();
    ~WeatherService() = default;

    WeatherInfo weather_info_;
    std::string api_key_;
    std::string city_;
    uint32_t last_update_time_;
    FetchStatus last_fetch_status_;

    // Helper function to encode URL
    static std::string UrlEncode(const std::string& value);

    // Helper function to capitalize words
    static std::string CapitalizeWords(std::string str);

    static std::string GetDayNameFromTime(time_t timestamp);
};

#endif // WEATHER_SERVICE_H