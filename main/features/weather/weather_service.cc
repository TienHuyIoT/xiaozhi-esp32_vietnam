#include "weather_service.h"
#include "settings.h"
#include "wifi_station.h"
#include "board.h"
#include "features/weather/weather_ui.h" 

#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <cmath>
#include <cstring>

#define TAG "WeatherService"

WeatherService::WeatherService() 
    : api_key_(OPEN_WEATHERMAP_API_KEY_DEFAULT)
    , last_update_time_(0) {
    weather_info_.valid = false;
}

bool WeatherService::NeedsUpdate() const {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (current_time - last_update_time_) >= WEATHER_UPDATE_INTERVAL_MS;
}

std::string WeatherService::UrlEncode(const std::string& value) {
    std::ostringstream escaped;
    escaped.fill('0');
    escaped << std::hex;

    for (std::string::const_iterator i = value.begin(), n = value.end(); i != n; ++i) {
        std::string::value_type c = (*i);
        
        // Keep alphanumeric and other accepted characters intact
        if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
            escaped << c;
            continue;
        }
        
        // Any other characters are percent-encoded
        escaped << std::uppercase;
        escaped << '%' << std::setw(2) << int((unsigned char)c);
        escaped << std::nouppercase;
    }

    return escaped.str();
}

std::string WeatherService::CapitalizeWords(std::string str) {
    bool newWord = true;
    for (size_t i = 0; i < str.length(); ++i) {
        if (std::isspace(str[i])) {
            newWord = true;
        } else if (newWord) {
            str[i] = std::toupper(str[i]);
            newWord = false;
        }
    }
    return str;
}

std::string WeatherService::GetDayNameFromTime(time_t timestamp) {
    struct tm* timeinfo = localtime(&timestamp);
    const char* wdays[] = {"CN", "T2", "T3", "T4", "T5", "T6", "T7"};
    return std::string(wdays[timeinfo->tm_wday]);
}

std::string WeatherService::GetCityFromIP() {
    std::string detected_city = "";
    
    // Check WiFi connection
    if (!WifiStation::GetInstance().IsConnected()) {
        ESP_LOGE(TAG, "GetCityFromIP: No WiFi connection");
        return "";
    }

    auto& board = Board::GetInstance();
    auto network = board.GetNetwork();
    auto http = network->CreateHttp(WEATHER_HTTP_TIMEOUT_MS);
    
    http->SetHeader("Content-Type", "application/json");
    
    if (!http->Open("GET", IP_LOCATION_API_ENDPOINT)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection for IP detection");
        return "";
    }

    int status_code = http->GetStatusCode();
    ESP_LOGI(TAG, "IP-Who-Is Status: %d", status_code);

    if (status_code == 200) {
        std::string body = http->ReadAll();
        
        ESP_LOGI(TAG, "IP-Who-Is Response: %s", body.c_str());

        if (!body.empty()) {
            cJSON* root = cJSON_Parse(body.c_str());
            if (root) {
                cJSON* success = cJSON_GetObjectItem(root, "success");
                
                if (cJSON_IsBool(success) && cJSON_IsTrue(success)) {
                    cJSON* city_json = cJSON_GetObjectItem(root, "region");
                    if (cJSON_IsString(city_json)) {
                        detected_city = city_json->valuestring;
                        ESP_LOGI(TAG, "Auto-detected Region success: %s", detected_city.c_str());
                    }
                } else {
                    ESP_LOGW(TAG, "IP-Who-Is returned success=false");
                }
                cJSON_Delete(root);
            } else {
                ESP_LOGE(TAG, "Failed to parse JSON");
            }
        }
    } else {
        ESP_LOGW(TAG, "HTTP Error: %d", status_code);
    }
    
    http->Close();
    return detected_city;
}

bool WeatherService::FetchWeatherData() {
#if !CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    ESP_LOGW(TAG, "Weather feature is disabled");
    return false;
#endif

    // Wait for WiFi connection
    int wait_retries = 0;
    while (!WifiStation::GetInstance().IsConnected() && wait_retries < 20) {
        ESP_LOGI(TAG, "Waiting for WiFi before fetching weather...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        wait_retries++;
    }

    if (!WifiStation::GetInstance().IsConnected()) {
        ESP_LOGE(TAG, "WiFi not connected, skipping weather update.");
        return false;
    }

    // Get configuration
    Settings weather_settings("wifi", false);
    std::string city = city_.empty() ? weather_settings.GetString("weather_city", "") : city_;
    std::string api_key = api_key_.empty() ? weather_settings.GetString("weather_api_key", "") : api_key_;

    if (api_key.empty()) {
        api_key = OPEN_WEATHERMAP_API_KEY_DEFAULT;
    }
    
    // Auto-detect city if not set
    if (city.empty() || city == "auto") {
        ESP_LOGI(TAG, "City not set or 'auto', detecting via IP...");
        
        for (int i = 0; i < 2; i++) {
            std::string auto_city = GetCityFromIP();
            if (!auto_city.empty()) {
                city = auto_city;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (city.empty() || city == "auto") {
            ESP_LOGW(TAG, "Failed to detect IP location, fallback to Hanoi");
            city = CITY_LOCATION_DEFAULT;
        }
    }

    // Fetch current weather and forecast data
    if (!FetchCurrentWeatherData(city, api_key)) {
        return false;
    }

    FetchForecastData(city, api_key);
    return true;
}

bool WeatherService::FetchCurrentWeatherData(const std::string& city, const std::string& api_key) {
    std::string url = std::string(WEATHER_API_ENDPOINT) + "?q=" + UrlEncode(city) +
                      "&appid=" + api_key + "&units=metric&lang=vi";

    ESP_LOGI(TAG, "Fetching current weather from: %s", url.c_str());

    auto& board = Board::GetInstance();
    auto network = board.GetNetwork();
    auto http = network->CreateHttp(WEATHER_HTTP_TIMEOUT_MS);
    
    http->SetHeader("Content-Type", "application/json");
    
    if (!http->Open("GET", url)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection for current weather");
        return false;
    }

    int status_code = http->GetStatusCode();
    if (status_code != 200) {
        ESP_LOGE(TAG, "Current weather HTTP request failed with status: %d", status_code);
        http->Close();
        return false;
    }
    
    std::string body = http->ReadAll();
    http->Close();

    cJSON* root = cJSON_Parse(body.c_str());
    if (root == nullptr) {
        ESP_LOGE(TAG, "Failed to parse current weather JSON");
        return false;
    }

    bool success = false;
    do {
        cJSON* name = cJSON_GetObjectItem(root, "name");
        cJSON* main_obj = cJSON_GetObjectItem(root, "main");
        cJSON* weather_arr = cJSON_GetObjectItem(root, "weather");

        if (!cJSON_IsString(name) || !cJSON_IsObject(main_obj) || 
            !cJSON_IsArray(weather_arr) || cJSON_GetArraySize(weather_arr) == 0) {
            break;
        }

        cJSON* temp = cJSON_GetObjectItem(main_obj, "temp");
        cJSON* humidity = cJSON_GetObjectItem(main_obj, "humidity");
        cJSON* feels_like = cJSON_GetObjectItem(main_obj, "feels_like");
        cJSON* pressure = cJSON_GetObjectItem(main_obj, "pressure");
        cJSON* wind_obj = cJSON_GetObjectItem(root, "wind");
        cJSON* wind_speed = cJSON_IsObject(wind_obj) ? cJSON_GetObjectItem(wind_obj, "speed") : nullptr;
        cJSON* w0 = cJSON_GetArrayItem(weather_arr, 0);
        cJSON* icon = cJSON_GetObjectItem(w0, "icon");
        cJSON* desc = cJSON_GetObjectItem(w0, "description");

        weather_info_.city = name->valuestring;
        if (cJSON_IsNumber(temp)) weather_info_.temp = (float)temp->valuedouble;
        if (cJSON_IsNumber(humidity)) weather_info_.humidity = humidity->valueint;
        if (cJSON_IsNumber(feels_like)) weather_info_.feels_like = (float)feels_like->valuedouble;
        if (cJSON_IsNumber(pressure)) weather_info_.pressure = pressure->valueint;
        if (cJSON_IsNumber(wind_speed)) weather_info_.wind_speed = (float)wind_speed->valuedouble;
        if (cJSON_IsString(icon)) weather_info_.icon_code = icon->valuestring;
        if (cJSON_IsString(desc)) weather_info_.description = CapitalizeWords(desc->valuestring);
        
        weather_info_.valid = true;
        success = true;
        last_update_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ESP_LOGI(TAG, "Current Weather Updated: %s, %.1f C", weather_info_.city.c_str(), weather_info_.temp);

    } while (0);

    cJSON_Delete(root);

    if (!success) {
        ESP_LOGE(TAG, "Failed to extract current weather data from JSON");
    }

    return success;
}

bool WeatherService::FetchForecastData(const std::string& city, const std::string& api_key) {    
    std::string url_forecast = "https://api.openweathermap.org/data/2.5/forecast?q=" + UrlEncode(city) +
                                "&appid=" + api_key + "&units=metric&cnt=40";
    ESP_LOGI(TAG, "Fetching forecast from: %s", url_forecast.c_str());

    auto& board = Board::GetInstance();
    auto network = board.GetNetwork();
    auto http = network->CreateHttp(WEATHER_HTTP_TIMEOUT_MS);
    
    http->SetHeader("Content-Type", "application/json");
    
    if (!http->Open("GET", url_forecast)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection for forecast");
        return false;
    }

    int status_code = http->GetStatusCode();
    if (status_code != 200) {
        ESP_LOGE(TAG, "Forecast HTTP request failed with status: %d", status_code);
        http->Close();
        return false;
    }

    size_t length = http->GetBodyLength();
    ESP_LOGI(TAG, "Forecast Content Length: %zu", length);
    if (length == 0 || length > (20 * 1024)) {
        http->Close();
        ESP_LOGE(TAG, "Forecast response body is empty");
        return false;
    }

    std::string body;
    body.reserve(length);
    if (body.capacity() < length) {
        ESP_LOGE(TAG, "Failed to reserve memory for forecast body");
        http->Close();
        return false;
    }

    constexpr size_t CHUNK_BUFFER_SIZE = 1024;
    char *buffer = (char*)malloc(CHUNK_BUFFER_SIZE);
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for forecast buffer");
        http->Close();
        return false;
    }

    int read_len;
    while ((read_len = http->Read(buffer, CHUNK_BUFFER_SIZE)) > 0) {
        body.append(buffer, read_len);
        ESP_LOGI(TAG, "Forecast Read %d bytes", read_len);
    }
    free(buffer);
    http->Close();
    ESP_LOGI(TAG, "Forecast Response %zu: %.512s", body.size(), body.c_str());

    cJSON* root = cJSON_Parse(body.c_str());
    if (root == nullptr) {
        ESP_LOGE(TAG, "Failed to parse Forecast JSON");
        return false;
    }

    bool success = false;
    cJSON* list = cJSON_GetObjectItem(root, "list");
    
    if (cJSON_IsArray(list)) {
        weather_info_.forecast.clear();
        int count = 0;
        int array_size = cJSON_GetArraySize(list);

        for (int i = 0; i < array_size; i++) {
            cJSON* item = cJSON_GetArrayItem(list, i);
            cJSON* dt_txt = cJSON_GetObjectItem(item, "dt_txt");
            
            // Tìm mốc 12:00
            if (cJSON_IsString(dt_txt) && strstr(dt_txt->valuestring, "12:00:00")) {
                ForecastItem f_item;
                
                cJSON* dt = cJSON_GetObjectItem(item, "dt");
                if (cJSON_IsNumber(dt)) f_item.day_name = GetDayNameFromTime((time_t)dt->valueint);

                cJSON* main_forecast = cJSON_GetObjectItem(item, "main");
                if (main_forecast) {
                    cJSON* t = cJSON_GetObjectItem(main_forecast, "temp");
                    if (cJSON_IsNumber(t)) f_item.temp = (float)t->valuedouble;
                }
                
                cJSON* w_arr = cJSON_GetObjectItem(item, "weather");
                if (w_arr && cJSON_GetArraySize(w_arr) > 0) {
                    cJSON* w0 = cJSON_GetArrayItem(w_arr, 0);
                    cJSON* ic = cJSON_GetObjectItem(w0, "icon");
                    if (cJSON_IsString(ic)) f_item.icon_code = ic->valuestring;
                }

                weather_info_.forecast.push_back(f_item);
                count++;
                if (count >= 5) break; 
            }
        }
        success = (count > 0);
        ESP_LOGI(TAG, "Forecast Updated: %d days", count);
    }

    cJSON_Delete(root);
    return success;
}
