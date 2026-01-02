#include "weather_ui.h"
#include <esp_log.h>
#include <time.h>
#include <cmath>
#include <font_awesome.h>

#ifndef FONT_AWESOME_BOLT
#define FONT_AWESOME_BOLT "\uf0e7"
#endif

#define TAG "WeatherUI"

// External font declarations
LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(lv_font_montserrat_22);
LV_FONT_DECLARE(lv_font_montserrat_28);
LV_FONT_DECLARE(lv_font_ds_digitb_48);

WeatherUI::WeatherUI() 
    : idle_panel_(nullptr)
    , idle_time_label_(nullptr)
    , idle_date_label_(nullptr)
    , idle_temp_label_(nullptr)
    , idle_icon_label_(nullptr)
    , idle_city_label_(nullptr)
    , idle_detail_label_(nullptr)
    , screen_width_(0)
    , screen_height_(0) {
}

WeatherUI::~WeatherUI() {
    HideIdleCard();
}

const char* WeatherUI::GetWeatherIcon(const std::string& code) {
    if (code.size() < 2) return FONT_AWESOME_CLOUD;
    std::string prefix = code.substr(0, 2);
    if (prefix == "01") return FONT_AWESOME_SUN;
    if (prefix == "09" || prefix == "10") return FONT_AWESOME_CLOUD_RAIN;
    if (prefix == "11") return FONT_AWESOME_BOLT;
    return FONT_AWESOME_CLOUD;
}

void WeatherUI::SetupIdleUI(lv_obj_t* parent, int screen_width, int screen_height) {
#ifndef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    return;
#endif

    screen_width_ = screen_width;
    screen_height_ = screen_height;

    if (!parent) {
        ESP_LOGE(TAG, "Parent object is null");
        return;
    }

    // Create main panel
    if (!idle_panel_) {
        idle_panel_ = lv_obj_create(parent);
        lv_obj_set_size(idle_panel_, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_radius(idle_panel_, 0, 0);
        lv_obj_set_style_bg_color(idle_panel_, lv_color_black(), 0);
        lv_obj_set_flex_flow(idle_panel_, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(idle_panel_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_border_width(idle_panel_, 0, 0);
        lv_obj_add_flag(idle_panel_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_scrollbar_mode(idle_panel_, LV_SCROLLBAR_MODE_OFF);
    }

    // Time label
    if (!idle_time_label_) {
        idle_time_label_ = lv_label_create(idle_panel_);
        lv_obj_set_style_text_font(idle_time_label_, &lv_font_ds_digitb_48, LV_PART_MAIN);
        lv_obj_set_style_text_color(idle_time_label_, lv_color_hex(0xFFFF00), 0);
        lv_obj_set_style_text_letter_space(idle_time_label_, 0, 0); // IMPORTANT: Disable letter spacing for digital font
    }

    // Date label
    if (!idle_date_label_) {
        idle_date_label_ = lv_label_create(idle_panel_);
        lv_obj_set_style_text_font(idle_date_label_, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(idle_date_label_, lv_color_hex(0xAAAAAA), 0);
    }

    // Icon and temperature row
    if (!idle_icon_label_) {
        lv_obj_t* row = lv_obj_create(idle_panel_);
        lv_obj_set_size(row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_gap(row, 15, 0);

        idle_icon_label_ = lv_label_create(row);
        lv_obj_set_style_text_font(idle_icon_label_, &font_awesome_30_4, 0);
        lv_obj_set_style_text_color(idle_icon_label_, lv_color_hex(0xFFD700), 0);

        idle_temp_label_ = lv_label_create(row);
        lv_obj_set_style_text_font(idle_temp_label_, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(idle_temp_label_, lv_color_white(), 0);
    }

    // Detail label (scrolling text)
    if (!idle_detail_label_) {
        idle_detail_label_ = lv_label_create(idle_panel_);
        // lv_obj_set_style_text_font(idle_detail_label_, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(idle_detail_label_, lv_color_hex(0x00FFC2), 0);
        lv_label_set_long_mode(idle_detail_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_width(idle_detail_label_, LV_PCT(95));
        lv_obj_set_style_text_align(idle_detail_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_anim_duration(idle_detail_label_, 30000, 0);
        lv_obj_set_style_margin_top(idle_detail_label_, 10, 0);
    }
    
    lv_obj_move_to_index(idle_detail_label_, -1);

    // City label
    if (!idle_city_label_) {
        idle_city_label_ = lv_label_create(idle_panel_);
        // lv_obj_set_style_text_font(idle_city_label_, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(idle_city_label_, lv_color_white(), 0);
        lv_obj_set_style_margin_top(idle_city_label_, 15, 0);
    }
    
    lv_obj_move_to_index(idle_city_label_, -1);

    ESP_LOGI(TAG, "Weather UI initialized");
}

void WeatherUI::ShowIdleCard(const IdleCardInfo& info) {
#ifndef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    return;
#endif

    if (!idle_panel_) {
        ESP_LOGE(TAG, "Idle panel not initialized");
        return;
    }

    // Update time
    if (idle_time_label_ && !info.time_text.empty()) {
        lv_label_set_text(idle_time_label_, info.time_text.c_str());
    }

    // Update date
    if (idle_date_label_ && !info.date_text.empty()) {
        lv_label_set_text(idle_date_label_, info.date_text.c_str());
    }

    // Update icon
    if (idle_icon_label_ && info.icon) {
        lv_label_set_text(idle_icon_label_, info.icon);
    }

    // Update temperature
    if (idle_temp_label_ && !info.temperature_text.empty()) {
        lv_label_set_text(idle_temp_label_, info.temperature_text.c_str());
    }

    // Update city
    if (idle_city_label_ && !info.city.empty()) {
        lv_label_set_text(idle_city_label_, info.city.c_str());
    }

    // Update detail (scrolling text)
    if (idle_detail_label_) {
        std::string detail = "";
        if (!info.description_text.empty()) {
            detail += info.description_text;
        }
        if (!info.humidity_text.empty()) {
            if (!detail.empty()) detail += "  |  ";
            detail += "Độ ẩm: " + info.humidity_text;
        }
        if (!info.feels_like_text.empty()) {
            if (!detail.empty()) detail += "  |  ";
            detail += info.feels_like_text;
        }
        if (!info.wind_text.empty()) {
            if (!detail.empty()) detail += "  |  ";
            detail += info.wind_text;
        }
        if (!info.pressure_text.empty()) {
            if (!detail.empty()) detail += "  |  ";
            detail += info.pressure_text;
        }
        
        lv_label_set_text(idle_detail_label_, detail.c_str());
    }

    // Show panel
    lv_obj_remove_flag(idle_panel_, LV_OBJ_FLAG_HIDDEN);
    
    // ESP_LOGI(TAG, "Weather card shown: %s, %s", info.city.c_str(), info.temperature_text.c_str());
}

void WeatherUI::HideIdleCard() {
    if (idle_panel_) {
        lv_obj_add_flag(idle_panel_, LV_OBJ_FLAG_HIDDEN);
    }
}

void WeatherUI::UpdateIdleDisplay(const WeatherInfo& weather_info) {
#ifndef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    return;
#endif

    IdleCardInfo card;
    
    // Get system time
    time_t now = time(nullptr);
    struct tm tm_buf;
    if (localtime_r(&now, &tm_buf) != nullptr) {
        char buffer[32];
        strftime(buffer, sizeof(buffer), "%H:%M", &tm_buf);
        card.time_text = buffer;
        
        strftime(buffer, sizeof(buffer), "%d-%m-%Y", &tm_buf);
        card.date_text = buffer;
        
        strftime(buffer, sizeof(buffer), "%A", &tm_buf);
        card.day_text = buffer;
    }

    // Weather data
    if (weather_info.valid) {
        card.city = weather_info.city;
        
        char temp_buf[16];
        snprintf(temp_buf, sizeof(temp_buf), "%d°C", (int)round(weather_info.temp));
        card.temperature_text = temp_buf;

        card.description_text = weather_info.description;
        card.humidity_text = std::to_string(weather_info.humidity) + "%";

        char extra_buf[32];
        snprintf(extra_buf, sizeof(extra_buf), "Nhiệt: %d°C", (int)round(weather_info.feels_like));
        card.feels_like_text = extra_buf;
        
        snprintf(extra_buf, sizeof(extra_buf), "Gió: %.1f m/s", weather_info.wind_speed);
        card.wind_text = extra_buf;
        
        snprintf(extra_buf, sizeof(extra_buf), "P: %d hPa", weather_info.pressure);
        card.pressure_text = extra_buf;

        card.icon = GetWeatherIcon(weather_info.icon_code);
    } else {
        card.city = "Connecting...";
        card.temperature_text = "--";
        card.icon = FONT_AWESOME_WIFI;
    }

    ShowIdleCard(card);
}
