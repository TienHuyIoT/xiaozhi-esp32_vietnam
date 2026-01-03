// --- [DIENBIEN MOD] ---
#include "weather_ui.h"
#include "board.h" 
#include <esp_log.h>
#include <time.h>
#include <cmath>
#include <font_awesome.h>
#include <cstdio>

// --- ICONS DEFINE ---
#ifndef FONT_AWESOME_EARTH_ASIA
#define FONT_AWESOME_EARTH_ASIA "\uf57e"
#endif
#ifndef FONT_AWESOME_GEARS
#define FONT_AWESOME_GEARS "\uf085"
#endif
#ifndef FONT_AWESOME_BOLT
#define FONT_AWESOME_BOLT "\uf0e7"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TAG "WeatherUI"

// External font declarations
LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(lv_font_montserrat_14);
LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(lv_font_montserrat_28);
LV_FONT_DECLARE(lv_font_ds_digitb_48);

// Khai báo hình ảnh background
LV_IMG_DECLARE(bg_weather);

// --- COLOR PALETTE ---
#define COLOR_BG            lv_color_hex(0x000000)

// Màu sắc
#define COLOR_NEON_GREEN    lv_color_hex(0x39FF14)
#define COLOR_LIME_GREEN    lv_color_hex(0xCCFF00)
#define COLOR_ORANGE        lv_color_hex(0xFFA500)
#define COLOR_YELLOW        lv_color_hex(0xFFFF00)
#define COLOR_WHITE         lv_color_hex(0xFFFFFF)
#define COLOR_GREY          lv_color_hex(0x808080)
#define COLOR_RED           lv_color_hex(0xFF0000)

// Màu kim đồng hồ
#define COLOR_HAND_HOUR     COLOR_LIME_GREEN
#define COLOR_HAND_MIN      COLOR_WHITE
#define COLOR_HAND_SEC      COLOR_RED

WeatherUI::WeatherUI() : container_(nullptr), screen_width_(0), screen_height_(0) {}

WeatherUI::~WeatherUI() {
    if (container_) {
        lv_obj_del(container_);
        container_ = nullptr;
    }
}

void WeatherUI::SetupIdleUI(lv_obj_t* parent, int screen_width, int screen_height) {
    screen_width_ = screen_width;
    screen_height_ = screen_height;

    // Tính toán tỉ lệ so với màn hình chuẩn 240x240
    // Nếu màn hình là 240x240 thì ratio = 1.0
    // Nếu màn hình 320x240 thì ratio_w = 1.33, ratio_h = 1.0
    float ratio_w = (float)screen_width / 240.0f;
    float ratio_h = (float)screen_height / 240.0f;
    float ratio_avg = (ratio_w + ratio_h) / 2.0f; // Tỉ lệ trung bình cho các object tròn

    // 1. Main Container
    container_ = lv_obj_create(parent);
    lv_obj_set_size(container_, screen_width, screen_height);
    lv_obj_center(container_);
    lv_obj_set_style_pad_all(container_, 0, 0);
    lv_obj_set_style_bg_color(container_, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(container_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_scrollbar_mode(container_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(container_, LV_OBJ_FLAG_SCROLLABLE);

    // 2. Background Image (Co giãn tự động)
    lv_obj_t* bg_img = lv_img_create(container_);
    lv_img_set_src(bg_img, &bg_weather);
    lv_obj_center(bg_img);

    // Tính toán Zoom cho ảnh
    // LVGL Zoom: 256 là tỉ lệ 1:1 (100%)
    // Giả sử ảnh gốc bg_weather thiết kế cho 240px
    int img_src_w = 240; 
    int zoom_val = (int)((float)screen_width / img_src_w * 256);
    lv_img_set_zoom(bg_img, zoom_val);

    
    // --- HÀNG 1: NGÀY THÁNG NĂM (Xanh Neon) ---
    label_date_ = lv_label_create(container_);
    lv_obj_set_style_text_font(label_date_, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_date_, COLOR_NEON_GREEN, 0);
    lv_label_set_text(label_date_, "--/--/----");
    lv_obj_align(label_date_, LV_ALIGN_TOP_MID, 0, 30);

    // --- HÀNG 2: ĐỒNG HỒ KỸ THUẬT SỐ (Màu Cam) ---
    label_time_ = lv_label_create(container_);
    lv_obj_set_style_text_font(label_time_, &lv_font_ds_digitb_48, 0); 
    lv_obj_set_style_text_color(label_time_, COLOR_ORANGE, 0);
    lv_label_set_text(label_time_, "00:00");
    lv_obj_align(label_time_, LV_ALIGN_TOP_MID, 0, 55);

    // --- HÀNG 3: ICON & NHIỆT ĐỘ (Màu Vàng) ---
    // Icon (Bên trái)
    icon_weather_main_ = lv_label_create(container_);
    lv_obj_set_style_text_font(icon_weather_main_, &font_awesome_30_4, 0);
    lv_obj_set_style_text_color(icon_weather_main_, COLOR_YELLOW, 0);
    lv_label_set_text(icon_weather_main_, FONT_AWESOME_EARTH_ASIA);
    lv_obj_align(icon_weather_main_, LV_ALIGN_CENTER, -55, -10); 

    // Nhiệt độ (Bên phải)
    label_temp_ = lv_label_create(container_);
    lv_obj_set_style_text_font(label_temp_, &lv_font_montserrat_20, 0); 
    lv_obj_set_style_text_color(label_temp_, COLOR_YELLOW, 0);
    lv_label_set_text(label_temp_, "--°C");
    lv_obj_align(label_temp_, LV_ALIGN_CENTER, 55, -10);

    // --- HÀNG 4: THÀNH PHỐ (Màu Trắng) ---
    label_city_ = lv_label_create(container_);
    lv_obj_set_style_text_font(label_city_, &font_puhui_14_1, 0);
    lv_obj_set_style_text_color(label_city_, COLOR_WHITE, 0);
    lv_label_set_text(label_city_, "Loading...");
    lv_obj_align(label_city_, LV_ALIGN_CENTER, 0, 25);

    // --- HÀNG 5: THÔNG TIN CHI TIẾT (Scroll) ---

    label_humidity_ = lv_label_create(container_);
    lv_obj_set_style_text_font(label_humidity_, &font_puhui_14_1, 0);
    lv_obj_set_style_text_color(label_humidity_, lv_color_hex(0x00FFC2), 0);
       
    
    // Giới hạn chiều rộng để chữ chạy
    lv_obj_set_width(label_humidity_, 160); 
    lv_label_set_long_mode(label_humidity_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(label_humidity_, LV_TEXT_ALIGN_CENTER, 0);
    
    lv_label_set_text(label_humidity_, "...");
    lv_obj_align(label_humidity_, LV_ALIGN_CENTER, 0, 45);

    // --- BRAND: "Xiaozhi AI-IoT 🇻🇳" (Màu xám) ---
    label_brand_ = lv_label_create(container_);
    lv_obj_set_style_text_font(label_brand_, &font_puhui_14_1, 0);
    lv_obj_set_style_text_color(label_brand_, COLOR_GREY, 0);
    lv_label_set_text(label_brand_, "Xiaozhi AI-IoT 🇻🇳");
    lv_obj_align(label_brand_, LV_ALIGN_CENTER, 0, 70);

    // 4. Khởi tạo Kim đồng hồ & ARC
    CreateCenterClock(container_, screen_width, screen_height, ratio_avg);
}


void WeatherUI::CreateCenterClock(lv_obj_t* parent, int w, int h, float ratio) {
    // --- HANDS (KIM ĐỒNG HỒ) ---
    // Bán kính = min(w, h) / 2
    int radius = std::min(w, h) / 2;

    // --- CẤU HÌNH ĐỘ DÀI ĐUÔI KIM (Phần thừa ra phía sau tâm) ---
    // Để kim trông thực tế, cần có một đoạn đuôi ngắn qua tâm
    int tail_len = (int)(15 * ratio); 

    // 2. Kim Giờ
    hand_hour_ = lv_obj_create(parent);
    
    int w_hour = (int)(6 * ratio);
    int h_hour = (int)(radius * 0.55); // Tổng chiều dài kim giờ
    int p_hour = h_hour - tail_len;    // Điểm xoay nằm cách đáy một đoạn bằng đuôi kim

    lv_obj_set_size(hand_hour_, w_hour, h_hour);
    lv_obj_set_style_bg_color(hand_hour_, COLOR_HAND_HOUR, 0);
    lv_obj_set_style_radius(hand_hour_, 3, 0);
    lv_obj_set_style_border_width(hand_hour_, 0, 0);
    
    // Pivot X: Chính giữa chiều ngang kim
    lv_obj_set_style_transform_pivot_x(hand_hour_, w_hour / 2, 0); 
    // Pivot Y: Tại vị trí đã tính toán
    lv_obj_set_style_transform_pivot_y(hand_hour_, p_hour, 0); 
    
    // [QUAN TRỌNG] Căn chỉnh Offset Y theo công thức chuẩn: (Height / 2) - Pivot_Y
    lv_obj_align(hand_hour_, LV_ALIGN_CENTER, 0, (h_hour / 2) - p_hour);


    // 3. Kim Phút
    hand_min_ = lv_obj_create(parent);
    
    int w_min = (int)(4 * ratio);
    int h_min = (int)(radius * 0.75); // Kim phút dài hơn
    int p_min = h_min - tail_len;     // Điểm xoay

    lv_obj_set_size(hand_min_, w_min, h_min);
    lv_obj_set_style_bg_color(hand_min_, COLOR_HAND_MIN, 0);
    lv_obj_set_style_radius(hand_min_, 2, 0);
    lv_obj_set_style_border_width(hand_min_, 0, 0);
    
    lv_obj_set_style_transform_pivot_x(hand_min_, w_min / 2, 0);
    lv_obj_set_style_transform_pivot_y(hand_min_, p_min, 0);
    
    // Căn chỉnh Offset Y
    lv_obj_align(hand_min_, LV_ALIGN_CENTER, 0, (h_min / 2) - p_min);


    // 4. Kim Giây
    hand_sec_ = lv_obj_create(parent);
    
    int w_sec = (int)(2 * ratio);
    int h_sec = (int)(radius * 0.85); // Kim giây dài nhất
    int p_sec = h_sec - (int)(20 * ratio); // Đuôi kim giây thường dài hơn chút (20px)

    lv_obj_set_size(hand_sec_, w_sec, h_sec);
    lv_obj_set_style_bg_color(hand_sec_, COLOR_HAND_SEC, 0);
    lv_obj_set_style_radius(hand_sec_, 1, 0);
    lv_obj_set_style_border_width(hand_sec_, 0, 0);
    
    lv_obj_set_style_transform_pivot_x(hand_sec_, w_sec / 2, 0);
    lv_obj_set_style_transform_pivot_y(hand_sec_, p_sec, 0); 
    
    // Căn chỉnh Offset Y
    lv_obj_align(hand_sec_, LV_ALIGN_CENTER, 0, (h_sec / 2) - p_sec);


    // 5. Center Point (Trục kim)
    center_point_ = lv_obj_create(parent);
    lv_coord_t point_size = (lv_coord_t)(8 * ratio);
    lv_obj_set_size(center_point_, point_size, point_size);
    lv_obj_set_style_radius(center_point_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(center_point_, COLOR_RED, 0);
    lv_obj_center(center_point_);
}

const char* WeatherUI::GetWeatherIcon(const std::string& code) {
    if (code == "01d" || code == "01n") return "\uf185"; // Sun
    if (code == "02d" || code == "02n") return "\uf6c4"; // Cloud Sun
    if (code == "03d" || code == "03n" || code == "04d" || code == "04n") return "\uf0c2"; // Cloud
    if (code == "09d" || code == "09n" || code == "10d" || code == "10n") return "\uf740"; // Cloud Rain
    if (code == "11d" || code == "11n") return "\uf0e7"; // Bolt
    if (code == "13d" || code == "13n") return "\uf2dc"; // Snowflake
    if (code == "50d" || code == "50n") return "\uf75f"; // Smog
    return FONT_AWESOME_EARTH_ASIA;
}

void WeatherUI::ShowIdleCard(const IdleCardInfo& info) {
    if (!container_) return;
    
    // Update Text Data
    if (label_city_) lv_label_set_text(label_city_, info.city.c_str());
    if (label_time_) lv_label_set_text(label_time_, info.time_text.c_str());
    if (label_date_) lv_label_set_text(label_date_, info.date_text.c_str());
    if (label_temp_) lv_label_set_text(label_temp_, info.temperature_text.c_str());
    
    if (info.icon && icon_weather_main_) {
        lv_label_set_text(icon_weather_main_, info.icon);
    }
    
    // Update Clock Hands & ARC Rotation
    time_t now = time(nullptr);
    struct tm tm_buf;
    if (localtime_r(&now, &tm_buf) != nullptr) {
        int32_t angle_sec = tm_buf.tm_sec * 60; 
        int32_t angle_min = tm_buf.tm_min * 60 + tm_buf.tm_sec; 
        int32_t angle_hour = (tm_buf.tm_hour % 12) * 300 + (tm_buf.tm_min * 5); 

        if (hand_sec_) lv_obj_set_style_transform_rotation(hand_sec_, angle_sec, 0);
        if (hand_min_) lv_obj_set_style_transform_rotation(hand_min_, angle_min, 0);
        if (hand_hour_) lv_obj_set_style_transform_rotation(hand_hour_, angle_hour, 0);

    }
    // Update detail (scrolling text)
    if (label_humidity_) {
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
        
        lv_label_set_text(label_humidity_, detail.c_str());
    }

    lv_obj_remove_flag(container_, LV_OBJ_FLAG_HIDDEN);
}

void WeatherUI::HideIdleCard() {
    if (container_) lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
}

void WeatherUI::UpdateIdleDisplay(const WeatherInfo& weather_info) {
#ifndef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    return;
#endif
    IdleCardInfo card;
    
    time_t now = time(nullptr);
    struct tm tm_buf;
    if (localtime_r(&now, &tm_buf) != nullptr) {
        char buffer[32];
        strftime(buffer, sizeof(buffer), "%H:%M", &tm_buf);
        card.time_text = buffer;
        
        strftime(buffer, sizeof(buffer), "%d/%m/%Y", &tm_buf);
        card.date_text = buffer;
    }
    
    if (weather_info.valid) {
        card.city = weather_info.city;
        
        char temp_buf[16];
        snprintf(temp_buf, sizeof(temp_buf), "%d°C", (int)round(weather_info.temp));
        card.temperature_text = temp_buf;

        card.description_text = weather_info.description;
        card.humidity_text = std::to_string(weather_info.humidity) + "%";

        char extra_buf[32];
        snprintf(extra_buf, sizeof(extra_buf), "Cảm giác như: %d°C", (int)round(weather_info.feels_like));
        card.feels_like_text = extra_buf;
        
        snprintf(extra_buf, sizeof(extra_buf), "Gió: %.1f m/s", weather_info.wind_speed);
        card.wind_text = extra_buf;
        
        snprintf(extra_buf, sizeof(extra_buf), "Áp suất: %d hPa", weather_info.pressure);
        card.pressure_text = extra_buf;

        card.icon = GetWeatherIcon(weather_info.icon_code);
    } else {
        card.city = "Connecting...";
        card.temperature_text = "--";
        card.icon = FONT_AWESOME_WIFI;
    }
    
    ShowIdleCard(card);
}