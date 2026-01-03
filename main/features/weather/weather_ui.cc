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
LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(lv_font_montserrat_14);
LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(lv_font_montserrat_28);
LV_FONT_DECLARE(lv_font_ds_digitb_48);

// --- COLOR PALETTE ---
#define COLOR_BG            lv_color_hex(0x000000)

// Màu chi tiết
#define COLOR_TICK_HOUR     lv_color_hex(0xCCFF00) // Dạ quang
#define COLOR_TICK_MIN      lv_color_hex(0x555555) // Xám nhạt
#define COLOR_TIME_ORANGE   lv_color_hex(0xFF6600) // Cam đậm
#define COLOR_TEXT_WHITE    lv_color_hex(0xFFFFFF)
#define COLOR_TEXT_GRAY     lv_color_hex(0xAAAAAA)
#define COLOR_NEON_BLUE     lv_color_hex(0x00FFFF)
#define COLOR_TEXT_YELLOW   lv_color_hex(0xFFFF00) 

// Màu kim đồng hồ
#define COLOR_HAND_HOUR     lv_color_hex(0xFFFFFF)
#define COLOR_HAND_MIN      lv_color_hex(0xFFFF00)  //0xCCCCCC đổi từ xám sang sáng vàng
#define COLOR_HAND_SEC      lv_color_hex(0xFF0000) 

WeatherUI::WeatherUI() 
    : container_(nullptr), label_city_(nullptr), label_time_(nullptr), label_date_(nullptr)
    , clock_arc_(nullptr), hand_hour_(nullptr), hand_min_(nullptr), hand_sec_(nullptr), center_point_(nullptr)
    , icon_weather_main_(nullptr), label_temp_(nullptr)
    , label_brand_(nullptr), label_humidity_(nullptr)
    , screen_width_(0), screen_height_(0) {}

WeatherUI::~WeatherUI() { HideIdleCard(); }

const char* WeatherUI::GetWeatherIcon(const std::string& code) {        //icon thời tiết
    if (code.size() < 2) return FONT_AWESOME_CLOUD;
    std::string prefix = code.substr(0, 2);
    if (prefix == "01") return FONT_AWESOME_SUN;
    if (prefix == "09" || prefix == "10") return FONT_AWESOME_CLOUD_RAIN;
    if (prefix == "11") return FONT_AWESOME_BOLT;
    if (prefix == "50") return FONT_AWESOME_SMOG;
    if (prefix == "13") return FONT_AWESOME_SNOWFLAKE;
    return FONT_AWESOME_CLOUD;
}

void WeatherUI::SetupIdleUI(lv_obj_t* parent, int screen_width, int screen_height) {
#ifndef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    return;
#endif
    screen_width_ = screen_width;
    screen_height_ = screen_height;
    if (!parent) return;
    if (container_) return;

    // 1. Main Container
    container_ = lv_obj_create(parent);
    lv_obj_set_size(container_, screen_width, screen_height);
    lv_obj_set_style_bg_color(container_, COLOR_BG, 0);
    lv_obj_set_style_pad_all(container_, 0, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_style_radius(container_, 0, 0);
    lv_obj_center(container_);
    lv_obj_set_scrollbar_mode(container_, LV_SCROLLBAR_MODE_OFF);
    
    // 2. Bezel (Vạch chia chính xác)
    CreateBezelTicksAndNumbers(container_, screen_width, screen_height);

    // 3. Center Clock (Arc nhỏ + Kim)
    CreateCenterClock(container_, screen_width, screen_height);

    // 4. Các khu vực thông tin
    CreateTopSection(container_);
    CreateMiddleSection(container_);
    CreateCitySection(container_);
    CreateBottomSection(container_);
    
    lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "Weather UI (Corrected Bezel) initialized");
}

// Hàm tính tọa độ điểm trên hình chữ nhật tương ứng với góc phút (0-59)
void WeatherUI::GetBezelXY(int minute_idx, int w, int h, int margin, int& out_x, int& out_y) {
    // Chuyển đổi phút sang góc (0 phút = -90 độ hay 270 độ trong tọa độ cực, nhưng ở đây ta tính theo 12h là 0 độ)
    // 1 phút = 6 độ.
    // Góc theta tính từ trục Y âm (12h) theo chiều kim đồng hồ.
    float angle = (minute_idx * 6.0f) * (M_PI / 180.0f);
    
    int cx = w / 2;
    int cy = h / 2;
    
    // Vector hướng từ tâm ra
    float vx = sin(angle);
    float vy = -cos(angle);

    // Tìm giao điểm với các cạnh hình chữ nhật (đã trừ margin)
    // Cạnh: x = margin, x = w-margin, y = margin, y = h-margin
    int box_w = w - 2 * margin;
    int box_h = h - 2 * margin;
    
    // Khoảng cách từ tâm đến cạnh
    float dx = box_w / 2.0f;
    float dy = box_h / 2.0f;

    // Ray casting đơn giản để tìm tỉ lệ t
    // t_x là khoảng cách để chạm cạnh dọc, t_y là khoảng cách chạm cạnh ngang
    float t_x = (vx != 0) ? fabs(dx / vx) : 100000.0f;
    float t_y = (vy != 0) ? fabs(dy / vy) : 100000.0f;

    // Chọn cái ngắn hơn (nghĩa là chạm cạnh đó trước)
    float t = (t_x < t_y) ? t_x : t_y;

    out_x = (int)(cx + vx * t);
    out_y = (int)(cy + vy * t);
}

void WeatherUI::CreateBezelTicksAndNumbers(lv_obj_t* parent, int w, int h) {
    int margin = 3; // Khoảng cách từ viền ngoài vào
    int tick_len_hour = 12;
    int tick_len_min = 6;
    int tick_w = 3;

    // Vẽ 60 vạch chia
    for (int i = 0; i < 60; i++) {
        // Bỏ qua vạch 0, 15, 30, 45 (Vì đã có số)
        if (i == 0 || i == 15 || i == 30 || i == 45) {
            continue;
        }

        bool is_hour = (i % 5 == 0);
        int x, y;
        
        // Tính vị trí vạch nằm trên đường viền
        GetBezelXY(i, w, h, margin, x, y);

        lv_obj_t* tick = lv_obj_create(parent);
        lv_obj_set_size(tick, tick_w, tick_w); // Vẽ dạng điểm/vuông nhỏ vì vạch dài sẽ cần xoay
        
        // Nếu muốn vạch dài, ta cần xoay tick hoặc dùng lv_line. 
        // Ở đây để đơn giản và hiệu quả, ta dùng hình vuông nhỏ cho vạch phút,
        // và hình chữ nhật dài (nhưng không xoay) cho vạch giờ ở 4 góc chính.
        // Hoặc vẽ tick hướng vào tâm:
        
        if (is_hour) {
             lv_obj_set_size(tick, 6, 6); // Điểm giờ to hơn
             lv_obj_set_style_radius(tick, 3, 0);
             lv_obj_set_style_bg_color(tick, COLOR_TICK_HOUR, 0);
        } else {
             lv_obj_set_size(tick, 2, 2); // Điểm phút nhỏ
             lv_obj_set_style_radius(tick, 1, 0);
             lv_obj_set_style_bg_color(tick, COLOR_TICK_MIN, 0);
        }
        
        lv_obj_set_style_border_width(tick, 0, 0);
        // Đặt tâm của tick vào tọa độ tính được
        lv_obj_set_pos(tick, x - lv_obj_get_width(tick)/2, y - lv_obj_get_height(tick)/2);
    }

    // Số 12, 3, 6, 9 (Dời vào trong một chút so với vạch)
    int num_margin = margin + 0; //+15
    lv_obj_t* lbl_12 = lv_label_create(parent); lv_label_set_text(lbl_12, "12");
    lv_obj_set_style_text_color(lbl_12, COLOR_TICK_HOUR, 0);
    lv_obj_set_style_text_font(lbl_12, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_12, LV_ALIGN_TOP_MID, 0, num_margin);

    lv_obj_t* lbl_6 = lv_label_create(parent); lv_label_set_text(lbl_6, "6");
    lv_obj_set_style_text_color(lbl_6, COLOR_TICK_HOUR, 0);
    lv_obj_set_style_text_font(lbl_6, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_6, LV_ALIGN_BOTTOM_MID, 0, -num_margin);

    lv_obj_t* lbl_3 = lv_label_create(parent); lv_label_set_text(lbl_3, "3");
    lv_obj_set_style_text_color(lbl_3, COLOR_TICK_HOUR, 0);
    lv_obj_set_style_text_font(lbl_3, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_3, LV_ALIGN_RIGHT_MID, -num_margin, 0);

    lv_obj_t* lbl_9 = lv_label_create(parent); lv_label_set_text(lbl_9, "9");
    lv_obj_set_style_text_color(lbl_9, COLOR_TICK_HOUR, 0);
    lv_obj_set_style_text_font(lbl_9, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_9, LV_ALIGN_LEFT_MID, num_margin, 0);
}

void WeatherUI::CreateCenterClock(lv_obj_t* parent, int w, int h) {
    // 1. Vòng tròn (Arc) - THU NHỎ (48x48)
    clock_arc_ = lv_arc_create(parent);
    lv_obj_set_size(clock_arc_, 48, 48); // [CHANGE] Giảm kích thước ~1/3 so với 70
    lv_arc_set_rotation(clock_arc_, 270);
    lv_arc_set_bg_angles(clock_arc_, 0, 360);
    lv_arc_set_value(clock_arc_, 0);
    lv_obj_remove_style(clock_arc_, NULL, LV_PART_KNOB);
    lv_obj_set_style_arc_color(clock_arc_, COLOR_NEON_BLUE, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(clock_arc_, 3, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(clock_arc_, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_center(clock_arc_);

    // Icon bánh răng (Nhỏ lại theo Arc)
    lv_obj_t* gear = lv_label_create(clock_arc_);
    lv_obj_set_style_text_font(gear, &lv_font_montserrat_14, 0); // Giảm font icon
    lv_label_set_text(gear, FONT_AWESOME_GEARS);
    lv_obj_set_style_text_color(gear, lv_color_hex(0x333333), 0);
    lv_obj_center(gear);

    // 2. Kim Đồng Hồ
    // Kim Giờ
    hand_hour_ = lv_obj_create(parent);
    lv_obj_set_size(hand_hour_, 5, 40); // Ngắn lại chút cho hợp vòng tròn nhỏ
    lv_obj_set_style_bg_color(hand_hour_, COLOR_HAND_HOUR, 0);
    lv_obj_set_style_border_width(hand_hour_, 0, 0);
    lv_obj_set_style_radius(hand_hour_, 3, 0);
    lv_obj_set_style_transform_pivot_x(hand_hour_, LV_PCT(50), 0);
    lv_obj_set_style_transform_pivot_y(hand_hour_, LV_PCT(100), 0);
    lv_obj_align(hand_hour_, LV_ALIGN_CENTER, 0, -20);

    // Kim Phút
    hand_min_ = lv_obj_create(parent);
    lv_obj_set_size(hand_min_, 3, 65); 
    lv_obj_set_style_bg_color(hand_min_, COLOR_HAND_MIN, 0);
    lv_obj_set_style_border_width(hand_min_, 0, 0);
    lv_obj_set_style_radius(hand_min_, 2, 0);
    lv_obj_set_style_transform_pivot_x(hand_min_, LV_PCT(50), 0);
    lv_obj_set_style_transform_pivot_y(hand_min_, LV_PCT(100), 0);
    lv_obj_align(hand_min_, LV_ALIGN_CENTER, 0, -32);

    // Kim Giây
    hand_sec_ = lv_obj_create(parent);
    lv_obj_set_size(hand_sec_, 2, 80); 
    lv_obj_set_style_bg_color(hand_sec_, COLOR_HAND_SEC, 0);
    lv_obj_set_style_border_width(hand_sec_, 0, 0);
    lv_obj_set_style_radius(hand_sec_, 1, 0);
    lv_obj_set_style_transform_pivot_x(hand_sec_, LV_PCT(50), 0);
    lv_obj_set_style_transform_pivot_y(hand_sec_, LV_PCT(100), 0);
    lv_obj_align(hand_sec_, LV_ALIGN_CENTER, 0, -40);

    // Chấm tâm
    center_point_ = lv_obj_create(parent);
    lv_obj_set_size(center_point_, 8, 8);
    lv_obj_set_style_bg_color(center_point_, COLOR_HAND_SEC, 0);
    lv_obj_set_style_radius(center_point_, 8, 0);
    lv_obj_center(center_point_);
}

// --- TOP SECTION: Date & Time ---
void WeatherUI::CreateTopSection(lv_obj_t* parent) {

    // 1. Xiaozhi VN (Top)
    label_brand_ = lv_label_create(parent);
    lv_label_set_text(label_brand_, "Xiaozhi AI-IoT 🇻🇳");
    lv_obj_set_style_text_font(label_brand_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_brand_, COLOR_TEXT_GRAY, 0);
    lv_obj_align(label_brand_, LV_ALIGN_TOP_MID, 0, 30);

    // 2. Ngày tháng (Dưới)
    label_date_ = lv_label_create(parent);
    lv_label_set_text(label_date_, "Mon 01/01"); 
    lv_obj_set_style_text_font(label_date_, &lv_font_montserrat_28, 0); // font chữ &lv_font_montserrat_20  lv_font_ds_digitb_48
    lv_obj_set_style_text_color(label_date_, COLOR_TIME_ORANGE, 0); //Mầu cũ COLOR_NEON_BLUE
    lv_obj_align(label_date_, LV_ALIGN_TOP_MID, 0, 55);     //cũ là 35, giờ đổi thành 50 nó dịch xuống dưới 

    /* 3. Giờ số (Ngay dưới ngày)
    label_time_ = lv_label_create(parent);
    lv_label_set_text(label_time_, "00:00:00");
    lv_obj_set_style_text_font(label_time_, &lv_font_ds_digitb_48, 0);
    lv_obj_set_style_text_color(label_time_, COLOR_TIME_ORANGE, 0);
    lv_obj_align(label_time_, LV_ALIGN_TOP_MID, 0, 60);
    */

}

// --- MIDDLE SECTION: Icon (Left) - Temp (Right) đối xứng qua tâm ---
void WeatherUI::CreateMiddleSection(lv_obj_t* parent) {
    int x_offset = 60; // Khoảng cách từ tâm ra 2 bên
    int y_pos = 0;     // Ngang hàng với tâm

    // 1. Icon thời tiết (Bên trái)
    icon_weather_main_ = lv_label_create(parent);
    lv_obj_set_style_text_font(icon_weather_main_, &font_awesome_30_4, 0);
    lv_label_set_text(icon_weather_main_, FONT_AWESOME_CLOUD);
    lv_obj_set_style_text_color(icon_weather_main_, COLOR_TEXT_WHITE, 0);
    lv_obj_align(icon_weather_main_, LV_ALIGN_CENTER, -x_offset, y_pos);

    // 2. Nhiệt độ (Bên phải - [CHANGE] Đưa lên ngang hàng icon)
    label_temp_ = lv_label_create(parent);
    lv_label_set_text(label_temp_, "--C");
    lv_obj_set_style_text_font(label_temp_, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_temp_, COLOR_TEXT_WHITE, 0); //COLOR_TEXT_YELLOW
    lv_obj_align(label_temp_, LV_ALIGN_CENTER, x_offset, y_pos);
}

// --- CITY SECTION: Dưới vòng tròn ---
void WeatherUI::CreateCitySection(lv_obj_t* parent) {
    // [CHANGE] Đưa thành phố xuống dưới vòng tròn trung tâm
    label_city_ = lv_label_create(parent);
    lv_label_set_text(label_city_, "City Name");
    lv_obj_set_style_text_color(label_city_, COLOR_TEXT_GRAY, 0);  //  COLOR_TEXT_WHITE
    //lv_obj_set_style_text_font(label_city_, &lv_font_montserrat_20, 0);
    // Vị trí: Dưới tâm khoảng 50px (để không chạm kim đồng hồ)
    lv_obj_align(label_city_, LV_ALIGN_CENTER, 0, 40);
}

// --- BOTTOM SECTION: Brand - Humidity ---
void WeatherUI::CreateBottomSection(lv_obj_t* parent) {
    int bottom_margin = -35;

    /*
    // 1. Xiaozhi VN (Bên trái)
    label_brand_ = lv_label_create(parent);
    lv_label_set_text(label_brand_, "AI-IoT 🇻🇳");
    lv_obj_set_style_text_font(label_brand_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_brand_, COLOR_TEXT_GRAY, 0);
    lv_obj_align(label_brand_, LV_ALIGN_BOTTOM_LEFT, 50, bottom_margin); //40 dịch lên trên 1 chút so với code gốc
    */

    // 2. Độ ẩm (Bên phải)
    label_humidity_ = lv_label_create(parent);
    lv_label_set_text(label_humidity_, "H: --%");
    lv_obj_set_style_text_font(label_humidity_, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_humidity_, COLOR_NEON_BLUE, 0);
    lv_obj_align(label_humidity_, LV_ALIGN_BOTTOM_RIGHT, -40, bottom_margin);
}

void WeatherUI::ShowIdleCard(const IdleCardInfo& info) {
#ifndef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    return;
#endif
    if (!container_) return;

    if (label_city_) lv_label_set_text(label_city_, info.city.c_str());
    if (label_time_) lv_label_set_text(label_time_, info.time_text.c_str());
    if (label_date_) lv_label_set_text(label_date_, info.date_text.c_str());
    if (label_temp_) lv_label_set_text(label_temp_, info.temperature_text.c_str());
    if (label_humidity_) lv_label_set_text(label_humidity_, info.humidity_text.c_str());

    if (icon_weather_main_ && info.icon) {
        lv_label_set_text(icon_weather_main_, info.icon);
    }
    
    if (clock_arc_) {
        lv_arc_set_value(clock_arc_, (time(NULL) % 60) * 100 / 60); 
    }

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
        strftime(buffer, sizeof(buffer), "%H:%M:%S", &tm_buf);
        card.time_text = buffer;
        
        strftime(buffer, sizeof(buffer), "%a %d/%m", &tm_buf);
        card.date_text = buffer;
    }
    
    if (weather_info.valid) {
        card.city = weather_info.city;
        char temp_buf[16];
        snprintf(temp_buf, sizeof(temp_buf), "%d°C", (int)round(weather_info.temp));
        card.temperature_text = temp_buf;
        
        // [CHANGE] Thêm chữ "H:" vào độ ẩm
        char humid_buf[16];
        snprintf(humid_buf, sizeof(humid_buf), "H:%d%%", weather_info.humidity);
        card.humidity_text = humid_buf;

        card.icon = GetWeatherIcon(weather_info.icon_code);
    } else {
        card.city = "No Data";
        card.temperature_text = "--";
        card.humidity_text = "H: --%";
        card.icon = FONT_AWESOME_WIFI;
    }

    ShowIdleCard(card);
}