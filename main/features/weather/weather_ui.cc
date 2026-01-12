// --- [DIENBIEN MOD] ---
#include "weather_ui.h"
#include "board.h" 
#include <esp_log.h>
#include <time.h>
#include <cstdio>
#include <vector>

// Fonts
LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(lv_font_montserrat_14);
LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(lv_font_montserrat_28);
LV_FONT_DECLARE(lv_font_ds_digitb_48); // Font đồng hồ

// Colors
#define COLOR_BG            lv_color_hex(0x000000)
#define COLOR_NEON_GREEN    lv_color_hex(0x39FF14) // Xanh Neon
#define COLOR_ORANGE        lv_color_hex(0xFFA500) // Cam
#define COLOR_WHITE         lv_color_hex(0xFFFFFF)
#define COLOR_GRAY          lv_color_hex(0x808080)
#define COLOR_CYAN          lv_color_hex(0x00FFFF)
#define COLOR_MAGENTA       lv_color_hex(0xFF00FF)

// Gradient Colors
#define GRAD_START          COLOR_CYAN
#define GRAD_END            COLOR_MAGENTA

WeatherUI::WeatherUI() 
    : container_(nullptr), 
      screen_width_(0), 
      screen_height_(0), 
      cont_clock_(nullptr),
      group_weather_(nullptr),
      label_main_desc_(nullptr),
      group_details_(nullptr)
{
    // Khởi tạo mảng nullptr cho an toàn
    for(int i=0; i<8; i++) lbl_clock_digits_[i] = nullptr;
}

WeatherUI::~WeatherUI() {
    if (container_) {
        lv_obj_del(container_);
        container_ = nullptr;
    }
}

// --- Helpers ---
const char* WeatherUI::GetWeatherIcon(const std::string& code) {
    if (code == "01d" || code == "01n") return "\uf185"; 
    if (code == "02d" || code == "02n") return "\uf6c4"; 
    if (code == "03d" || code == "03n") return "\uf0c2"; 
    if (code == "04d" || code == "04n") return "\uf0c2"; 
    if (code == "09d" || code == "09n") return "\uf740"; 
    if (code == "10d" || code == "10n") return "\uf743"; 
    if (code == "11d" || code == "11n") return "\uf0e7"; 
    return "\uf0c2"; 
}

// Hàm vẽ dãy 9 chấm bi (Dùng chung cho trên và dưới)
lv_obj_t* CreateDotsRow(lv_obj_t* parent, int screen_width, float ratio) {
    int gap_dots = (int)(4 * ratio);
    
    lv_obj_t* dots_cont = lv_obj_create(parent);
    lv_obj_set_size(dots_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(dots_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(dots_cont, 0, 0);
    lv_obj_set_style_pad_all(dots_cont, 0, 0);
    lv_obj_set_flex_flow(dots_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(dots_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(dots_cont, gap_dots, 0); 

    int dot_sizes[] = {2, 3, 4, 5, 6, 5, 4, 3, 2}; 

    for(int i=0; i<9; i++) {
        lv_obj_t* dot = lv_obj_create(dots_cont);
        int s = (int)(dot_sizes[i] * ratio);
        if (s < 2) s = 2;
        lv_obj_set_size(dot, s, s);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, COLOR_NEON_GREEN, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
    }
    return dots_cont;
}

void WeatherUI::CreateGradientBars(lv_obj_t* parent) {
    int bar_thick = screen_width_ * 0.030;
    if (bar_thick < 2) bar_thick = 2;
    int h_3 = screen_height_ / 3;

    static lv_style_t style_grad;
    lv_style_init(&style_grad);
    lv_style_set_bg_opa(&style_grad, LV_OPA_COVER);
    lv_style_set_bg_grad_color(&style_grad, GRAD_END);
    lv_style_set_bg_color(&style_grad, GRAD_START);
    lv_style_set_radius(&style_grad, 0); 

    // Left
    lv_obj_t* line = lv_obj_create(parent);
    lv_obj_set_size(line, bar_thick, h_3);
    lv_obj_align(line, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_style(line, &style_grad, 0);
    lv_obj_set_style_bg_grad_dir(line, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_set_style_radius(line, 0, 0); 

    // Right
    line = lv_obj_create(parent);
    lv_obj_set_size(line, bar_thick, h_3);
    lv_obj_align(line, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_style(line, &style_grad, 0);
    lv_obj_set_style_bg_grad_dir(line, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_set_style_radius(line, 0, 0); 
}

void WeatherUI::CreateDetailArc(lv_obj_t* parent, lv_obj_t** arc_out, lv_obj_t** label_out, lv_color_t color) {
    int box_size = screen_width_ / 6.5; 
    
    lv_obj_t* wrap = lv_obj_create(parent);
    lv_obj_set_size(wrap, box_size, box_size); 
    lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wrap, 0, 0);
    lv_obj_set_style_pad_all(wrap, 0, 0); 
    
    lv_obj_t* arc = lv_arc_create(wrap);
    lv_obj_set_size(arc, box_size - 4, box_size - 4); 
    lv_obj_center(arc);
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_value(arc, 0);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB); 
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(arc, box_size / 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, box_size / 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
    *arc_out = arc;

    *label_out = lv_label_create(arc);
    lv_obj_set_style_text_font(*label_out, &font_puhui_14_1, 0); 
    lv_obj_set_style_text_color(*label_out, COLOR_WHITE, 0);
    lv_label_set_text(*label_out, "-");
    lv_obj_center(*label_out);
    
    if (screen_width_ < 240) {
        lv_obj_set_style_transform_zoom(*label_out, 200, 0);
    }
}

void WeatherUI::SetupIdleUI(lv_obj_t* parent, int screen_width, int screen_height) {
    screen_width_ = screen_width;
    screen_height_ = screen_height;

    // Tỷ lệ hệ số
    float h_ratio = (float)screen_height / 280.0f;
    float w_ratio = (float)screen_width / 240.0f;
    int zoom_std = (int)(256 * w_ratio);
    
    // Đệm an toàn để tránh cụt chân chữ (g, y, p...)
    int safe_pad_text = (int)(4 * h_ratio);
    if (safe_pad_text < 3) safe_pad_text = 3;

    // 1. Container Chính
    container_ = lv_obj_create(parent);
    lv_obj_set_size(container_, screen_width, screen_height);
    lv_obj_center(container_);
    lv_obj_set_style_bg_color(container_, COLOR_BG, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_style_radius(container_, 0, 0); 
    lv_obj_set_style_pad_all(container_, 0, 0); 
    lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(container_, LV_OBJ_FLAG_SCROLLABLE);

    CreateGradientBars(container_);

    // --- MAIN LAYOUT ---
    lv_obj_t* main_col = lv_obj_create(container_);
    lv_obj_set_size(main_col, lv_pct(100), lv_pct(100));
    lv_obj_clear_flag(main_col, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(main_col, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(main_col, 0, 0);
    
    lv_obj_set_flex_flow(main_col, LV_FLEX_FLOW_COLUMN);
    // Căn đều dọc (Space Between) và Căn giữa ngang (Center)
    lv_obj_set_flex_align(main_col, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // Margin nhẹ 2 đầu
    int margin_v = (int)(screen_height * 0.02); 
    lv_obj_set_style_pad_ver(main_col, margin_v, 0);
    lv_obj_set_style_pad_hor(main_col, 0, 0);
    lv_obj_set_style_pad_row(main_col, 0, 0);

    // --- HÀNG 0: CHẤM BI TRÊN ---
    CreateDotsRow(main_col, screen_width, w_ratio);

    // --- HÀNG 1: HEADER (FIXED: ABSOLUTE CENTER) ---
    lv_obj_t* row_header = lv_obj_create(main_col);
    // Bắt buộc Full Width để tính toán toạ độ chuẩn
    lv_obj_set_width(row_header, lv_pct(100)); 
    // Chiều cao tự động, nhưng cộng thêm pad để không cụt chân
    lv_obj_set_height(row_header, LV_SIZE_CONTENT); 
    lv_obj_set_style_bg_opa(row_header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_header, 0, 0);
    lv_obj_set_style_pad_all(row_header, 0, 0); 
    lv_obj_set_style_pad_ver(row_header, safe_pad_text, 0); 
    
    // [QUAN TRỌNG] Không dùng Flex cho row_header, để dùng Align toạ độ
    lv_obj_clear_flag(row_header, LV_OBJ_FLAG_SCROLLABLE);

    int icon_pad = (int)(10 * w_ratio);
    
    // 1. Wifi (Neo sát Trái)
    label_wifi_icon_ = lv_label_create(row_header);
    lv_obj_set_style_text_font(label_wifi_icon_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_wifi_icon_, COLOR_NEON_GREEN, 0);
    lv_label_set_text(label_wifi_icon_, "\uf1eb");
    lv_obj_align(label_wifi_icon_, LV_ALIGN_LEFT_MID, icon_pad, 0);
    lv_obj_set_style_transform_zoom(label_wifi_icon_, zoom_std, 0);

    // 2. Battery (Neo sát Phải)
    label_bat_icon_ = lv_label_create(row_header);
    lv_obj_set_style_text_font(label_bat_icon_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_bat_icon_, COLOR_NEON_GREEN, 0);
    lv_label_set_text(label_bat_icon_, "\uf240");
    lv_obj_align(label_bat_icon_, LV_ALIGN_RIGHT_MID, -icon_pad, 0);
    lv_obj_set_style_transform_zoom(label_bat_icon_, zoom_std, 0);

    // 3. Date (Neo CHÍNH GIỮA)
    label_full_date_ = lv_label_create(row_header);
    // [FIX] Quay lại Size Content để không bị xung đột layout
    lv_obj_set_width(label_full_date_, LV_SIZE_CONTENT);
    lv_obj_set_style_text_font(label_full_date_, &font_puhui_14_1, 0);
    lv_obj_set_style_text_color(label_full_date_, COLOR_CYAN, 0);
    lv_label_set_text(label_full_date_, "...");
    
    // [FIX] Lệnh này bắt buộc chữ nằm chính giữa Header
    lv_obj_align(label_full_date_, LV_ALIGN_CENTER, 0, 0); 
    lv_obj_set_style_transform_zoom(label_full_date_, zoom_std, 0);

    // --- HÀNG 2: ĐỒNG HỒ ---
    cont_clock_ = lv_obj_create(main_col);
    lv_obj_set_width(cont_clock_, lv_pct(100)); 
    lv_obj_set_height(cont_clock_, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(cont_clock_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont_clock_, 0, 0);
    lv_obj_set_style_pad_all(cont_clock_, 0, 0);
    lv_obj_set_style_pad_ver(cont_clock_, safe_pad_text + 10, 0); // Chống cụt chân
    
    lv_obj_set_flex_flow(cont_clock_, LV_FLEX_FLOW_ROW); 
    lv_obj_set_flex_align(cont_clock_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(cont_clock_, 0, 0); 

    int digit_w = screen_width_ / 9; 
    int colon_w = digit_w / 2;
    for(int i=0; i<8; i++) {
        lbl_clock_digits_[i] = lv_label_create(cont_clock_);
        lv_obj_set_style_text_font(lbl_clock_digits_[i], &lv_font_ds_digitb_48, 0);
        lv_obj_set_style_text_color(lbl_clock_digits_[i], COLOR_ORANGE, 0);
        lv_obj_set_style_text_align(lbl_clock_digits_[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_transform_zoom(lbl_clock_digits_[i], zoom_std, 0);

        if (i == 2 || i == 5) { 
            lv_obj_set_width(lbl_clock_digits_[i], colon_w);
            lv_label_set_text(lbl_clock_digits_[i], ":");
        } else { 
            lv_obj_set_width(lbl_clock_digits_[i], digit_w);
            lv_label_set_text(lbl_clock_digits_[i], "0");
        }
    }
    
    // --- HÀNG 3: THÀNH PHỐ (ĐÃ FIX CĂN GIỮA) ---
    label_city_ = lv_label_create(main_col);
    // [FIX] Để width là CONTENT để Flex tự căn giữa Object
    lv_obj_set_width(label_city_, LV_SIZE_CONTENT); 
    lv_obj_set_style_text_font(label_city_, &font_puhui_14_1, 0);
    lv_obj_set_style_text_color(label_city_, COLOR_WHITE, 0);
    lv_label_set_text(label_city_, "City");
    lv_obj_set_style_transform_zoom(label_city_, zoom_std, 0);
    lv_obj_set_style_pad_ver(label_city_, safe_pad_text + 5, 0); // Chống cụt chân

    // --- HÀNG 4: THỜI TIẾT (ĐÃ FIX DÍNH & WIDTH) ---
    group_weather_ = lv_obj_create(main_col);
    lv_obj_set_width(group_weather_, LV_SIZE_CONTENT);
    lv_obj_set_height(group_weather_, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(group_weather_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(group_weather_, 0, 0);
    lv_obj_set_style_pad_all(group_weather_, 0, 0);
    lv_obj_set_style_pad_ver(group_weather_, safe_pad_text + 10, 0);
    
    lv_obj_set_flex_flow(group_weather_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(group_weather_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // [FIX 1] Tăng khoảng cách Gap lên 30px (scaled) để Icon không dính Text
    int weather_gap = (int)(30 * w_ratio);
    lv_obj_set_style_pad_gap(group_weather_, weather_gap, 0); 

    // 1. Icon (Bên trái)
    label_main_icon_ = lv_label_create(group_weather_);
    lv_obj_set_style_text_font(label_main_icon_, &font_awesome_30_4, 0);
    lv_obj_set_style_text_color(label_main_icon_, COLOR_NEON_GREEN, 0);
    lv_label_set_text(label_main_icon_, "\uf185");
    lv_obj_set_style_transform_zoom(label_main_icon_, (int)(zoom_std * 1.2), 0); 
    // Đảm bảo không dính bằng padding phải thêm cho chắc chắn
    lv_obj_set_style_pad_right(label_main_icon_, 5, 0);

    // 2. Text Group (Bên phải)
    lv_obj_t* weather_text_col = lv_obj_create(group_weather_);
    lv_obj_set_size(weather_text_col, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(weather_text_col, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(weather_text_col, 0, 0);
    lv_obj_set_style_pad_all(weather_text_col, 0, 0); 
    lv_obj_set_flex_flow(weather_text_col, LV_FLEX_FLOW_COLUMN); 
    // Căn trái nội bộ (để Nhiệt độ và Mô tả thẳng cột với nhau)
    lv_obj_set_flex_align(weather_text_col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    
    label_main_temp_ = lv_label_create(weather_text_col);
    lv_obj_set_style_text_font(label_main_temp_, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_main_temp_, COLOR_NEON_GREEN, 0);
    lv_label_set_text(label_main_temp_, "--°C");
    lv_obj_set_style_transform_zoom(label_main_temp_, zoom_std, 0);

    label_main_desc_ = lv_label_create(weather_text_col);
    // [FIX 2] Giới hạn chiều rộng (~120px) để không đẩy khung ra ngoài mép phải
    lv_obj_set_width(label_main_desc_, (int)(120 * w_ratio)); 
    lv_obj_set_style_pad_ver(label_main_desc_, safe_pad_text + 5, 0); // Chống cụt chân
    // [FIX 3] Bật chế độ chữ chạy vòng tròn để hiển thị hết nội dung dài
    lv_label_set_long_mode(label_main_desc_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    
    lv_obj_set_style_text_font(label_main_desc_, &font_puhui_14_1, 0);
    lv_obj_set_style_text_color(label_main_desc_, COLOR_GRAY, 0);
    lv_label_set_text(label_main_desc_, "---");
    lv_obj_set_style_transform_zoom(label_main_desc_, zoom_std, 0);

    // --- HÀNG 5: CHI TIẾT ---
    int gap_details = (int)(10 * w_ratio);
    group_details_ = lv_obj_create(main_col);
    lv_obj_set_width(group_details_, lv_pct(100)); 
    lv_obj_set_height(group_details_, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(group_details_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(group_details_, 0, 0);
    lv_obj_set_style_pad_all(group_details_, 0, 0); 
    lv_obj_set_flex_flow(group_details_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(group_details_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(group_details_, gap_details, 0);

    CreateDetailArc(group_details_, &arc_humid_, &label_humid_val_, COLOR_MAGENTA);
    CreateDetailArc(group_details_, &arc_press_, &label_press_val_, COLOR_ORANGE);
    CreateDetailArc(group_details_, &arc_wind_, &label_wind_val_, COLOR_CYAN);

    // --- HÀNG 6: DỰ BÁO ---
    int gap_forecast = (int)(2 * w_ratio);
    obj_forecast_cont_ = lv_obj_create(main_col);
    lv_obj_set_width(obj_forecast_cont_, lv_pct(100)); 
    lv_obj_set_height(obj_forecast_cont_, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(obj_forecast_cont_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj_forecast_cont_, 0, 0);
    lv_obj_set_style_pad_all(obj_forecast_cont_, 0, 0);
    lv_obj_set_flex_flow(obj_forecast_cont_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(obj_forecast_cont_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(obj_forecast_cont_, gap_forecast, 0);
    int gap_day_icon = (int)(5 * h_ratio); // Khoảng cách giữa Thứ và Icon

    for(int i=0; i<5; i++) {
        lv_obj_t* day_wrap = lv_obj_create(obj_forecast_cont_);
        lv_obj_set_size(day_wrap, LV_SIZE_CONTENT, LV_SIZE_CONTENT); 
        int min_w = (int)(28 * w_ratio);
        lv_obj_set_style_min_width(day_wrap, min_w, 0); 
        lv_obj_set_style_bg_opa(day_wrap, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(day_wrap, 0, 0);
        lv_obj_set_style_pad_all(day_wrap, 0, 0);
        lv_obj_set_style_pad_gap(day_wrap, 0, 0); 
        lv_obj_set_flex_flow(day_wrap, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_gap(day_wrap, gap_day_icon, 0);
        lv_obj_set_flex_align(day_wrap, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t* lbl_d = lv_label_create(day_wrap);
        lv_obj_set_style_text_font(lbl_d, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl_d, COLOR_GRAY, 0);
        lv_label_set_text(lbl_d, "-");
        lv_obj_set_style_transform_zoom(lbl_d, zoom_std, 0);

        lv_obj_t* lbl_icon = lv_label_create(day_wrap);
        lv_obj_set_style_text_font(lbl_icon, &font_awesome_30_4, 0);
        lv_obj_set_style_text_color(lbl_icon, COLOR_CYAN, 0);
        lv_label_set_text(lbl_icon, "\uf0c2");
        lv_obj_set_style_transform_zoom(lbl_icon, (int)(zoom_std * 0.7), 0);
    }

    // --- HÀNG 7: CHẤM BI DƯỚI ---
    CreateDotsRow(main_col, screen_width, w_ratio);
}

void WeatherUI::ShowIdleCard(const IdleCardInfo& info) {
    if (!container_) return;

    if (label_wifi_icon_) lv_label_set_text(label_wifi_icon_, info.network_icon.c_str());
    if (label_bat_icon_) lv_label_set_text(label_bat_icon_, info.battery_icon.c_str());

    if (label_full_date_) {
        std::string full_str = info.day_text + ", " + info.date_text;
        lv_label_set_text(label_full_date_, full_str.c_str());
    }
    
    // CẬP NHẬT ĐỒNG HỒ SỐ (8 Ô)
    if (cont_clock_ && info.time_text.length() >= 8) {
        for(int i=0; i<8; i++) {
            if (lbl_clock_digits_[i]) {
                char c[2] = {info.time_text[i], '\0'};
                lv_label_set_text(lbl_clock_digits_[i], c);
            }
        }
    }

    if (label_main_temp_) lv_label_set_text(label_main_temp_, info.temperature_text.c_str());
    if (info.icon && label_main_icon_) lv_label_set_text(label_main_icon_, info.icon);
    if (label_main_desc_) lv_label_set_text(label_main_desc_, info.description_text.c_str());

    if (label_city_) lv_label_set_text(label_city_, info.city.c_str());

    if (obj_forecast_cont_ && !info.forecast.empty()) {
        int count = lv_obj_get_child_cnt(obj_forecast_cont_);
        for (int i = 0; i < count && i < info.forecast.size(); i++) {
            lv_obj_t* day_wrap = lv_obj_get_child(obj_forecast_cont_, i);
            if (day_wrap) {
                lv_obj_t* lbl_d = lv_obj_get_child(day_wrap, 0);
                if (lbl_d) lv_label_set_text(lbl_d, info.forecast[i].day_name.c_str());
                
                lv_obj_t* lbl_i = lv_obj_get_child(day_wrap, 1);
                if (lbl_i) lv_label_set_text(lbl_i, GetWeatherIcon(info.forecast[i].icon_code));
            }
        }
    }

    try {
        if (!info.humidity_text.empty() && arc_humid_) {
            int val = std::stoi(info.humidity_text);
            lv_arc_set_value(arc_humid_, val);
            lv_label_set_text(label_humid_val_, info.humidity_text.c_str());
        }
        if (arc_press_) {
            lv_arc_set_value(arc_press_, 65); 
            lv_label_set_text(label_press_val_, "OK");
        }
        if (!info.wind_text.empty() && arc_wind_) {
            std::string w = info.wind_text;
            size_t space = w.find(' ');
            if(space != std::string::npos) w = w.substr(0, space);
            
            lv_arc_set_value(arc_wind_, 40);
            lv_label_set_text(label_wind_val_, w.c_str());
        }
    } catch (...) {}

    lv_obj_remove_flag(container_, LV_OBJ_FLAG_HIDDEN);
}

void WeatherUI::HideIdleCard() {
    if (container_) lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
}
