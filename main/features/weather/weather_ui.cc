#include "weather_ui.h"

#include <cstdio>
#include <cstring>
#include <ctime>

LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(font_awesome_20_4);
LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(lv_font_ds_digitb_48);

static WeatherUI* g_weather_ui = nullptr;

WeatherUI::WeatherUI() {
  host_screen_ = nullptr;
  idle_screen_ = nullptr;
  container_ = nullptr;
  screen_width_ = 0;
  screen_height_ = 0;
  label_wifi_icon_ = nullptr;
  label_wifi_text_ = nullptr;
  label_bat_icon_ = nullptr;
  label_bat_text_ = nullptr;
  label_time_ = nullptr;
  label_date_ = nullptr;
  label_location_ = nullptr;
  label_state_badge_ = nullptr;
  hero_row_ = nullptr;
  label_main_temp_ = nullptr;
  label_main_icon_ = nullptr;
  label_description_ = nullptr;
  label_meta_ = nullptr;
  label_updated_ = nullptr;
  label_hint_ = nullptr;
  has_last_info_ = false;
  last_state_ = WeatherDataState::kLoading;
  has_last_state_ = false;
}

WeatherUI::~WeatherUI() {
  if (idle_screen_ != nullptr) {
    lv_obj_del(idle_screen_);
    idle_screen_ = nullptr;
  }
}

const char* WeatherUI::GetWeatherIcon(const std::string& code) {
  if (code == "01d" || code == "01n") return "\uf185";
  if (code == "02d" || code == "02n") return "\uf6c4";
  if (code == "03d" || code == "03n") return "\uf0c2";
  if (code == "04d" || code == "04n") return "\uf0c2";
  if (code == "09d" || code == "09n") return "\uf740";
  if (code == "10d" || code == "10n") return "\uf743";
  if (code == "11d" || code == "11n") return "\uf0e7";
  if (code == "13d" || code == "13n") return "\uf2dc";
  if (code == "50d" || code == "50n") return "\uf75f";
  return "\uf0c2";
}

void WeatherUI::SetupIdleUI(lv_obj_t* parent, int screen_width, int screen_height) {
  if (idle_screen_ != nullptr) {
    return;
  }

  host_screen_ = lv_obj_get_screen(parent);
  if (host_screen_ == nullptr) {
    host_screen_ = parent;
  }

  screen_width_ = screen_width;
  screen_height_ = screen_height;

  idle_screen_ = lv_obj_create(nullptr);
  lv_obj_set_size(idle_screen_, screen_width_, screen_height_);
  lv_obj_set_style_bg_color(idle_screen_, lv_color_hex(0x000000), 0);
  lv_obj_set_style_border_width(idle_screen_, 0, 0);
  lv_obj_set_style_radius(idle_screen_, 0, 0);
  lv_obj_clear_flag(idle_screen_, LV_OBJ_FLAG_SCROLLABLE);

  container_ = lv_obj_create(idle_screen_);
  lv_obj_set_size(container_, screen_width_, screen_height_);
  lv_obj_set_style_bg_color(container_, lv_color_hex(0x000000), 0);
  lv_obj_set_style_border_width(container_, 0, 0);
  lv_obj_set_style_radius(container_, 0, 0);
  lv_obj_set_style_pad_all(container_, 8, 0);
  lv_obj_set_style_pad_row(container_, 8, 0);
  lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(container_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(container_, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* top_row = lv_obj_create(container_);
  lv_obj_set_size(top_row, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(top_row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(top_row, 0, 0);
  lv_obj_set_style_pad_all(top_row, 0, 0);
  lv_obj_set_flex_flow(top_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(top_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t* bat_group = lv_obj_create(top_row);
  lv_obj_set_size(bat_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(bat_group, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(bat_group, 0, 0);
  lv_obj_set_style_pad_all(bat_group, 0, 0);
  lv_obj_set_style_pad_column(bat_group, 4, 0);
  lv_obj_set_flex_flow(bat_group, LV_FLEX_FLOW_ROW);

  label_bat_icon_ = lv_label_create(bat_group);
  lv_obj_set_style_text_font(label_bat_icon_, &font_awesome_20_4, 0);
  lv_obj_set_style_text_color(label_bat_icon_, lv_color_hex(0x39FF14), 0);
  lv_label_set_text(label_bat_icon_, "\xef\x89\x80");

  label_bat_text_ = lv_label_create(bat_group);
  lv_obj_set_style_text_font(label_bat_text_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_bat_text_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(label_bat_text_, "--%");

  lv_obj_t* wifi_group = lv_obj_create(top_row);
  lv_obj_set_size(wifi_group, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(wifi_group, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(wifi_group, 0, 0);
  lv_obj_set_style_pad_all(wifi_group, 0, 0);
  lv_obj_set_style_pad_column(wifi_group, 4, 0);
  lv_obj_set_flex_flow(wifi_group, LV_FLEX_FLOW_ROW);

  label_wifi_text_ = lv_label_create(wifi_group);
  lv_obj_set_style_text_font(label_wifi_text_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_wifi_text_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(label_wifi_text_, "--");

  label_wifi_icon_ = lv_label_create(wifi_group);
  lv_obj_set_style_text_font(label_wifi_icon_, &font_awesome_20_4, 0);
  lv_obj_set_style_text_color(label_wifi_icon_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(label_wifi_icon_, "\xef\x87\xab");

  label_time_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_time_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_time_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(label_time_, "--:--");

  label_date_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_date_, &font_puhui_20_4, 0);
  lv_obj_set_style_text_color(label_date_, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(label_date_, "---- -- --");

  hero_row_ = lv_obj_create(container_);
  lv_obj_set_size(hero_row_, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(hero_row_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(hero_row_, 0, 0);
  lv_obj_set_style_pad_all(hero_row_, 0, 0);
  lv_obj_set_style_pad_column(hero_row_, 16, 0);
  lv_obj_set_flex_flow(hero_row_, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(hero_row_, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  label_main_temp_ = lv_label_create(hero_row_);
  lv_obj_set_style_text_font(label_main_temp_, &lv_font_ds_digitb_48, 0);
  lv_obj_set_style_text_color(label_main_temp_, lv_color_hex(0xFFA500), 0);
  lv_label_set_text(label_main_temp_, "--°C");

  label_main_icon_ = lv_label_create(hero_row_);
  lv_obj_set_style_text_font(label_main_icon_, &font_awesome_30_4, 0);
  lv_obj_set_style_text_color(label_main_icon_, lv_color_hex(0x39FF14), 0);
  lv_label_set_text(label_main_icon_, "\uf185");

  label_description_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_description_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_description_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(label_description_, "--");

  label_location_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_location_, &font_puhui_20_4, 0);
  lv_obj_set_style_text_color(label_location_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(label_location_, "\xef\x81\x81 Dang cap nhat...");

  label_meta_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_meta_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_meta_, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(label_meta_, "Wind: --  Humidity: --");

  label_updated_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_updated_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_updated_, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(label_updated_, "Updated: ---- -- --:--");

  label_state_badge_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_state_badge_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_state_badge_, lv_color_hex(0xFFB347), 0);
  lv_label_set_text(label_state_badge_, "");
  lv_obj_add_flag(label_state_badge_, LV_OBJ_FLAG_HIDDEN);

  label_hint_ = lv_label_create(container_);
  lv_obj_set_style_text_font(label_hint_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(label_hint_, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(label_hint_, "Hold BOOT: weather details");

  detail_screen_.Setup(idle_screen_, screen_width_, screen_height_);
}

void WeatherUI::UpdateClockAndDate() {
  if (label_time_ == nullptr || label_date_ == nullptr) {
    return;
  }

  time_t now;
  struct tm local_tm;
  time(&now);
  localtime_r(&now, &local_tm);

  char time_buf[16];
  snprintf(time_buf, sizeof(time_buf), "%02d:%02d", local_tm.tm_hour, local_tm.tm_min);
  lv_label_set_text(label_time_, time_buf);

  char date_buf[32];
  snprintf(date_buf, sizeof(date_buf), "%04d-%02d-%02d", local_tm.tm_year + 1900,
           local_tm.tm_mon + 1, local_tm.tm_mday);
  lv_label_set_text(label_date_, date_buf);
}

void WeatherUI::UpdateStateBadge(WeatherDataState state) {
  const char* badge = "";
  switch (state) {
    case WeatherDataState::kLoading:
      badge = "Loading weather...";
      break;
    case WeatherDataState::kStale:
      badge = "Stale 15m+";
      break;
    case WeatherDataState::kOffline:
      badge = "Offline";
      break;
    case WeatherDataState::kError:
      badge = "Weather unavailable";
      break;
    case WeatherDataState::kSuccess:
    default:
      break;
  }

  if (badge[0] == '\0') {
    if (label_state_badge_ != nullptr) {
      lv_obj_add_flag(label_state_badge_, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }

  lv_label_set_text(label_state_badge_, badge);
  if (label_state_badge_ != nullptr) {
    if (state == WeatherDataState::kError) {
      lv_obj_set_style_text_color(label_state_badge_, lv_color_hex(0xFF6B6B), 0);
    } else {
      lv_obj_set_style_text_color(label_state_badge_, lv_color_hex(0xFFB347), 0);
    }
    lv_obj_clear_flag(label_state_badge_, LV_OBJ_FLAG_HIDDEN);
  }
}

void WeatherUI::AnimateStateTransition() {
  if (hero_row_ == nullptr) {
    return;
  }
  lv_anim_t anim;
  lv_anim_init(&anim);
  lv_anim_set_var(&anim, hero_row_);
  lv_anim_set_values(&anim, LV_OPA_50, LV_OPA_100);
  lv_anim_set_duration(&anim, 180);
  lv_anim_set_exec_cb(&anim, [](void* obj, int32_t value) {
    lv_obj_set_style_opa(static_cast<lv_obj_t*>(obj), value, 0);
  });
  lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
  lv_anim_start(&anim);
}

void WeatherUI::ShowIdleCard(const IdleCardInfo& info) {
  if (container_ == nullptr) {
    return;
  }

  last_info_ = info;
  has_last_info_ = true;

  char battery_buf[16];
  snprintf(battery_buf, sizeof(battery_buf), "%d%%", info.battery_level);
  lv_label_set_text(label_bat_text_, battery_buf);
  lv_label_set_text(label_bat_icon_, info.battery_icon.c_str());

  char rssi_buf[16];
  snprintf(rssi_buf, sizeof(rssi_buf), "%d", info.rssi);
  lv_label_set_text(label_wifi_text_, rssi_buf);
  lv_label_set_text(label_wifi_icon_, info.network_icon.c_str());

  UpdateClockAndDate();

  if (!info.city.empty()) {
    char city_buf[96];
    snprintf(city_buf, sizeof(city_buf), "\xef\x81\x81 %s", info.city.c_str());
    lv_label_set_text(label_location_, city_buf);
  }

  lv_label_set_text(label_main_temp_,
                    info.temperature_text.empty() ? "--°C" : info.temperature_text.c_str());
  lv_label_set_text(label_main_icon_, info.icon == nullptr ? "?" : info.icon);
  lv_label_set_text(label_description_,
                    info.description_text.empty() ? "--" : info.description_text.c_str());

  char meta_buf[160];
  snprintf(meta_buf, sizeof(meta_buf), "Wind: %s  Humidity: %s",
           info.wind_text.empty() ? "--" : info.wind_text.c_str(),
           info.humidity_text.empty() ? "--" : info.humidity_text.c_str());
  lv_label_set_text(label_meta_, meta_buf);

  char updated_buf[96];
  snprintf(updated_buf, sizeof(updated_buf), "Updated: %s",
           info.updated_text.empty() ? "---- -- --:--" : info.updated_text.c_str());
  lv_label_set_text(label_updated_, updated_buf);

  UpdateStateBadge(info.weather_state);

  if (!has_last_state_ || last_state_ != info.weather_state) {
    AnimateStateTransition();
    last_state_ = info.weather_state;
    has_last_state_ = true;
  }

  if (lv_screen_active() != idle_screen_) {
    lv_scr_load(idle_screen_);
  }

  if (detail_screen_.IsVisible()) {
    detail_screen_.Show(info);
  }
}

void WeatherUI::HideIdleCard() {
  detail_screen_.Hide();
  if (host_screen_ != nullptr && lv_screen_active() == idle_screen_) {
    lv_scr_load(host_screen_);
  }
}

void WeatherUI::ToggleDetails() {
  if (!has_last_info_ || container_ == nullptr || idle_screen_ == nullptr) {
    return;
  }

  if (lv_screen_active() != idle_screen_) {
    return;
  }

  if (detail_screen_.IsVisible()) {
    detail_screen_.Hide();
    return;
  }

  detail_screen_.Show(last_info_);
}

bool WeatherUI::IsVisible() const {
  return idle_screen_ != nullptr && lv_screen_active() == idle_screen_;
}

extern "C" void weather_idle_init(void) {
  if (g_weather_ui == nullptr) {
    g_weather_ui = new WeatherUI();
  }
}

extern "C" lv_obj_t* weather_idle_create_screen(lv_obj_t* host_screen, int screen_width,
                                                  int screen_height) {
  if (g_weather_ui == nullptr) {
    g_weather_ui = new WeatherUI();
  }
  WeatherUI& ui = *g_weather_ui;
  ui.SetupIdleUI(host_screen, screen_width, screen_height);
  return ui.GetScreen();
}

extern "C" void weather_idle_show_card(const IdleCardInfo* info) {
  if (info == nullptr) {
    return;
  }
  if (g_weather_ui == nullptr) {
    g_weather_ui = new WeatherUI();
  }
  g_weather_ui->ShowIdleCard(*info);
}

extern "C" void weather_idle_hide(void) {
  if (g_weather_ui == nullptr) {
    g_weather_ui = new WeatherUI();
  }
  g_weather_ui->HideIdleCard();
}

extern "C" void weather_idle_toggle_details(void) {
  if (g_weather_ui == nullptr) {
    g_weather_ui = new WeatherUI();
  }
  g_weather_ui->ToggleDetails();
}

extern "C" bool weather_idle_is_visible(void) {
  return g_weather_ui != nullptr && g_weather_ui->IsVisible();
}
