#include "weather_detail_screen.h"

#include <cstdio>
#include <cstring>

LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(lv_font_ds_digitb_48);

namespace {

void SetLabelTextIfChanged(lv_obj_t* label, const char* text) {
  if (label == nullptr || text == nullptr) {
    return;
  }
  const char* old_text = lv_label_get_text(label);
  if (old_text != nullptr && strcmp(old_text, text) == 0) {
    return;
  }
  lv_label_set_text(label, text);
}

const char* StateLabel(WeatherDataState state) {
  switch (state) {
    case WeatherDataState::kLoading:
      return "Loading weather...";
    case WeatherDataState::kStale:
      return "Stale data";
    case WeatherDataState::kOffline:
      return "Offline";
    case WeatherDataState::kError:
      return "Weather unavailable";
    case WeatherDataState::kSuccess:
    default:
      return "Current weather";
  }
}

}  // namespace

WeatherDetailScreen::WeatherDetailScreen()
    : root_(nullptr),
      title_label_(nullptr),
      city_label_(nullptr),
      temp_icon_label_(nullptr),
      temp_value_label_(nullptr),
      status_label_(nullptr),
      detail_label_(nullptr),
      forecast_label_(nullptr),
      hint_label_(nullptr) {}

void WeatherDetailScreen::Setup(lv_obj_t* parent, int screen_width, int screen_height) {
  if (root_ != nullptr) {
    return;
  }

  root_ = lv_obj_create(parent);
  lv_obj_set_size(root_, screen_width, screen_height);
  lv_obj_set_style_bg_color(root_, lv_color_hex(0x000000), 0);
  lv_obj_set_style_border_width(root_, 0, 0);
  lv_obj_set_style_pad_all(root_, 10, 0);
  lv_obj_set_style_pad_row(root_, 8, 0);
  lv_obj_set_flex_flow(root_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(root_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);

  title_label_ = lv_label_create(root_);
  lv_obj_set_style_text_font(title_label_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(title_label_, lv_color_hex(0xDDDDDD), 0);
  lv_label_set_text(title_label_, "Weather details");

  city_label_ = lv_label_create(root_);
  lv_obj_set_style_text_font(city_label_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(city_label_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(city_label_, "Location: --");

  lv_obj_t* hero_row = lv_obj_create(root_);
  lv_obj_set_size(hero_row, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(hero_row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(hero_row, 0, 0);
  lv_obj_set_style_pad_all(hero_row, 0, 0);
  lv_obj_set_flex_flow(hero_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(hero_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  temp_value_label_ = lv_label_create(hero_row);
  lv_obj_set_style_text_font(temp_value_label_, &lv_font_ds_digitb_48, 0);
  lv_obj_set_style_text_color(temp_value_label_, lv_color_hex(0xFFA500), 0);
  lv_label_set_text(temp_value_label_, "--°C");

  temp_icon_label_ = lv_label_create(hero_row);
  lv_obj_set_style_text_font(temp_icon_label_, &font_awesome_30_4, 0);
  lv_obj_set_style_text_color(temp_icon_label_, lv_color_hex(0x39FF14), 0);
  lv_label_set_text(temp_icon_label_, "?");

  status_label_ = lv_label_create(root_);
  lv_obj_set_style_text_font(status_label_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(status_label_, lv_color_hex(0xFFB347), 0);
  lv_label_set_text(status_label_, "Status: --");

  detail_label_ = lv_label_create(root_);
  lv_obj_set_width(detail_label_, LV_PCT(100));
  lv_obj_set_style_text_font(detail_label_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(detail_label_, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text(detail_label_, "Humidity: --\nWind: --\nFeels like: --\nPressure: --");

  forecast_label_ = lv_label_create(root_);
  lv_obj_set_width(forecast_label_, LV_PCT(100));
  lv_obj_set_style_text_font(forecast_label_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(forecast_label_, lv_color_hex(0xBBBBBB), 0);
  lv_label_set_text(forecast_label_, "Forecast: --");

  hint_label_ = lv_label_create(root_);
  lv_obj_set_style_text_font(hint_label_, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(hint_label_, lv_color_hex(0x888888), 0);
  lv_label_set_text(hint_label_, "Hold BOOT: back to idle weather");
}

void WeatherDetailScreen::UpdateContent(const IdleCardInfo& info) {
  if (root_ == nullptr) {
    return;
  }

  char city_buf[96];
  snprintf(city_buf, sizeof(city_buf), "Location: %s", info.city.empty() ? "--" : info.city.c_str());
  SetLabelTextIfChanged(city_label_, city_buf);

  SetLabelTextIfChanged(temp_value_label_,
                        info.temperature_text.empty() ? "--°C" : info.temperature_text.c_str());
  SetLabelTextIfChanged(temp_icon_label_, info.icon != nullptr ? info.icon : "?");

  char status_buf[96];
  snprintf(status_buf, sizeof(status_buf), "Status: %s", StateLabel(info.weather_state));
  SetLabelTextIfChanged(status_label_, status_buf);

  char detail_buf[192];
  snprintf(detail_buf, sizeof(detail_buf),
           "Humidity: %s\nWind: %s\nFeels like: %s\nPressure: %s",
           info.humidity_text.empty() ? "--" : info.humidity_text.c_str(),
           info.wind_text.empty() ? "--" : info.wind_text.c_str(),
           info.feels_like_text.empty() ? "--" : info.feels_like_text.c_str(),
           info.pressure_text.empty() ? "--" : info.pressure_text.c_str());
  SetLabelTextIfChanged(detail_label_, detail_buf);

  char forecast_buf[256];
  if (info.forecast.empty()) {
    snprintf(forecast_buf, sizeof(forecast_buf), "Forecast: --");
  } else {
    size_t offset = 0;
    offset += snprintf(forecast_buf + offset, sizeof(forecast_buf) - offset, "Forecast: ");
    for (size_t i = 0; i < info.forecast.size() && i < 3 && offset < sizeof(forecast_buf); ++i) {
      offset += snprintf(forecast_buf + offset, sizeof(forecast_buf) - offset, "%s %.0fC%s",
                         info.forecast[i].day_name.c_str(), info.forecast[i].temp,
                         (i == 2 || i + 1 >= info.forecast.size()) ? "" : " | ");
    }
  }
  SetLabelTextIfChanged(forecast_label_, forecast_buf);
}

void WeatherDetailScreen::Show(const IdleCardInfo& info) {
  if (root_ == nullptr) {
    return;
  }
  UpdateContent(info);
  lv_obj_clear_flag(root_, LV_OBJ_FLAG_HIDDEN);
}

void WeatherDetailScreen::Hide() {
  if (root_ == nullptr) {
    return;
  }
  lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);
}

void WeatherDetailScreen::Toggle(const IdleCardInfo& info) {
  if (IsVisible()) {
    Hide();
    return;
  }
  Show(info);
}

bool WeatherDetailScreen::IsVisible() const {
  return root_ != nullptr && !lv_obj_has_flag(root_, LV_OBJ_FLAG_HIDDEN);
}
