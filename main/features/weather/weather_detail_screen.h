#ifndef WEATHER_DETAIL_SCREEN_H
#define WEATHER_DETAIL_SCREEN_H

#include "weather_model.h"
#include <lvgl.h>

/**
 * @brief Full-screen weather detail panel for idle mode.
 *
 * This screen is separated from the compact idle weather card to reduce
 * layout complexity and avoid heavy redraws in the compact card path.
 */
class WeatherDetailScreen {
 public:
  WeatherDetailScreen();
  ~WeatherDetailScreen() = default;

  void Setup(lv_obj_t* parent, int screen_width, int screen_height);
  void Show(const IdleCardInfo& info);
  void Hide();
  void Toggle(const IdleCardInfo& info);
  bool IsVisible() const;

 private:
  void UpdateContent(const IdleCardInfo& info);

  lv_obj_t* root_;
  lv_obj_t* title_label_;
  lv_obj_t* city_label_;
  lv_obj_t* temp_icon_label_;
  lv_obj_t* temp_value_label_;
  lv_obj_t* status_label_;
  lv_obj_t* detail_label_;
  lv_obj_t* forecast_label_;
  lv_obj_t* hint_label_;
};

#endif  // WEATHER_DETAIL_SCREEN_H
