#ifndef WEATHER_UI_H
#define WEATHER_UI_H

#include "weather_model.h"
#include "weather_detail_screen.h"
#include "lvgl_display.h"
#if HAVE_LVGL
#include <lvgl.h>
#endif
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initialize weather idle singleton instance. */
void weather_idle_init(void);

/** @brief Create/get dedicated weather idle screen. */
lv_obj_t* weather_idle_create_screen(lv_obj_t* host_screen, int screen_width, int screen_height);

/** @brief Push latest idle weather card data to weather idle screen. */
void weather_idle_show_card(const IdleCardInfo* info);

/** @brief Hide weather idle screen and restore host screen. */
void weather_idle_hide(void);

/** @brief Toggle full-screen weather detail view on idle weather screen. */
void weather_idle_toggle_details(void);

/** @brief Return true if weather idle screen is currently active. */
bool weather_idle_is_visible(void);

#ifdef __cplusplus
}
#endif

class WeatherUI {
 public:
  WeatherUI();
  ~WeatherUI();

  /**
   * @brief Build compact idle weather layout on the parent screen.
   */
  void SetupIdleUI(lv_obj_t* parent, int screen_width, int screen_height);

  /**
   * @brief Update and show the idle weather card.
   */
  void ShowIdleCard(const IdleCardInfo& info);

  /**
   * @brief Hide idle weather UI and detail screen.
   */
  void HideIdleCard();

  /**
   * @brief Toggle full-screen weather detail screen.
   */
  void ToggleDetails();

  /**
   * @brief Returns true when idle card or detail screen is visible.
   */
  bool IsVisible() const;

  static const char* GetWeatherIcon(const std::string& code);
  lv_obj_t* GetScreen() const { return idle_screen_; }

  bool IsInitialized() const { return container_ != nullptr; }

 private:
  void UpdateClockAndDate();
  void UpdateStateBadge(WeatherDataState state);
  void AnimateStateTransition();

  lv_obj_t* host_screen_;
  lv_obj_t* idle_screen_;
  lv_obj_t* container_;
  int screen_width_;
  int screen_height_;

  lv_obj_t* label_wifi_icon_;
  lv_obj_t* label_wifi_text_;
  lv_obj_t* label_bat_icon_;
  lv_obj_t* label_bat_text_;

  lv_obj_t* label_time_;
  lv_obj_t* label_date_;
  lv_obj_t* label_location_;
  lv_obj_t* label_state_badge_;
  lv_obj_t* hero_row_;
  lv_obj_t* label_main_temp_;
  lv_obj_t* label_main_icon_;
  lv_obj_t* label_description_;
  lv_obj_t* label_meta_;
  lv_obj_t* label_updated_;
  lv_obj_t* label_hint_;

  WeatherDetailScreen detail_screen_;
  IdleCardInfo last_info_;
  bool has_last_info_;
  WeatherDataState last_state_;
  bool has_last_state_;
};

#endif // WEATHER_UI_H