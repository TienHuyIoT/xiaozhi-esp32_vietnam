#include "weather_view_model.h"

#include "board.h"
#include "font_awesome.h"
#include "ml307_board.h"
#include "weather_service.h"
#include "weather_ui.h"
#include "wifi_station.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace {

std::string FormatUpdatedAgeText(uint32_t age_ms) {
  if (age_ms == UINT32_MAX) {
    return "---- -- --:--";
  }
  const uint32_t age_min = age_ms / 60000;
  char buf[24];
  snprintf(buf, sizeof(buf), "-%lum", static_cast<unsigned long>(age_min));
  return std::string(buf);
}

std::string FormatWindKmh(float speed_mps) {
  const float speed_kmh = speed_mps * 3.6f;
  char buf[24];
  if (speed_kmh < 10.0f) {
    snprintf(buf, sizeof(buf), "%.1f km/h", speed_kmh);
  } else {
    snprintf(buf, sizeof(buf), "%d km/h", static_cast<int>(std::round(speed_kmh)));
  }
  return std::string(buf);
}

WeatherSystemSnapshot CaptureSystemSnapshot(Board& board) {
  WeatherSystemSnapshot snapshot;
  snapshot.network_icon = board.GetNetworkStateIcon();

  if (board.GetBoardType() == "wifi") {
    snapshot.rssi = WifiStation::GetInstance().GetRssi();
  } else {
    AtModem* cellular_modem = static_cast<AtModem*>(board.GetNetwork());
    snapshot.rssi = cellular_modem ? cellular_modem->GetCsq() : 0;
  }

  int battery_level = -1;
  bool charging = false;
  bool discharging = false;
  if (board.GetBatteryLevel(battery_level, charging, discharging)) {
    const char* icon = FONT_AWESOME_BATTERY_BOLT;
    if (!charging) {
      const char* const levels[] = {
          FONT_AWESOME_BATTERY_EMPTY,
          FONT_AWESOME_BATTERY_QUARTER,
          FONT_AWESOME_BATTERY_HALF,
          FONT_AWESOME_BATTERY_THREE_QUARTERS,
          FONT_AWESOME_BATTERY_FULL,
          FONT_AWESOME_BATTERY_FULL,
      };
      const int index = battery_level < 0 ? 0 : (battery_level > 100 ? 5 : battery_level / 20);
      icon = levels[index];
    }
    snapshot.battery_icon = icon;
    snapshot.battery_level = battery_level;
    snapshot.is_charging = charging;
  } else {
    snapshot.battery_icon = FONT_AWESOME_BATTERY_BOLT;
    snapshot.battery_level = -1;
    snapshot.is_charging = false;
  }

  return snapshot;
}

WeatherDataSnapshot CaptureWeatherSnapshot(const WeatherService& service) {
  WeatherDataSnapshot snapshot;
  snapshot.weather = service.GetWeatherInfo();
  snapshot.fetch_status = service.GetLastFetchStatus();
  snapshot.is_stale = service.IsStale();
  snapshot.data_age_ms = service.GetDataAgeMs();
  return snapshot;
}

}  // namespace

IdleCardInfo WeatherViewModel::BuildFromSnapshot(const WeatherDataSnapshot& weather,
                                                 const WeatherSystemSnapshot& system) {
  IdleCardInfo card;
  card.network_icon = system.network_icon;
  card.rssi = system.rssi;
  card.battery_icon = system.battery_icon;
  card.battery_level = system.battery_level;
  card.is_charging = system.is_charging;

  if (weather.weather.valid) {
    const WeatherInfo& src = weather.weather;
    card.city = src.city;

    char temp_buf[16];
    snprintf(temp_buf, sizeof(temp_buf), "%d°C", static_cast<int>(std::round(src.temp)));
    card.temperature_text = temp_buf;

    card.description_text = src.description;

    char humidity_buf[16];
    snprintf(humidity_buf, sizeof(humidity_buf), "%d%%", src.humidity);
    card.humidity_text = humidity_buf;

    card.wind_text = FormatWindKmh(src.wind_speed);

    char feels_like_buf[24];
    snprintf(feels_like_buf, sizeof(feels_like_buf), "%.1f°C", src.feels_like);
    card.feels_like_text = feels_like_buf;

    char pressure_buf[24];
    snprintf(pressure_buf, sizeof(pressure_buf), "%d hPa", src.pressure);
    card.pressure_text = pressure_buf;

    card.forecast = src.forecast;
    card.icon = WeatherUI::GetWeatherIcon(src.icon_code);
    card.weather_state = weather.is_stale ? WeatherDataState::kStale : WeatherDataState::kSuccess;
    if (weather.is_stale) {
      card.stale_badge_text = "Stale data";
    }
    card.updated_text = FormatUpdatedAgeText(weather.data_age_ms);
    return card;
  }

  card.city = "Dang cap nhat...";
  card.temperature_text = "--°C";
  card.icon = "?";

  if (weather.fetch_status == WeatherService::FetchStatus::kOffline) {
    card.weather_state = WeatherDataState::kOffline;
  } else if (weather.fetch_status == WeatherService::FetchStatus::kError) {
    card.weather_state = WeatherDataState::kError;
  } else {
    card.weather_state = WeatherDataState::kLoading;
  }
  card.updated_text = "---- -- --:--";
  return card;
}

IdleCardInfo WeatherViewModel::BuildIdleCardInfo(const WeatherService& service, Board& board) {
  const WeatherDataSnapshot weather_snapshot = CaptureWeatherSnapshot(service);
  const WeatherSystemSnapshot system_snapshot = CaptureSystemSnapshot(board);
  return BuildFromSnapshot(weather_snapshot, system_snapshot);
}
