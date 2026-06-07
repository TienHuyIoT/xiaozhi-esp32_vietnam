#include "weather_view_model_fixture.h"

#include "weather_view_model.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

struct FixtureCase {
  const char* name;
  WeatherDataSnapshot weather;
  WeatherSystemSnapshot system;
  WeatherDataState expected_state;
  const char* expected_temp;
  const char* expected_updated;
};

FixtureCase MakeSuccessCase() {
  FixtureCase tc{};
  tc.name = "success_case";
  tc.weather.weather.valid = true;
  tc.weather.weather.city = "Hanoi";
  tc.weather.weather.temp = 30.6f;
  tc.weather.weather.humidity = 75;
  tc.weather.weather.feels_like = 34.1f;
  tc.weather.weather.pressure = 1008;
  tc.weather.weather.wind_speed = 4.2f;
  tc.weather.weather.icon_code = "01d";
  tc.weather.fetch_status = WeatherService::FetchStatus::kSuccess;
  tc.weather.is_stale = false;
  tc.weather.data_age_ms = 3 * 60000;

  tc.system.network_icon = "wifi";
  tc.system.rssi = -58;
  tc.system.battery_icon = "bat";
  tc.system.battery_level = 83;
  tc.system.is_charging = false;

  tc.expected_state = WeatherDataState::kSuccess;
  tc.expected_temp = "31°C";
  tc.expected_updated = "-3m";
  return tc;
}

FixtureCase MakeStaleCase() {
  FixtureCase tc{};
  tc.name = "stale_case";
  tc.weather.weather.valid = true;
  tc.weather.weather.city = "Hanoi";
  tc.weather.weather.temp = 28.1f;
  tc.weather.weather.humidity = 69;
  tc.weather.weather.feels_like = 31.0f;
  tc.weather.weather.pressure = 1005;
  tc.weather.weather.wind_speed = 2.3f;
  tc.weather.weather.icon_code = "10d";
  tc.weather.fetch_status = WeatherService::FetchStatus::kSuccess;
  tc.weather.is_stale = true;
  tc.weather.data_age_ms = 21 * 60000;

  tc.system.network_icon = "wifi";
  tc.system.rssi = -65;
  tc.system.battery_icon = "bat";
  tc.system.battery_level = 45;
  tc.system.is_charging = false;

  tc.expected_state = WeatherDataState::kStale;
  tc.expected_temp = "28°C";
  tc.expected_updated = "-21m";
  return tc;
}

FixtureCase MakeOfflineCase() {
  FixtureCase tc{};
  tc.name = "offline_case";
  tc.weather.weather.valid = false;
  tc.weather.fetch_status = WeatherService::FetchStatus::kOffline;
  tc.weather.is_stale = false;
  tc.weather.data_age_ms = UINT32_MAX;

  tc.system.network_icon = "wifi_off";
  tc.system.rssi = -120;
  tc.system.battery_icon = "bat";
  tc.system.battery_level = 31;
  tc.system.is_charging = false;

  tc.expected_state = WeatherDataState::kOffline;
  tc.expected_temp = "--°C";
  tc.expected_updated = "---- -- --:--";
  return tc;
}

bool ExpectEq(const std::string& lhs, const char* rhs) {
  return lhs == rhs;
}

}  // namespace

bool RunWeatherViewModelFixture(std::string* failure_reason) {
  const std::vector<FixtureCase> cases = {
      MakeSuccessCase(),
      MakeStaleCase(),
      MakeOfflineCase(),
  };

  for (const FixtureCase& tc : cases) {
    const IdleCardInfo card = WeatherViewModel::BuildFromSnapshot(tc.weather, tc.system);

    if (card.weather_state != tc.expected_state) {
      if (failure_reason != nullptr) {
        *failure_reason = std::string(tc.name) + ": unexpected weather_state";
      }
      return false;
    }
    if (!ExpectEq(card.temperature_text, tc.expected_temp)) {
      if (failure_reason != nullptr) {
        *failure_reason = std::string(tc.name) + ": unexpected temperature_text";
      }
      return false;
    }
    if (!ExpectEq(card.updated_text, tc.expected_updated)) {
      if (failure_reason != nullptr) {
        *failure_reason = std::string(tc.name) + ": unexpected updated_text";
      }
      return false;
    }
  }

  return true;
}
