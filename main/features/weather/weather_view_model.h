#ifndef WEATHER_VIEW_MODEL_H
#define WEATHER_VIEW_MODEL_H

#include "weather_model.h"
#include "weather_service.h"

#include <cstdint>

class Board;
class WeatherService;

struct WeatherSystemSnapshot {
  std::string network_icon;
  int8_t rssi = 0;
  std::string battery_icon;
  int battery_level = -1;
  bool is_charging = false;
};

struct WeatherDataSnapshot {
  WeatherInfo weather;
  WeatherService::FetchStatus fetch_status = WeatherService::FetchStatus::kNever;
  bool is_stale = false;
  uint32_t data_age_ms = UINT32_MAX;
};

/**
 * @brief Builds UI-ready idle weather card data from service + board state.
 */
class WeatherViewModel {
 public:
  /**
   * @brief Build idle card data from pure snapshots.
   *
   * This API is deterministic and suitable for regression fixtures/tests.
   */
  static IdleCardInfo BuildFromSnapshot(const WeatherDataSnapshot& weather,
                                        const WeatherSystemSnapshot& system);

  /**
   * @brief Build idle card data from runtime service + board state.
   */
  static IdleCardInfo BuildIdleCardInfo(const WeatherService& service, Board& board);
};

#endif  // WEATHER_VIEW_MODEL_H
