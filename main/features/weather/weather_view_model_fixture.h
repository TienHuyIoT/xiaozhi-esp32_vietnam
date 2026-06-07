#ifndef WEATHER_VIEW_MODEL_FIXTURE_H
#define WEATHER_VIEW_MODEL_FIXTURE_H

#include <string>

/**
 * @brief Runs deterministic WeatherViewModel regression fixtures.
 *
 * Returns true when all fixture cases pass. On failure, failure_reason is filled
 * with a compact description of the first failed assertion.
 */
bool RunWeatherViewModelFixture(std::string* failure_reason);

#endif  // WEATHER_VIEW_MODEL_FIXTURE_H
