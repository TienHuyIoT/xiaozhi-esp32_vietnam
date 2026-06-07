#ifndef WEATHER_CONFIG_H
#define WEATHER_CONFIG_H

// Weather update interval (in milliseconds)
#ifndef WEATHER_UPDATE_INTERVAL_MS
#define WEATHER_UPDATE_INTERVAL_MS (30 * 60 * 1000)  // 30 minutes
#endif

// Weather data is considered stale when it is older than this threshold.
#ifndef WEATHER_STALE_THRESHOLD_MS
#define WEATHER_STALE_THRESHOLD_MS (15 * 60 * 1000)  // 15 minutes
#endif

// Default OpenWeatherMap API key, you can replace it with your own key
// Get a free API key from https://openweathermap.org/api
#ifndef OPEN_WEATHERMAP_API_KEY_DEFAULT
// This is a demo API key with limited usage and may be revoked at any time.
#define OPEN_WEATHERMAP_API_KEY_DEFAULT "ae8d3c2fda691593ce3e84472ef25784"
#endif

// Weather API endpoints
#define WEATHER_API_ENDPOINT "https://api.openweathermap.org/data/2.5/"
#define IP_LOCATION_API_ENDPOINT "https://ipwho.is"
#define CITY_LOCATION_DEFAULT "Hanoi"

// HTTP timeout settings
#define WEATHER_HTTP_TIMEOUT_MS 10000

#endif // WEATHER_CONFIG_H
