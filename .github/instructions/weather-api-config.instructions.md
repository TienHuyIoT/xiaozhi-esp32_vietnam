---
description: "Weather API configuration source-of-truth. Use when implementing or editing weather networking/config logic to enforce endpoint, API key, and location constants from weather_config.h."
applyTo: "main/features/weather/**,main/**/*.h,main/**/*.cc,docs/**"
---

# Weather API Config Instructions

Apply these rules whenever creating or editing weather fetch/config code.

## Source Of Truth

Use `main/features/weather/weather_config.h` as the canonical config source for weather networking defaults.

Reference values currently used by this project:

- Weather API base endpoint: `https://api.openweathermap.org/data/2.5/`
- IP geolocation endpoint: `https://ipwho.is`
- Default API key value is defined in config header for fallback usage.

Macro names are flexible. You may create/rename macros to match module design, as long as endpoint/key defaults remain centralized in `weather_config.h`.

## Implementation Rules

- Do not hardcode weather endpoint URLs in `.cc` files.
- Do not hardcode IP geolocation endpoint URLs in `.cc` files.
- Do not inline OpenWeatherMap API key strings in implementation files.
- Resolve API key through a centralized default macro/constant in `weather_config.h` unless runtime/user setting overrides it.
- Keep all default weather endpoint and key definitions centralized in `weather_config.h`.

## Override Strategy

- If project needs custom endpoint/key per environment, override via build config or compile definition.
- Keep fallback defaults in `weather_config.h` to preserve boot-time behavior.
- Document any environment override path in docs when added.

## Reliability And Security Notes

- Treat the default API key macro/constant as demo/default only; production deployments should use project-specific key provisioning.
- Avoid logging full API keys in plaintext logs.
- Keep timeout behavior consistent with `WEATHER_HTTP_TIMEOUT_MS` unless there is a documented reason.

## Validation Checklist

1. Weather fetch code references a centralized weather endpoint macro/constant from `weather_config.h`.
2. IP geolocation code references a centralized IP endpoint macro/constant from `weather_config.h`.
3. API key resolution path includes centralized default key fallback from `weather_config.h`.
4. No duplicated endpoint/key literals are introduced.
5. Any override mechanism is documented and backward compatible.
