---
name: weather-api-config-implementation
description: "Use when implementing or refactoring weather networking/config code to enforce centralized weather endpoint, IP endpoint, and default API key values from weather_config.h (macro names may vary)."
---

# Weather API Config Implementation Skill

Use this skill when writing weather API integration code so configuration stays centralized and consistent.

## Source File

- `main/features/weather/weather_config.h`

## Required Config Values

- Weather API base endpoint: `https://api.openweathermap.org/data/2.5/`
- IP geolocation endpoint: `https://ipwho.is`
- Default weather API key fallback value from config header
- Optional defaults: city and HTTP timeout

Note: macro/constant names can be redesigned to fit architecture. Centralization in `weather_config.h` is mandatory.

## Rules

1. Never hardcode weather endpoint or IP geolocation endpoint inside `.cc` weather implementation files.
2. Never inline API key string literals in implementation files.
3. Resolve API key through a centralized default macro/constant fallback unless runtime/user config overrides it.
4. Keep default endpoints and timeout values centralized in `weather_config.h`.
5. Avoid logging full API key values.

## Implementation Checklist

1. Include weather config header in implementation file.
2. Replace endpoint literals with centralized endpoint macros/constants from `weather_config.h`.
3. Replace inline key usage with centralized default-key fallback logic.
4. Ensure request timeout uses centralized config from `weather_config.h`.
5. Validate no duplicated endpoint/key literals remain.

## Output Expectation

When applied, return:

1. Files updated.
2. Literals replaced by symbols.
3. Any unresolved override/config risks.
