---
description: "Weather UI consistency conventions for embedded screens. Use when designing or editing weather widgets/screens to keep temperature, wind unit, timestamp, and icon mapping consistent."
applyTo: "main/features/weather/**,main/display/**,main/boards/**,main/**/*.h,main/**/*.cc,docs/**"
---

# Weather UI Consistency Instructions

Apply these rules whenever creating or editing weather UI.

## Temperature Format

- Default unit: `°C`.
- Show one decimal only when needed by source precision or user setting.
- Preferred display:
  - Integer mode: `31°C`
  - Decimal mode: `31.5°C`
- Keep sign for below-zero values (for example `-2°C`).
- Do not show trailing `.0` in integer mode.

## Wind Unit Format

- Default wind speed unit: `km/h`.
- Optional unit (if user setting exists): `m/s`.
- Always include one space between value and unit (for example `12 km/h`, `4.2 m/s`).
- Precision by range:
  - `< 10`: one decimal
  - `>= 10`: integer

## Timestamp Format

- Primary timestamp format: `YYYY-MM-DD HH:mm` (24-hour).
- Use device-local time only in UI labels.
- If weather time is unavailable, show fallback: `---- -- --:--`.
- If data is stale, append stale marker in compact form: `• stale`.

## Weather Icon Mapping

Map normalized weather condition to icon ID consistently:

- `clear_day` -> `icon_sun`
- `clear_night` -> `icon_moon`
- `partly_cloudy_day` -> `icon_partly_cloudy_day`
- `partly_cloudy_night` -> `icon_partly_cloudy_night`
- `cloudy` -> `icon_cloud`
- `rain_light` -> `icon_rain_light`
- `rain_moderate` -> `icon_rain`
- `rain_heavy` -> `icon_rain_heavy`
- `thunderstorm` -> `icon_thunder`
- `snow` -> `icon_snow`
- `fog` -> `icon_fog`
- `windy` -> `icon_wind`
- Unknown condition -> `icon_weather_unknown`

Rules:

- Condition normalization must be case-insensitive.
- Do not infer day/night from local clock if API already provides day/night code.
- Unknown/unmapped code must always fall back to `icon_weather_unknown`.

## Idle Screen Layout Consistency

- Keep primary weather block compact and scanable:
  - Top line: condition + icon
  - Middle line: large temperature
  - Bottom line: wind + humidity + update time
- Use consistent spacing tokens from UI rules (`xs=4`, `sm=8`, `md=12`, `lg=16`).
- Avoid more than 3 text styles on weather area (title/body/meta).

## State Labels

- `loading`: `Loading weather...`
- `success`: no state badge unless debug mode
- `offline`: `Offline`
- `error`: `Weather unavailable`
- `stale`: `Stale data`

## Validation Checklist

1. Temperature text follows `°C` rule and precision policy.
2. Wind text uses configured unit and spacing.
3. Timestamp uses `YYYY-MM-DD HH:mm` or fallback token.
4. Icon mapping uses normalized condition and deterministic fallback.
5. UI state label is one of the standard labels above.
