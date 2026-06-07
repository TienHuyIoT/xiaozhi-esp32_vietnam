---
mode: ask
description: "Generate weather mock UI states (success/loading/offline/error/stale) for rapid LVGL UI validation with realistic sample values and edge cases."
---

Generate reusable weather mock states for embedded UI testing.

## Input

- Screen name: ${input:screen_name:Idle Weather}
- Resolution (W x H): ${input:resolution:320x240}
- Locale: ${input:locale:vi-VN}
- Temperature unit: ${input:temp_unit:C}
- Wind unit: ${input:wind_unit:km/h}
- Forecast slots: ${input:forecast_slots:3}
- Include stale state: ${input:include_stale:yes}
- Output format: ${input:output_format:C++ structs + JSON sample}

## Instructions

Create practical mock data sets and expected UI behavior for these states:

1. `success`
2. `loading`
3. `offline`
4. `error`
5. `stale` (if enabled)

For each state, provide:

- Display text fields (temperature, condition, wind, humidity, location, updated_at).
- Icon key name (mapped weather icon ID).
- Forecast snippet entries (if requested).
- Flags (`is_loading`, `is_offline`, `is_error`, `is_stale`).
- Short UI expectation note (what must be visible/hidden).

## Output Sections

1. Shared schema (field names and types)
2. Mock values by state
3. Edge-case variants
   - Missing timestamp
   - Unknown condition code
   - Very high/low temperature
4. LVGL binding hints
   - Which labels/icons should update per state
   - Which controls should be disabled in loading/offline/error
5. Quick test checklist for manual QA

## Output Style

- Keep values realistic for Vietnam region by default.
- Keep each state compact and copy-paste friendly.
- Prefer deterministic naming for icons and condition codes.
