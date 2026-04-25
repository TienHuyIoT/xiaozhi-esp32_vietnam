---
name: lvgl-media-ui-integration
description: 'Integrate media player states and controls into LVGL UI on ESP32-S3. Use when adding playback progress, status, errors, and safe seek/play interactions in LCD apps.'
argument-hint: 'Provide scope: audio-only or audio+video, UI pages involved, and UX constraints.'
user-invocable: true
disable-model-invocation: false
---

# LVGL Media UI Integration

Workflow for connecting media playback state to LVGL-based UI with thread-safe interactions.

## When To Use

- Build playback UI: progress, current time, duration, status, and error display.
- Connect button and slider actions to media control APIs.
- Prevent race conditions from rapid user interactions.

## Inputs

- UI scope: page names and widgets affected.
- Playback scope: `audio-only` or `audio+video`.
- UX constraints: latency target, error behavior, retry strategy.

## Procedure

1. Define app-level media UI state model.
2. Map player events to UI state transitions.
3. Bind state to LVGL widgets (progress bar, labels, status badges).
4. Route UI commands through serialized control APIs.
5. Debounce or throttle seek slider updates.
6. Handle error/recovery states with user-readable messages.
7. Validate rapid input behavior and teardown safety.
8. Update docs with UI mapping and known limitations.

## Quality Gates

- UI reflects playback state consistently.
- Rapid play/pause/seek interactions do not deadlock or race.
- Error states are visible and recoverable.
- Integration remains responsive on target hardware.

## Suggested Prompts

- `/lvgl-media-ui-integration Add playback bar and time labels for audio page.`
- `/lvgl-media-ui-integration Wire seek slider with debounce and safe command serialization.`
- `/lvgl-media-ui-integration Add error banners and retry UX for HTTP stream failures.`
