---
name: media-player-implementation
description: 'Implement and integrate tempotian/media_player on ESP-IDF projects. Use when adding audio/video playback, SD card or HTTP sources, thread-safe control APIs, event callbacks, RAM/FIFO tuning, and validation checklist for ESP32-S3.'
argument-hint: 'Provide scope: audio-only or audio+video, source type (SD/HTTP), and constraints (RAM, latency).'
user-invocable: true
disable-model-invocation: false
---

# Media Player Implementation

Reusable workflow for integrating `tempotian/media_player` into this ESP-IDF project with clean architecture, thread safety, and performance checks.

## When To Use

- Add media playback (`audio` or `audio + video`) on ESP32.
- Integrate SD-card file playback or HTTP/HTTPS streaming.
- Build a wrapper service around third-party media player APIs.
- Tune memory/FIFO settings for ESP32-S3 stability.

## Inputs

- Product scope: `audio-only` or `audio+video`.
- Source type: `SD file`, `HTTP/HTTPS`, or both.
- Constraints: startup latency target, RAM budget, long-run stability target.

## Decision Points

1. If product needs only sound playback, set play mask for audio only and skip video renderer.
2. If streaming reliability is variable, prioritize robust reconnect/error mapping before advanced seek/speed features.
3. If RAM is tight, start with smaller FIFO presets and increase incrementally using measured heap data.
4. If user experience needs fast seek more than exact positioning, enable non-accurate seek mode.

## Procedure

1. Confirm scope and non-functional targets (latency, RAM, stability).
2. Add dependency: `idf.py add-dependency "tempotian/media_player^0.5.0"`.
3. Run clean build to verify dependency and link integrity.
4. Create a wrapper module (for example `MediaPlayerService`) to isolate external APIs.
5. Initialize audio/video renderers and open player with `player_cfg_t`.
6. Register callback and map core events to internal app state/events.
7. Implement source selection APIs for file and network playback.
8. Implement control APIs: play, pause/resume, stop, seek, speed, loop.
9. Add command serialization (queue/mutex) to ensure thread-safe multi-task control.
10. Tune `player_fifo_cfg_t` based on measured heap and playback smoothness.
11. Validate error paths, long-run stability, and teardown safety.
12. Update docs in `docs/` with changes, rationale, and usage.

## Quality Gates

- Build/rebuild passes on target `ESP32-S3`.
- Core scenarios pass: local playback, network playback, seek, error handling.
- No obvious memory leak in >= 30 minutes run.
- No deadlock/race during concurrent control requests.
- Documentation is updated for team usage.

## Completion Output

- Implemented wrapper and integration points.
- Verified test matrix results.
- Updated project docs with implementation notes and usage guide.

## Suggested Prompt Examples

- `/media-player-implementation Integrate audio-only player from SD card with low RAM profile.`
- `/media-player-implementation Add HTTP stream playback with retry and error-state mapping.`
- `/media-player-implementation Implement thread-safe control APIs and run stability checklist.`
