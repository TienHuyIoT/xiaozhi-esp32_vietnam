---
description: "ESP32 media implementation standards. Use when editing media playback, stream handling, thread-safe controls, memory tuning, and related docs updates."
applyTo: "main/**,docs/**"
---

# ESP32 Media Instructions

Apply these rules for media-related changes in this project.

## Architecture and Boundaries

- Isolate third-party media APIs behind internal service wrappers.
- Keep command/control flow separate from state/event flow.
- Avoid leaking component-specific types through app-level interfaces.

## Thread Safety and FreeRTOS

- Serialize player commands through a queue or guarded API layer.
- Do not call heavy operations directly from callbacks.
- Keep callback handlers non-blocking and post work to worker tasks.
- Define clear ownership for lifecycle: init, start, stop, deinit.

## Memory and Performance

- Tune FIFO sizes using measured data, not fixed assumptions.
- Record heap impact before and after major media changes.
- Prefer stable playback under memory pressure over aggressive buffering.

## Code Quality

- Use clear and maintainable naming.
- Add concise English comments only for non-obvious logic.
- Follow SOLID, DRY, KISS, and YAGNI principles.

## Validation and Documentation

- Verify local-file and network-stream error paths.
- Include thread-safety checks for concurrent controls.
- After implementation, update matching docs in `docs/` with:
  - What changed
  - Why it changed
  - How to use it
