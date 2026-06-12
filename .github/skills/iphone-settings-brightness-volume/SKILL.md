---
name: iphone-settings-brightness-volume
description: 'Design and implement an iPhone-like settings screen for brightness and volume in LVGL on ESP32. Use when building slider-based settings menus, encoder/button/touch interaction mapping, live preview behavior, and persistence for screen brightness and speaker volume.'
argument-hint: 'Target board/input profile and requested UX constraints'
user-invocable: true
disable-model-invocation: false
---

# iPhone-Like Brightness And Volume Settings (LVGL)

## What This Skill Produces
- A consistent settings interaction model for brightness and volume similar to iPhone behavior.
- An implementation-ready mapping for touch, encoder, and hardware button input.
- Safety and persistence rules so preview changes feel immediate but never brick UX.
- A validation checklist for responsiveness, readability, and saved-state correctness.

## When To Use
- You are adding or refactoring a settings menu with brightness and volume sliders.
- You need consistent navigation behavior across touch and non-touch devices.
- You need short-press, long-press, and repeat behavior defined before coding.
- You need deterministic persistence rules (preview now, commit later, rollback on cancel).

## Inputs To Collect
1. Display model and brightness control backend (PWM/backlight IC/API).
2. Audio pipeline and output gain control backend (codec/I2S/software volume).
3. Available inputs: touch, rotary encoder, up/down/ok/back buttons.
4. Range and step constraints:
- Brightness min/max and minimum readable floor.
- Volume min/max and mute handling strategy.

## Interaction Contract

### Core Behavior
1. Open screen with current persisted brightness and volume values.
2. Focus lands on the first interactive control (Brightness slider).
3. Any slider adjustment updates preview immediately.
4. Exiting with Confirm saves both values atomically.
5. Exiting with Cancel restores entry values.

### iPhone-Like UX Decisions
1. Brightness and volume sliders use continuous bars with optional icon and numeric percent.
2. Adjustment latency target is low enough to feel real-time (no visible lag).
3. Volume feedback is subtle and non-blocking:
- Optional short sample tone at throttled intervals.
- No repeated loud click on every step if hardware is noisy.
4. Brightness never reaches a value that makes text unreadable unless explicitly allowed by product requirement.

## Control/Event Mapping Table

| Input | Focused Row | Action | Result |
|---|---|---|---|
| Touch tap | Any row | Select row | Row becomes active/focused |
| Touch drag | Slider | Drag thumb | Update preview continuously |
| Encoder rotate CW/CCW | Slider | Increment/decrement | Step value and preview |
| Encoder short press | Row | Enter/confirm row | Toggle active editing state |
| Encoder long press | Screen | Back/cancel | Restore entry values and leave |
| Button Up/Down | List rows | Move focus | Previous/next focusable row |
| Button Left/Right | Slider | Decrement/increment | Step value and preview |
| Button OK | Footer action | Confirm | Persist settings and leave |
| Button Back | Screen | Cancel | Restore entry values and leave |

## Focus And Navigation Rules
1. Focus order:
- Brightness slider
- Volume slider
- Confirm button
- Cancel button
2. If Confirm/Cancel are hidden, focus loops only between sliders.
3. Disabled controls are skipped by focus traversal.
4. If a control becomes disabled while focused, move focus to next enabled control.
5. Screen-level Back always works, even when a slider is active.

## Long-Press And Repeat Rules
1. Initial repeat delay: 350-500 ms.
2. Repeat interval: 60-120 ms for press-and-hold increment/decrement.
3. Accelerated step after 1.5 s hold:
- Use larger delta while preserving hard min/max clamp.
4. Haptic/audio feedback should be rate-limited.
5. Long-press Back cancels without saving from any focused element.

## State Model And Branching Logic
1. On Enter:
- Snapshot entry values (`entry_brightness`, `entry_volume`).
- Load sliders from persisted values.
2. On Slider Change:
- Clamp to allowed range.
- Apply preview immediately.
- Mark `dirty = true` if value differs from entry snapshot.
3. On Confirm:
- Persist both settings in one commit path.
- Clear `dirty` and exit.
4. On Cancel/Back:
- If `dirty`, restore snapshot values.
- Exit without persistence.
5. On Backend Apply Failure:
- Revert failed preview change.
- Show non-blocking error toast/state.

## Quality Gates
1. Functional:
- Preview updates while dragging/rotating.
- Confirm persists and survives reboot.
- Cancel restores original values every time.
2. Interaction consistency:
- Same key mappings across all settings screens.
- No focus trap when using encoder/buttons only.
3. Safety:
- Brightness floor preserves readability.
- Volume cap respects device safe limits.
4. Performance:
- No visible frame drops during slider drag.
- No audio glitch burst during rapid volume steps.

## Implementation Checklist (Repository-Oriented)
1. Add or update screen layout and controls in LVGL screen files under `main/display/` or board-specific UI modules.
2. Wire input device events to a shared action mapper (touch, encoder, buttons).
3. Implement preview apply functions:
- `ApplyBrightnessPreview(percent)`
- `ApplyVolumePreview(percent)`
4. Implement persistence and rollback in settings/domain layer.
5. Add screen enter/exit hooks to manage snapshots and commit/cancel behavior.
6. Add lightweight telemetry/logging for adjustment and save/cancel paths.
7. Validate on real hardware with each available input method.

## Completion Criteria
- User can change brightness/volume quickly with any supported input.
- Cancel path is lossless and always restores entry state.
- Confirm path is atomic and durable.
- Navigation and long-press behavior matches the contract with no special-case surprises.

## Example Prompts
- Design an iPhone-like brightness/volume settings screen for ESP32 LVGL with encoder-only input.
- Implement preview + confirm/cancel persistence for brightness and volume sliders in the current board UI.
- Audit this settings screen for focus traps and inconsistent key mapping using the iphone-settings-brightness-volume skill.
