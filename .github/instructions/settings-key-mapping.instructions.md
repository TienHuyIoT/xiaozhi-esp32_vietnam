---
description: "Use when implementing or refactoring settings screens in LVGL/ESP32 to keep key and input mapping consistent across touch, encoder, and button navigation."
name: "Settings Key Mapping Consistency"
applyTo: "main/display/**,main/boards/**,main/**/*.h,main/**/*.cc"
---

# Settings Key Mapping Consistency

Apply these rules to all settings screens so interaction feels identical everywhere.

## Supported Inputs
- Touch (tap, drag, optional long-press)
- Rotary encoder (rotate, short press, long press)
- Buttons (`up`, `down`, `left`, `right`, `ok`, `back`)

## Canonical Action Model
Define screen behavior using logical actions first, then map hardware events to these actions.

- `NavigateNext`
- `NavigatePrev`
- `AdjustIncrease`
- `AdjustDecrease`
- `EnterOrEdit`
- `Confirm`
- `CancelOrBack`
- `OpenContext` (optional)

Never implement per-screen ad hoc key semantics when a canonical action exists.

## Required Mapping Contract

### List/Row Navigation
- `up` or encoder CCW: move focus to previous focusable row.
- `down` or encoder CW: move focus to next focusable row.
- Focus must skip disabled rows.
- At list boundaries, use one policy consistently within product:
- Wrap mode: last -> first and first -> last.
- Stop mode: hold at boundary.

### Value Controls (Slider/Stepper)
- `left` or encoder CCW while editing: decrement value by step.
- `right` or encoder CW while editing: increment value by step.
- Clamp to min/max always.
- Preview updates must be immediate and non-blocking.

### Enter/OK
- `ok` short press toggles between row focus and active edit mode for value rows.
- On button rows, `ok` triggers primary action.

### Back/Cancel
- `back` always exits current edit mode first.
- If not editing: `back` leaves screen.
- If the screen has dirty state and explicit confirm/cancel flow:
- `back` maps to Cancel and restores entry snapshot.

### Long-Press
- Long-press `back`: force cancel and exit without saving.
- Long-press `ok`: optional fast confirm only if this behavior is used on all settings screens.
- Repeat behavior for hold-to-adjust:
- Initial delay: 350-500 ms.
- Repeat period: 60-120 ms.
- Optional acceleration after 1.5 s.

## Focus Rules
- Deterministic order top-to-bottom, left-to-right.
- Screen entry focus starts at first enabled interactive control.
- Modal dialogs trap focus until dismissed.
- Focus is never lost when controls are hidden/disabled dynamically; move to nearest enabled sibling.

## UI Feedback Rules
- Active edit mode must be visually distinct from row focus mode.
- Disabled rows are visibly de-emphasized and non-focusable.
- Keep iconography and labels stable so key hints remain learnable.

## Persistence Rules
- For adjustable settings, use snapshot-on-enter.
- Changes preview immediately.
- Persist on Confirm.
- Restore snapshot on Cancel.
- Never persist partially if one field commit fails; use atomic commit path for related settings.

## Event Handling Pattern
- Create one shared input-to-action mapper used by all settings screens.
- Keep screen logic action-driven, not hardware-event-driven.
- Route action handling through a common helper so behavior stays synchronized.

## Validation Checklist
- Encoder-only operation can complete every settings flow.
- Button-only operation can complete every settings flow.
- No focus traps across rows, dialogs, or footer actions.
- Back behavior is identical across settings screens.
- Long-press timing and repeat feel identical across settings screens.
- Confirm/Cancel semantics are deterministic and lossless.
