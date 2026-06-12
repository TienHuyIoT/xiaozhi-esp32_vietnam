---
name: iphone-toggle-segmented-haptics
description: 'Design and implement iPhone-like toggle rows, segmented controls, and haptic feedback behavior for LVGL settings screens on ESP32. Use when defining row interactions, focus/selection logic, and tactile/audio confirmation patterns across touch, encoder, and buttons.'
argument-hint: 'Screen type, available inputs, and haptic/audio capabilities'
user-invocable: true
disable-model-invocation: false
---

# iPhone-Like Toggle Rows, Segmented Controls, And Feedback Patterns

## What This Skill Produces
- A reusable interaction contract for toggle rows and segmented selectors in settings UIs.
- Deterministic focus and selection behavior across touch, encoder, and button input.
- Haptic/audio feedback rules that feel intentional and non-fatiguing.
- An implementation and validation checklist for LVGL firmware projects.

## When To Use
- You are building settings rows that use ON/OFF toggles.
- You need segmented options (for example Low/Medium/High or C/F units).
- You need consistent tactile/audio confirmation patterns across screens.
- You need to remove per-screen interaction quirks and enforce one UX contract.

## Input Capability Matrix
Collect capabilities before implementation:
1. Touch available? (tap, swipe, long-press)
2. Encoder available? (rotate, short press, long press)
3. Dedicated buttons available? (`up/down/left/right/ok/back`)
4. Haptic output available? (ERM/LRA motor, driver, API)
5. Audio click available? (short UI tone path)

## Toggle Row Pattern

### Row Anatomy
- Leading icon (optional)
- Primary label
- Secondary description (optional)
- Trailing switch control

### Behavior Contract
1. Row tap toggles switch state unless row is disabled.
2. Direct tap on switch also toggles state.
3. Encoder/button mode:
- `ok` toggles focused row switch.
- `left/right` may toggle when row is in edit mode.
4. Toggle state applies immediately for preview-safe settings.
5. For risky settings, use confirm dialog before commit.

### Visual States
- `default`
- `focused`
- `pressed`
- `on`
- `off`
- `disabled`

## Segmented Control Pattern

### Structure
- One row with N mutually exclusive options.
- Exactly one active option at all times unless product explicitly allows none.

### Interaction
1. Touch tap on segment selects that segment.
2. Encoder/button mode:
- `left/right` changes selected segment.
- `ok` confirms if explicit confirmation is required.
3. Selection preview applies immediately unless it causes expensive reconfiguration.
4. If reconfiguration is expensive, debounce and display transient applying state.

### Focus Rules
- Row focus first, then segment-level focus when entering edit mode.
- Exiting edit mode returns focus to row container.
- Disabled segments are skipped and never selected by traversal.

## Haptic/Audio Feedback Pattern

### Event-to-Feedback Mapping
- Focus move: very light tick (optional).
- Toggle change: medium confirmation tick.
- Segment change: light tick per step.
- Confirm/save success: short double tick or success tone.
- Error/rejected action: low-frequency buzz or error tone.

### Safety And Fatigue Rules
1. Rate-limit feedback events during rapid repeat.
2. Disable haptic/audio for silent mode if system policy requires.
3. Avoid long haptics for routine navigation.
4. Keep feedback duration short and consistent.

## Branching Logic
1. If haptic is available:
- Use haptic as primary confirmation feedback.
- Use audio as optional secondary.
2. If haptic is unavailable and audio is available:
- Use subtle click tones for key confirmations.
3. If neither available:
- Strengthen visual feedback states and animations.
4. If a setting is disabled by dependency:
- Show disabled reason text and suppress interaction feedback.

## Long-Press And Repeat Rules
1. Long-press `ok` on toggle row:
- Optional quick-jump to details page (only if used consistently).
2. Hold `left/right` on segmented control:
- Repeat with initial delay 350-500 ms and interval 60-120 ms.
3. After 1.5 s hold:
- Optional accelerated traversal for controls with many segments.

## LVGL Implementation Checklist
1. Define shared styles for row, switch, segment states.
2. Use one action mapper for touch/encoder/button to avoid per-screen drift.
3. Keep row widgets focusable and set deterministic focus order.
4. Separate `preview apply` from `persist commit` pathways.
5. Implement feedback helper:
- `EmitUiFeedback(event_type)`
6. Add dependency guards and disabled-state reason labels.
7. Add test screen or QA path for encoder-only and button-only operation.

## Quality Gates
- Toggle rows behave identically across all settings screens.
- Segmented controls enforce single selection with no dead-end focus.
- Feedback is noticeable but not spammy during rapid interaction.
- Disabled controls are discoverable, non-interactive, and clearly explained.
- Save/cancel flows remain deterministic when toggles and segments are mixed.

## Example Prompts
- Use iphone-toggle-segmented-haptics to design a settings page with Wi-Fi toggle and quality segmented selector.
- Audit my LVGL settings screen for inconsistent toggle and segmented control behavior.
- Add haptic/audio feedback mapping for encoder navigation and setting confirmation events.
