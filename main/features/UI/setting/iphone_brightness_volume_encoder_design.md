# iPhone-Like Brightness and Volume Settings for ESP32 LVGL (Encoder-Only)

## Scope
This design defines an encoder-first settings screen for brightness and volume with iPhone-like slider behavior, explicit confirm/cancel persistence, and consistent key mapping.

Target storage path requested by user:
- main/features/UI/setting/iphone_brightness_volume_encoder_design.md

---

## 1) Interaction Pattern Spec

### Controls
- Brightness row: icon + label + percent value + slider
- Volume row: icon + label + percent value + slider
- Footer actions: Confirm, Cancel
- Optional title bar: Settings

### Encoder Input Contract
- Rotate clockwise: increase current value when editing, otherwise move to next focusable row
- Rotate counterclockwise: decrease current value when editing, otherwise move to previous focusable row
- Short press:
- On slider row (view mode): enter edit mode for that row
- On slider row (edit mode): leave edit mode and return to row focus
- On Confirm: persist both values atomically and exit
- On Cancel: rollback both values to entry snapshot and exit
- Long press:
- Global cancel-and-exit from anywhere (restore snapshot)

### LVGL Event Mapping Table

All touch gesture behavior must be implemented in LVGL object events (`LV_EVENT_GESTURE`, `LV_EVENT_LONG_PRESSED`) inside the settings screen module. Do not route settings gestures through `LcdTouch::SetGestureCallback`.

| Encoder Event | Screen Mode | LVGL Action | App Action |
|---|---|---|---|
| Rotate + | Row focus | `LV_KEY_NEXT` | Move focus to next row |
| Rotate - | Row focus | `LV_KEY_PREV` | Move focus to previous row |
| Short press | Row focus on slider | `LV_KEY_ENTER` | Enter slider edit mode |
| Rotate + | Slider edit | `LV_KEY_RIGHT` | Increase slider + preview apply |
| Rotate - | Slider edit | `LV_KEY_LEFT` | Decrease slider + preview apply |
| Short press | Slider edit | `LV_KEY_ENTER` | Exit slider edit mode |
| Short press on Confirm | Row focus | `LV_KEY_ENTER` | Commit persistent values |
| Short press on Cancel | Row focus | `LV_KEY_ENTER` | Restore snapshot and exit |
| Long press | Any | App-level cancel | Restore snapshot and exit |

### Data/Persistence Contract
- On screen enter: snapshot current values
- `entry_brightness = board.GetBacklight()->brightness()`
- `entry_volume = board.GetAudioCodec()->output_volume()`
- During edit: apply preview immediately without persisting
- Confirm: persist brightness and volume together
- Cancel/back: restore both snapshot values without persistence

---

## 2) Focus and Navigation Rules

### Focus Order
1. Brightness slider row
2. Volume slider row
3. Confirm
4. Cancel

### Rules
- Exactly one focus target at a time
- Disabled widgets are skipped by traversal
- Entering edit mode pins focus to the active slider until short press exits edit mode
- Long press always escapes to cancel path
- If Confirm/Cancel are hidden (compact mode), focus loops only between slider rows

### Accessibility/Consistency Rule
- The same encoder semantics used in this screen must match all other settings screens to avoid relearning costs.

---

## 3) Long-Press and Repeat Behavior

### Timings
- Long-press threshold: 450 ms
- Repeat start delay: 380 ms
- Repeat interval: 80 ms
- Acceleration phase: after 1500 ms of hold, step increases from 1% to 3%

### Behavior
- Hold while editing slider:
- Repeats value changes at interval
- Never exceeds clamped bounds
- Hold outside edit mode:
- Treated as global cancel shortcut

---

## 4) Edge Cases and Consistency Checklist

### Edge Cases
- No backlight on board (`GetBacklight() == nullptr`): hide brightness row and adjust focus order
- No audio codec (`GetAudioCodec() == nullptr`): hide volume row and adjust focus order
- Apply failure on preview:
- Revert to previous stable value
- Show non-blocking message
- Entering with invalid persisted values:
- Clamp to safe range before rendering
- Brightness floor policy:
- Use floor >= 5% to avoid unreadable UI unless board explicitly allows 0%

### Current Code Audit (Focused)
1. No reusable settings screen exists in main/features/UI/setting yet (folder was empty before this design file).
2. Encoder mapping is feature-local in file browser and not shared globally:
- main/features/file_browser/screens/file_browser_keymap.cc
- Uses `LV_KEY_NEXT`, `LV_KEY_PREV`, `LV_KEY_ENTER`, plus custom long-press context event.
3. Active board in sdkconfig is `CONFIG_BOARD_TYPE_XIAOZHI_AI_IOT_VIETNAM_ES3N28P_LCD_2_8`.
4. In the active board file, touch gesture-based brightness/volume code exists but is currently wrapped in `#if (0)` (disabled):
- main/boards/xiaozhi-ai-iot-vietnam-es3n28p-lcd-2.8/xiaozhi_ai_iot_vietnam_es3n28p_lcd_2.8.cc
- This means no currently enabled settings UX for brightness/volume in the board-specific interaction path.
5. Active board button mapping is limited to BOOT click/long-press actions (chat toggle and file browser), not settings adjustment actions:
- main/boards/xiaozhi-ai-iot-vietnam-es3n28p-lcd-2.8/xiaozhi_ai_iot_vietnam_es3n28p_lcd_2.8.cc
6. Volume persistence currently writes immediately in codec setter:
- main/audio/audio_codec.cc (`SetOutputVolume` writes to Settings("audio")).
7. Brightness persistence is available but optional through permanent flag:
- main/boards/common/backlight.cc (`SetBrightness(uint8_t, bool permanent)`).

### Consistency Checklist
- One canonical encoder action map reused across settings screens
- No screen-specific inversion of rotate direction
- Confirm/cancel semantics identical in every settings page
- No focus traps in encoder-only use
- Long-press always means cancel/back in settings context

---

## 5) Suggested Implementation Steps (Repository Files)

### Step A: Add settings screen module
Create new files:
- main/features/UI/setting/brightness_volume_screen.h
- main/features/UI/setting/brightness_volume_screen.cc

Responsibilities:
- Build LVGL objects and group/focus
- Maintain mode (`row_focus` vs `slider_edit`)
- Keep entry snapshot and dirty tracking
- Expose `ShowBrightnessVolumeSettings()` and `HideBrightnessVolumeSettings()`

### Step B: Add shared encoder action mapper for settings
Create new files:
- main/features/UI/setting/settings_encoder_map.h
- main/features/UI/setting/settings_encoder_map.cc

Responsibilities:
- Convert encoder rotation and press to logical actions
- Emit `LV_KEY_*` for focus/edit transitions
- Handle long-press cancel and repeat timing

### Step C: Integrate with board/application event flow
Integration candidates:
- main/application.cc
- main/boards/xiaozhi-ai-iot-vietnam-es3n28p-lcd-2.8/xiaozhi_ai_iot_vietnam_es3n28p_lcd_2.8.cc

Responsibilities:
- Open settings screen from a deterministic trigger
- Route encoder events to settings mapper while settings screen is active
- Keep touch gesture handling local to LVGL settings screen events
- Restore previous input routing on exit

Note for the current board:
- Hardware encoder input is not wired in the active board path today.
- To run encoder-only UX on this board, add a virtual encoder adapter (for example map BOOT short/long plus swipe gesture deltas to encoder actions), or add physical encoder support in board input handling.

### Step D: Implement preview and commit/rollback logic
Use existing APIs:
- Brightness preview: `Board::GetInstance().GetBacklight()->SetBrightness(v, false)`
- Brightness commit: `SetBrightness(v, true)`
- Volume preview and commit currently both persist via `AudioCodec::SetOutputVolume(v)`

Recommended refinement for clean confirm/cancel:
- Add non-persistent preview setter in audio path (example: `SetOutputVolumePreview(int)`)
- Commit function writes settings only on Confirm
- Cancel restores `entry_volume`

### Step E: Validation
- Encoder-only walk-through:
- Enter screen -> edit brightness -> edit volume -> cancel -> values restored
- Enter screen -> edit both -> confirm -> values survive reboot
- Long-press from any state cancels and exits
- Verify no focus trap when one row is hidden

---

## Minimal Pseudocode

```cpp
onEnter() {
  entry_brightness = backlight ? backlight->brightness() : -1;
  entry_volume = codec ? codec->output_volume() : -1;
  working_brightness = entry_brightness;
  working_volume = entry_volume;
  dirty = false;
}

onBrightnessChanged(v) {
  working_brightness = clamp(v, kMinBrightness, 100);
  if (backlight) backlight->SetBrightness(working_brightness, false);
  dirty = true;
}

onVolumeChanged(v) {
  working_volume = clamp(v, 0, kMaxVolume);
  if (codec) codec->SetOutputVolume(working_volume);  // replace with preview setter if added
  dirty = true;
}

onConfirm() {
  if (backlight) backlight->SetBrightness(working_brightness, true);
  if (codec) codec->SetOutputVolume(working_volume);
  close();
}

onCancel() {
  if (backlight && entry_brightness >= 0) backlight->SetBrightness(entry_brightness, false);
  if (codec && entry_volume >= 0) codec->SetOutputVolume(entry_volume);
  close();
}
```

---

## Deliverable Summary
- Interaction contract defined for encoder-only iPhone-like brightness/volume settings
- Confirm/cancel persistence model specified
- Current key mapping and behavior audited against existing code
- File-level implementation plan provided for direct execution
