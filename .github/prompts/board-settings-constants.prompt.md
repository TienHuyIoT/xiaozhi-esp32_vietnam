---
name: board-settings-constants
description: "Generate board-specific constants for brightness floor, volume cap, key-repeat timing, and related settings UX limits from one compact input form."
argument-hint: "Board name, display/audio ranges, and desired feel"
agent: "agent"
---

Generate board-specific settings constants from the single input form below.

## Input Form
Fill all fields before generating output.

- Board name:
- Display type (LCD/OLED/other):
- Brightness backend (PWM/backlight IC/API):
- Brightness range (raw min..max):
- Minimum readable brightness target (%) in dark room:
- Maximum comfortable brightness target (%) indoors:
- Audio output path (codec/I2S/software gain):
- Volume range (raw min..max):
- Safe max volume target (%) for speaker protection:
- Mute behavior (hard mute/floor value):
- Input methods (touch/encoder/buttons):
- Desired key repeat feel (gentle/normal/fast):
- Need accelerated hold-to-adjust? (yes/no):

## Requirements
1. Output constants as C/C++ header-friendly macros for ESP32 firmware.
2. Include brightness floor/cap, volume floor/cap, repeat timings, and acceleration settings.
3. Include a short rationale line for each constant.
4. Include one recommended default preset and one conservative-safe preset.
5. Keep naming consistent and uppercase with a shared prefix derived from board name.

## Output Format
### 1) Derived assumptions
- Brief bullet list of inferred assumptions from the form.

### 2) Constants block (`.h` style)
Provide a single fenced C block ready to paste.

### 3) Tuning notes
- 5-10 bullets explaining how to tweak values after on-device testing.

### 4) Validation checklist
- Quick steps to verify readability, loudness safety, and interaction feel on hardware.
