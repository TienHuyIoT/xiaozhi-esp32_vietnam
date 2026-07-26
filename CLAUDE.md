# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Xiaozhi ESP32 Vietnam (`xiaozhi_vn`) is a Vietnamese-community fork of [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) — a voice AI chatbot running on ESP32 chips. It connects to large AI models (Qwen, DeepSeek) over WebSocket or MQTT+UDP, and provides voice interaction, music streaming, radio, OTA updates, and IoT control via MCP.

**Code style**: Google C++ Style Guide. Default language: `vi-VN` (Vietnamese).

---

## Build Commands

Requires ESP-IDF v5.4+. Run commands from the project root with ESP-IDF environment active.

```bash
# First-time setup (must set target before menuconfig)
idf.py fullclean
idf.py set-target esp32s3        # or esp32, esp32c3, esp32p4
idf.py menuconfig                # select board type, language, display, etc.

# Regular build & flash
idf.py build
idf.py -p COM_PORT flash monitor

# Create merged single binary (for web flasher)
idf.py merge-bin
```

Open an IDF shell on Windows using `scripts/open-idf-shell.bat`.

**menuconfig paths for board selection**: `Xiaozhi Assistant` → `Board Type`  
**Language selection**: `Xiaozhi Assistant` → `Default Language`

---

## Architecture

### Entry Point & Application Singleton

[main/main.cc](main/main.cc) calls `Application::GetInstance().Start()`. `Application` ([main/application.h](main/application.h)) is a singleton that owns:
- The active `Protocol` (WebSocket or MQTT+UDP)
- `AudioService` — full audio pipeline
- Media players: `Esp32Music`, `Esp32Radio`, `Esp32SdMusic`, `VideoPlayer`
- `MusicVisualizer` and `SpectrumManager` for FFT display
- Event loop via FreeRTOS `EventGroupHandle_t`

### Board Abstraction

Every hardware target implements the abstract `Board` class ([main/boards/common/board.h](main/boards/common/board.h)):
- `GetAudioCodec()`, `GetDisplay()`, `GetNetwork()`, `StartNetwork()`, `GetNetworkStateIcon()`, etc.
- Register with the `DECLARE_BOARD(ClassName)` macro in the board's `.cc` file.
- Board type is selected via Kconfig (`CONFIG_BOARD_TYPE_*`), mapped in [main/CMakeLists.txt](main/CMakeLists.txt) to the board directory name and font settings.
- Board-specific config lives in `main/boards/<board-name>/config.h` (GPIO pins, display parameters, I2S settings).

**Vietnam-specific boards** live under `main/boards/xiaozhi-ai-iot-vietnam-*`.

Shared board utilities are in [main/boards/common/](main/boards/common/): `WifiBoard`, `Ml307Board`, `DualNetworkBoard`, `Button`, `SdCard`, `Backlight`, `AdcBatteryMonitor`, etc.

### Audio Pipeline

Defined in [main/audio/audio_service.h](main/audio/audio_service.h):

```
MIC → [HPF + AFE Processor] → OPUS encode (16kHz) → Server
Server → OPUS decode → Speaker (24kHz)
```

- Wake word detection uses `esp-sr` (ESP32-S3/P4) or `EspWakeWord` (C3/classic).
- AEC (Acoustic Echo Cancellation) is optional; configured at runtime via `Application::SetAecMode()`.
- Audio codec drivers: ES8311, ES8374, ES8388, ES8389, BOX, or `NoAudioCodec` for direct I2S.

### Communication Protocols

[main/protocols/protocol.h](main/protocols/protocol.h) defines the `Protocol` interface.  
Implementations: `WebsocketProtocol` and `MqttProtocol` (MQTT + UDP).  
Binary audio uses `BinaryProtocol2` / `BinaryProtocol3` structs (packed, OPUS payloads).

### Display

Abstract `Display` base in [main/display/display.h](main/display/display.h). Concrete implementations:
- `OledDisplay` — SH1106/SSD1306 OLED
- `LcdDisplay` — SPI LCD via LVGL 9.x (ST7789, ILI9341, ILI9488, GC9A01, ST7796, etc.)
- `LvglDisplay` — sub-layer used by LCD displays for LVGL rendering, emoji, and GIF playback

Emoji collections (`twemoji_32`, `twemoji_64`) and fonts (`font_puhui_*`, `font_awesome_*`) are set per board in CMakeLists.txt and built into the `assets` partition.

### Features

Feature modules under [main/features/](main/features/):
- `music/` — `Esp32Music` (online, HTTP API), `Esp32Radio` (streaming), `Esp32SdMusic` (SD card), `AudioStreamPlayer` (base player), `LyricManager`
- `spectrum/` — `SpectrumAnalyzer`, `SpectrumRenderer`, `SpectrumManager` (FFT visualization for LCD/OLED)
- `weather/` — `WeatherService`, `WeatherUi` (idle-screen weather display, `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE`)
- `video/` — `VideoPlayer` (AVI from SD card)
- `QRCode/` — `QrcodeDisplay`
- `alarm_clock/` — alarm clock feature
- `why_questions/` — image display for the "why questions" / story-telling feature; polls `server_url/api/image_queue` and shows images on a `% show_why_image` tool-call notification

### MCP Server

[main/mcp_server.h](main/mcp_server.h) implements device-side MCP (Model Control Protocol) — a JSON-based tool-calling interface that lets the LLM control hardware (volume, LED, GPIO, motors). Additional tool registrations in [main/features/mcp_server_features.cc](main/features/mcp_server_features.cc).

### Localization & Assets

- Language strings: `main/assets/locales/<lang>/language.json` → compiled to `main/assets/lang_config.h` by `scripts/gen_lang.py`.
- Audio prompts: `.ogg` files per locale; missing files fall back to `en-US`.
- The `assets` partition (v2 partition table) holds fonts, emoji, and wake word models. Built by `scripts/build_default_assets.py`.
- Kconfig controls what is flashed: default assets, custom assets file/URL, or nothing (`CONFIG_FLASH_DEFAULT_ASSETS` / `CONFIG_FLASH_CUSTOM_ASSETS` / `CONFIG_FLASH_NONE_ASSETS`).

### Partition Tables

Located in [partitions/](partitions/):
- `v1/` — classic layout (no separate assets partition)
- `v2/` — layout with dedicated `assets` partition (default: `partitions/v2/16m.csv` for 16 MB flash)

Default set in `sdkconfig.defaults`. Change via `menuconfig` or override in `sdkconfig.defaults.<chip>`.

---

## Adding a New Board

1. Create `main/boards/<board-name>/` with `<board-name>.cc` and `config.h`.
2. Implement the `Board` subclass; use `DECLARE_BOARD(YourBoardClass)` at file scope.
3. Add a `CONFIG_BOARD_TYPE_*` entry in `main/Kconfig.projbuild`.
4. Add the corresponding `elseif` block in [main/CMakeLists.txt](main/CMakeLists.txt) to set `BOARD_TYPE` and font variables.
5. Choose the appropriate partition CSV and set it via menuconfig.
