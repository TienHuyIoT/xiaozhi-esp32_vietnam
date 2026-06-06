# File Browser Module - Integration Summary

## Module Created Successfully ✓

**Location:** `main/features/file_browser/`

## Directory Structure

```
main/features/file_browser/
├── file_browser.h                           # Public API header
├── file_browser.cc                          # Module implementation
├── file_browser_integration_example.cc      # Integration examples
├── README.md                                 # Detailed module documentation
└── screens/
    ├── file_browser_keymap.h                # Encoder input device header
  ├── file_browser_keymap.cc               # Encoder input implementation
    ├── file_browser_state_machine.h         # FSM header
  ├── file_browser_state_machine.cc        # FSM implementation
    ├── file_browser_row.h                   # Row component header
  ├── file_browser_row.cc                  # Row component implementation
    ├── file_browser_scan.h                  # Async scan task header
  └── file_browser_scan.cc                 # Async scan implementation
```

## Files Added to Build System

**main/CMakeLists.txt** updated with:
```cmake
# Source files
"features/file_browser/file_browser.cc"
"features/file_browser/screens/file_browser_keymap.cc"
"features/file_browser/screens/file_browser_state_machine.cc"
"features/file_browser/screens/file_browser_row.cc"
"features/file_browser/screens/file_browser_scan.cc"

# Include directories
"features/file_browser"
"features/file_browser/screens"
```

## Module Components

### 1. **Encoder Input Device** (`file_browser_keymap.cc`)
- Converts encoder rotation to LVGL key events (NEXT/PREV)
- Detects short press (< 350 ms) and long press (>= 450 ms)
- Emits custom `APP_EVT_CTX_MENU_OPEN` event on long-press

### 2. **State Machine** (`file_browser_state_machine.cc`)
- States: BROWSING, MENU_OPEN, CONFIRM_DELETE, LOADING, ERROR
- Modal focus group management
- Focus preservation across state transitions

### 3. **Row Component** (`file_browser_row.cc`)
- Visual representation of file/folder item
- Icon + Name (bold for folders) + Size + Date
- Focus/selected/disabled styling per UI consistency rules
- File type icon mapping: audio 🎵, image 🖼️, video 🎬, etc.

### 4. **Async Folder Scan** (`file_browser_scan.cc`)
- Non-blocking file enumeration with vTaskDelay batching
- Posts progress and completion events to queue
- Auto-detects file type by extension

### 5. **Module API** (`file_browser.cc`)
- `file_browser_init()` - Initialize module
- `file_browser_create_screen()` - Create and return screen
- `file_browser_open_folder(path)` - Open folder (async scan)
- `file_browser_get_state()` - Query current state

## Quick Start

### 1. Initialize Module (in app startup)
```c
#include "features/file_browser/file_browser.h"

void app_init(void) {
    file_browser_init();  // Initialize encoder + scan task
}
```

### 2. Create and Load Screen
```c
lv_obj_t *screen = file_browser_create_screen();
lv_scr_load(screen);
file_browser_open_folder("/sdcard");
```

### 3. Monitor Scan Events (in app update loop, ~100 ms)
```c
QueueHandle_t scan_queue = file_browser_scan_get_event_queue();
file_scan_event_t evt;

if (xQueueReceive(scan_queue, &evt, 0) == pdTRUE) {
    if (evt.type == SCAN_EVT_COMPLETE) {
        uint32_t count;
        const file_item_t *files = file_browser_scan_get_results(&count);
        // Populate UI with files (max 5 visible per 32 px row)
    }
}
```

See `file_browser_integration_example.cc` for complete integration code.

## Next Steps (Before Full Integration)

### 1. **Encoder Driver Integration** (CRITICAL)
- [ ] Confirm rotary_encoder component is included in build
- [ ] Replace placeholder encoder read in `file_browser_keymap.cc`:
  ```c
  int32_t encoder_val = 0;  // ← Replace with actual API
  // Example:
  int32_t encoder_val = rotary_encoder_get_counter(encoder_handle);
  ```
- [ ] Test encoder rotation mapping to list navigation

### 2. **Button / Back Key Integration**
- [ ] Connect GPIO back button to `file_browser_encoder_button_handler(bool pressed)`
- [ ] Set up periodic timer (~50 ms) to call `file_browser_encoder_long_press_check()`
- [ ] Test short press (< 350 ms) and long press (>= 450 ms) behavior

### 3. **File Metadata Support** (OPTIONAL but recommended)
- [ ] Update `file_browser_scan_task()` to call `stat()` for file size/date:
  ```c
  struct stat file_stat;
  if (stat(item->full_path, &file_stat) == 0) {
      item->size_bytes = file_stat.st_size;
      item->date_unix = file_stat.st_mtime;
  }
  ```

### 4. **Icon Assets** (OPTIONAL)
- [ ] Replace emoji icons in `file_browser_row.cc` with actual LVGL image resources
- [ ] Icon paths: folder 📁, audio 🎵, image 🖼️, video 🎬, document 📄, etc.

### 5. **Focus Group Setup** (CRITICAL)
- [ ] Ensure LVGL default group is created before `file_browser_init()`:
  ```c
  lv_group_t *g = lv_group_create();
  lv_group_set_default(g);
  file_browser_init();  // Keymap will use this group
  ```

### 6. **Touch Integration** (Automatic)
- [ ] Ensure lcd_touch driver is initialized for your board
- [ ] File browser screen is LVGL-based; touch events work natively
- [ ] Board: xiaozhi-ai-iot-vietnam-es3n28p-lcd-2.8 (320×240)

### 7. **Application Integration**
- [ ] Copy integration examples from `file_browser_integration_example.cc`
- [ ] Implement actual file operations (delete, rename, copy)
- [ ] Add state handlers for when user selects file/folder
- [ ] Test with real SD card content

### 8. **Testing Checklist**
- [ ] Encoder navigation (rotate → scroll list, wrapping behavior)
- [ ] Short press (select folder, open folder)
- [ ] Long press (open context menu)
- [ ] Back button (go parent, exit screen)
- [ ] Empty folder / error states
- [ ] Folder scan progress feedback
- [ ] Async scan doesn't block UI
- [ ] Touch interactions (tap, swipe, long-press)
- [ ] Focus group management (no focus conflicts)
- [ ] Memory usage on real SD card with many files

## Design Specifications Reference

- **Encoder-first** focus rules: See [lvgl-interaction-patterns.agent.md](../../.github/agents/lvgl-interaction-patterns.agent.md)
- **UI consistency:** See [ui-consistency.instructions.md](../../.github/instructions/ui-consistency.instructions.md)
- **Screen spec:** See [new-screen-spec.prompt.md](../../.github/prompts/new-screen-spec.prompt.md)
- **File browser spec:** See detailed spec in README.md

## Hardware Target

- **Board:** xiaozhi-ai-iot-vietnam-es3n28p-lcd-2.8
- **Display:** 320×240 LCD with touch support
- **Input:** Encoder + push button (back key)
- **Storage:** SD card (FATFS or similar)
- **Chip:** ESP32-S3
- **RAM budget:** ~25-30 KiB for module

## Build Command

```bash
idf.py build
```

If build fails:
1. Check that all source files are listed in CMakeLists.txt SOURCES
2. Verify INCLUDE_DIRS includes both "features/file_browser" and "features/file_browser/screens"
3. Ensure LVGL and FreeRTOS are configured in sdkconfig

## Documentation

- **Module README:** `main/features/file_browser/README.md`
- **Integration Examples:** `main/features/file_browser/file_browser_integration_example.cc`
- **API Docs:** Comments in `file_browser.h` and component headers

---

**Status:** Module created and ready for encoder/button integration and testing.
