# File Browser Module

Embedded SD card file manager UI for ESP32-based devices with support for dual-input (touch + encoder) navigation.

## Directory Structure

```
features/file_browser/
├── file_browser.h              # Module header (public API)
├── file_browser.cc             # Module implementation
└── screens/
    ├── file_browser_keymap.h   # Encoder input device handler
    ├── file_browser_keymap.cc  # Encoder keymap implementation
    ├── file_browser_state_machine.h  # FSM header
    ├── file_browser_state_machine.cc  # FSM implementation
    ├── file_browser_row.h      # Row component header
    ├── file_browser_row.cc     # Row component implementation
    ├── file_browser_scan.h     # Async folder scan task header
    └── file_browser_scan.cc    # Async folder scan implementation
```

## Features

- **Folder Navigation:** Open/close folders, breadcrumb trail
- **Dual Input:** Both touch (tap, long-press, swipe) and encoder (rotate, press, long-press)
- **Context Menu:** Long-press opens action menu (Open, Rename, Copy, Delete, Properties)
- **Async Scanning:** Non-blocking folder enumeration with progress events
- **UI Consistency:** File type icons, size/date formatting per specifications
- **State Machine:** Clean state transitions (Browsing → Menu → Confirm)
- **Focus Management:** Encoder-first navigation with focus group integration

## Key Components

### 1. Keymap (`file_browser_keymap.cc`)
- Maps encoder rotation → LVGL key events (NEXT/PREV)
- Detects short/long press (thresholds: 350ms / 450ms)
- Posts custom app event `APP_EVT_CTX_MENU_OPEN` on long-press

### 2. State Machine (`file_browser_state_machine.cc`)
- States: BROWSING, MENU_OPEN, CONFIRM_DELETE, LOADING, ERROR
- Handles state transitions and modal focus trapping
- Preserves focus position when entering/exiting modals

### 3. Row Component (`file_browser_row.cc`)
- Renders single file/folder row (icon + name + size + date)
- Focus/selected/disabled visual states
- UI consistency: icon mapping, size/date formatting, truncation

### 4. Async Scan (`file_browser_scan.cc`)
- Enumerates files without blocking UI (vTaskDelay batching every 10 files)
- Posts progress events to queue
- Detects file types by extension

## Usage

### Initialize Module
```c
#include "features/file_browser/file_browser.h"

void app_init(void) {
    file_browser_init();  // Initialize keymap + scan task
}
```

### Create Screen
```c
lv_obj_t *file_browser_screen = file_browser_create_screen();
lv_scr_load(file_browser_screen);
```

### Open Folder
```c
file_browser_open_folder("/sdcard");
// Screen transitions to LOADING
// Async scan posts SCAN_EVT_PROGRESS and SCAN_EVT_COMPLETE to event queue
```

### Monitor Scan Progress
```c
QueueHandle_t scan_queue = file_browser_scan_get_event_queue();
file_scan_event_t evt;

if (xQueueReceive(scan_queue, &evt, 0) == pdTRUE) {
    if (evt.type == SCAN_EVT_COMPLETE) {
        uint32_t count;
        const file_item_t *files = file_browser_scan_get_results(&count);
        // Populate UI with results
    }
}
```

### Get Current State
```c
file_browser_state_t state = file_browser_get_state();
if (state == FB_STATE_BROWSING) {
    // Normal browsing
}
```

## Configuration

### Encoder Timing (in `file_browser_keymap.h`)
- `ENCODER_SHORT_PRESS_THRESHOLD_MS`: 350 ms
- `ENCODER_LONG_PRESS_THRESHOLD_MS`: 450 ms
- `ENCODER_VERY_LONG_PRESS_MS`: 1000 ms (reserved)

### Scan Batching (in `file_browser_scan.h`)
- `FILE_BROWSER_SCAN_BATCH_SIZE`: 10 files per batch
- `FILE_BROWSER_SCAN_YIELD_MS`: 10 ms yield between batches

### Row Layout (in `file_browser_row.cc`)
- Row height: 32 px
- Icon: 24×24 px
- Name: 120 px (bold for folders)
- Size: 40 px (right-aligned, gray)
- Date: 60 px (YYYY-MM-DD HH:mm format)

## Interaction Spec

### Touch
- Tap row → Select + focus
- Swipe up/down → Scroll list
- Long-press row → Open context menu
- Tap Back → Go parent folder
- Tap Refresh → Reload folder

### Encoder
- Rotate CW → Next row, auto-scroll
- Rotate CCW → Prev row, auto-scroll
- Short press (< 350 ms) → Select/open focused row
- Long press (>= 450 ms) → Open context menu
- Back button short press → Go parent folder
- Back button long press (>= 700 ms) → Jump to root

### Menu Navigation
- Rotate → Cycle menu items (wrap enabled)
- Short press → Execute menu action
- Back/long-press → Close menu, restore list focus

### Confirm Dialog
- Default focus: Cancel button
- Require explicit Enter on Delete to confirm
- Back closes dialog without action

## Integration Notes

1. **Encoder Driver:** Replace placeholder `encoder_val = 0` in `file_browser_encoder_read()` with actual rotary_encoder driver API call.

2. **File Metadata:** Update `file_browser_scan_task()` to call `stat()` for size and date if not using a filesystem driver that provides it.

3. **Icon Assets:** Map icon IDs in `file_browser_row.cc` to actual LVGL image resources (currently using emoji placeholders).

4. **Focus Group:** Ensure LVGL default group is set before calling `file_browser_keymap_init()`.

5. **Async I/O:** The scan task uses `vTaskDelay(10ms)` to yield. Adjust based on UI responsiveness requirements.

## Compliance

- Follows [ui-consistency.instructions.md](../../.github/instructions/ui-consistency.instructions.md) for icon mapping, size/date formatting, spacing, typography.
- Follows [lvgl-interaction-patterns.agent.md](../../.github/agents/lvgl-interaction-patterns.agent.md) for encoder navigation and long-press semantics.
- Targets 320×240 display (xiaozhi-ai-iot-vietnam-es3n28p-lcd-2.8).
- Optimized for ESP32-S3 with ~12 KiB RAM budget for list + 5 visible rows.

## TODO / Future Enhancements

- [ ] Full stat() support for file size/date
- [ ] Actual icon asset integration (replace emoji)
- [ ] File delete/rename/copy operations backend
- [ ] Multi-file selection UI
- [ ] Search/filter functionality
- [ ] Breadcrumb click-to-navigate
- [ ] Settings persistence (last opened folder)
- [ ] Drag-and-drop reorder (if touch gesture library available)
