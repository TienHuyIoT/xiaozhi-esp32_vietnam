# File Browser Module - Integration Checklist

Use this checklist to track integration progress and ensure all components are properly connected.

---

## Phase 1: Module Build and Compilation

- [ ] All 10 source files present in `main/features/file_browser/`:
  - [ ] `file_browser.h`
  - [ ] `file_browser.cc`
  - [ ] `file_browser_integration_example.cc`
  - [ ] `README.md`
  - [ ] `INTEGRATION.md`
  - [ ] `screens/file_browser_keymap.h`
  - [ ] `screens/file_browser_keymap.cc`
  - [ ] `screens/file_browser_state_machine.h`
  - [ ] `screens/file_browser_state_machine.cc`
  - [ ] `screens/file_browser_row.h`
  - [ ] `screens/file_browser_row.cc`
  - [ ] `screens/file_browser_scan.h`
  - [ ] `screens/file_browser_scan.cc`

- [ ] `main/CMakeLists.txt` updated with file_browser sources and includes:
  - [ ] 5 source files added to SOURCES
  - [ ] 2 include directories added to INCLUDE_DIRS

- [ ] Run `idf.py build` successfully
  - [ ] No compilation errors
  - [ ] No linker errors
  - [ ] Binary size reasonable (check with `idf.py size`)

---

## Phase 2: Hardware Dependencies

### Encoder Driver
- [ ] Rotary encoder component configured in `idf_components.yml`
- [ ] Encoder GPIO pins defined (e.g., CLK, DT, SW pins)
- [ ] Encoder driver initialized in application before `file_browser_init()`
- [ ] Replace placeholder in `file_browser_keymap.cc`:
  ```c
  // OLD: int32_t encoder_val = 0;
  // NEW: int32_t encoder_val = rotary_encoder_get_counter(encoder_handle);
  ```
- [ ] Test encoder rotation produces LV_KEY_NEXT/LV_KEY_PREV

### Back Button / GPIO
- [ ] Back button GPIO pin defined and initialized
- [ ] Button handler function `file_browser_encoder_button_handler(bool pressed)` connected
- [ ] Periodic timer (50 ms) calls `file_browser_encoder_long_press_check()`
- [ ] Test short press (< 350 ms) triggers ENTER event
- [ ] Test long press (>= 450 ms) triggers APP_EVT_CTX_MENU_OPEN

### Touch / LCD Display
- [ ] LCD controller configured for board (2.8" LCD)
- [ ] Touch controller driver initialized (if supported)
- [ ] LVGL display driver registered with proper resolution (320×240)
- [ ] Test touch tap on row → selects row (if touch enabled)
- [ ] Test long-press on row → opens context menu (if touch enabled)

### SD Card / File System
- [ ] SD card driver initialized (SPI or SDIO mode)
- [ ] FATFS or LittleFS configured and mounted at `/sdcard` or similar path
- [ ] Test folder enumeration works (`opendir()`, `readdir()`)
- [ ] (Optional) `stat()` available for file size/date metadata

---

## Phase 3: LVGL Integration

### Display and Group Setup
- [ ] LVGL initialized and display driver registered
- [ ] Default focus group created:
  ```c
  lv_group_t *g = lv_group_create();
  lv_group_set_default(g);
  ```
- [ ] Before calling `file_browser_init()`

### Module Initialization
- [ ] `file_browser_init()` called in app startup (before main loop)
  - [ ] Encoder keymap driver registered with LVGL
  - [ ] Folder scan task created
  - [ ] Initialization completes without errors

### Screen Creation
- [ ] `lv_obj_t *screen = file_browser_create_screen()` returns valid screen
- [ ] Screen loaded: `lv_scr_load(screen)`
- [ ] Header, content, footer, menu panel, and confirm dialog created
- [ ] Layout matches 320×240 with header 40px, content 160px, footer 40px

---

## Phase 4: Encoder and Button Interaction

### Encoder Rotation
- [ ] Rotate encoder clockwise → Focus moves down in file list
- [ ] Rotate encoder counter-clockwise → Focus moves up in file list
- [ ] List auto-scrolls when focus reaches viewport edge
- [ ] Wrapping behavior: Stops at first/last item (no wrap)

### Encoder Short Press
- [ ] Short press (< 350 ms) on folder → Folder opens, scan starts, LOADING state
- [ ] Short press on file → File select event (callback to be implemented)
- [ ] Short press on menu item → Menu action executes (delete, rename, etc.)

### Encoder Long Press
- [ ] Long press (>= 450 ms) on list row → Context menu opens
- [ ] Menu focus cycles with encoder rotation (wrap enabled in menu)
- [ ] Long press on menu item → No action (only short press executes)
- [ ] Long press while menu open → Menu closes, list focus restored

### Back Button
- [ ] Back short press in subfolder → Go to parent folder
- [ ] Back short press at root → Exit file browser screen
- [ ] Back long press (>= 700 ms) → Jump directly to root folder

---

## Phase 5: State Machine and Events

### State Transitions
- [ ] Initial state = BROWSING
  - [ ] List interactions enabled
  - [ ] Menu hidden
  - [ ] Confirm dialog hidden

- [ ] BROWSING → MENU_OPEN (on long-press)
  - [ ] List frozen (LV_STATE_DISABLED)
  - [ ] Menu becomes visible and focused
  - [ ] Focused row saved before menu opens

- [ ] MENU_OPEN → BROWSING (on back/long-press from menu)
  - [ ] Menu hidden
  - [ ] List enabled
  - [ ] Focus restored to saved row (not top of list)

- [ ] MENU_OPEN → CONFIRM_DELETE (on delete menu item)
  - [ ] Menu frozen
  - [ ] Confirm dialog shown with Cancel focused
  - [ ] User must press Enter on Delete to confirm

- [ ] BROWSING → LOADING (on folder open)
  - [ ] List frozen
  - [ ] Spinner shown in footer
  - [ ] Async scan starts

- [ ] LOADING → BROWSING (on scan complete)
  - [ ] Spinner hidden
  - [ ] List enabled with results
  - [ ] Focus set to first or restored row

### Async Scan Events
- [ ] SCAN_EVT_PROGRESS posted after each batch (every 10 files)
  - [ ] Check with `xQueueReceive(scan_queue, &evt, 0)`
  - [ ] evt.loaded_files increments
  - [ ] UI can show progress indicator

- [ ] SCAN_EVT_COMPLETE posted when folder scan finishes
  - [ ] `file_browser_scan_get_results(&count)` returns file array
  - [ ] All files enumerated correctly
  - [ ] count matches total number of items

- [ ] SCAN_EVT_ERROR on I/O failure
  - [ ] Error message available in evt.error_msg
  - [ ] Error code in evt.error_code
  - [ ] Folder view frozen; Retry button offered

---

## Phase 6: File List and Row Rendering

### Row Creation and Display
- [ ] File list visible in content area (320×160 px)
- [ ] Rows displayed with proper height (32 px each)
- [ ] 5 rows fit exactly in viewport
- [ ] Scrolling shows next/previous items

### Row Content (Icon + Name + Size + Date)
- [ ] File icon displayed correctly (📁, 🎵, 🖼️, etc.)
- [ ] Folder names shown in **bold**
- [ ] File names shown in regular weight
- [ ] Filenames truncated with ellipsis if > 20 chars
- [ ] Size formatted correctly:
  - [ ] Folders show "—"
  - [ ] Files show B, KiB, MiB, GiB with 1 decimal
- [ ] Date formatted as YYYY-MM-DD HH:mm (24-hour)

### Row State Visuals
- [ ] Focused row: Left border highlight (4 px, cyan) + background darkening
- [ ] Unfocused row: No border, transparent background
- [ ] Selected row: Brief flash (200 ms) with accent color
- [ ] Disabled/unsupported row: Dimmed text (gray, 50% opacity)

### File Type Icons
- [ ] Folder (FILE_TYPE_FOLDER) → 📁
- [ ] Audio (FILE_TYPE_AUDIO) → 🎵
- [ ] Image (FILE_TYPE_IMAGE) → 🖼️
- [ ] Video (FILE_TYPE_VIDEO) → 🎬
- [ ] Document (FILE_TYPE_DOCUMENT) → 📄
- [ ] Compressed (FILE_TYPE_COMPRESSED) → 📦
- [ ] Executable (FILE_TYPE_EXECUTABLE) → ⚙️
- [ ] Unsupported (FILE_TYPE_UNSUPPORTED) → ⚠️

---

## Phase 7: Context Menu

### Menu Display
- [ ] Long-press opens centered modal menu (220×180 px)
- [ ] Menu items visible: Open, Rename, Copy, Move, Delete, Properties
- [ ] Menu border distinct (cyan) from main UI
- [ ] Menu appears above list without obscuring critical UI

### Menu Navigation
- [ ] Encoder rotate cycles through menu items
- [ ] Wrapping enabled (top item → bottom item on CW at end)
- [ ] Focused menu item highlighted
- [ ] Unfocused items dimmed

### Menu Actions (Placeholder)
- [ ] Open: Opens folder (if folder selected)
- [ ] Delete: Transitions to CONFIRM_DELETE
- [ ] Rename: (To be implemented)
- [ ] Copy: (To be implemented)
- [ ] Move: (To be implemented)
- [ ] Properties: (To be implemented)

### Menu Close
- [ ] Back button or long-press closes menu
- [ ] List focus restored to originating row
- [ ] State returns to BROWSING

---

## Phase 8: Confirm Dialog (Delete)

### Dialog Display
- [ ] Delete action from menu opens confirm dialog
- [ ] Dialog shows "Delete file?" title in red
- [ ] Two buttons: Cancel (left, safe default) and Delete (right, red)
- [ ] Cancel is focused by default

### User Confirmation
- [ ] Encoder rotate moves focus between buttons
- [ ] Short press on Delete requires explicit confirmation
- [ ] Short press on Cancel closes dialog without action
- [ ] Back key closes dialog without action

### After Confirmation
- [ ] Delete confirmed: File deleted from filesystem
- [ ] Dialog closed, menu restored, state back to MENU_OPEN
- [ ] File list refreshed to remove deleted item
- [ ] Focus adjusted if focused row was deleted

---

## Phase 9: Edge Cases and Error States

### Empty Folder
- [ ] Single placeholder row "No files" displayed (gray, non-focusable)
- [ ] No encoder navigation possible
- [ ] Back button still works to exit folder

### Folder Scan Error (I/O)
- [ ] Error banner displayed in list area
- [ ] Error message shows reason (permission denied, SD card error, etc.)
- [ ] Retry button offered
- [ ] Back button still works

### Unsupported File Type
- [ ] File row visible and focusable
- [ ] Icon shows ⚠️ (warning)
- [ ] Name and size shown but dimmed
- [ ] Open action disabled (can't select)
- [ ] Delete, Properties, and other actions still available

### File Removed During Browse
- [ ] Async scan detects missing file on refresh
- [ ] Focus shifts to nearest surviving row
- [ ] User continues browsing without interruption

---

## Phase 10: Performance and Memory

### Startup
- [ ] `file_browser_init()` completes in < 100 ms
- [ ] Screen creation takes < 200 ms
- [ ] No console errors or warnings

### Folder Scan
- [ ] Async scan doesn't block UI (vTaskDelay batching working)
- [ ] Smooth encoder rotation while scanning
- [ ] Progress updates visible (spinner or count)
- [ ] Scan completes for 100+ files in reasonable time (< 5 sec)

### Memory Usage
- [ ] Check with `lv_mem_monitor()` or heap_trace:
  - [ ] List + 5 rows: ~8-12 KiB
  - [ ] Scan results (100 files): ~25 KiB
  - [ ] Total module: ~40 KiB (safe for ESP32-S3 with 8 MB RAM)
- [ ] No memory leaks detected on repeated open/close

### Rendering
- [ ] List scrolling smooth (30+ FPS target)
- [ ] No dropped frames during encoder rotation
- [ ] LVGL partial redraw enabled (dirty region optimization)
- [ ] No heavy animations causing stuttering

---

## Phase 11: Integration with Application

### Application Callback Hooks (To Be Implemented)
- [ ] `on_file_selected(file_item_t *file)` - User taps/presses file
- [ ] `on_file_deleted(file_item_t *file)` - Confirm dialog confirmed delete
- [ ] `on_file_renamed(file_item_t *file, const char *new_name)` - Rename completed
- [ ] `on_folder_opened(const char *path)` - New folder loaded
- [ ] `on_error(int error_code)` - I/O or state error

### Example Integration
- [ ] Copy integration examples from `file_browser_integration_example.cc`
- [ ] Call `file_browser_init()` in app startup
- [ ] Call `file_browser_create_screen()` and load screen
- [ ] Implement scan event polling in app update loop (every 100 ms)
- [ ] Call `file_browser_get_state()` to monitor state changes

---

## Phase 12: Documentation and Testing

### Documentation Verified
- [ ] `README.md` - Module overview, features, components
- [ ] `INTEGRATION.md` - Integration steps and next actions
- [ ] `file_browser_integration_example.cc` - Code examples
- [ ] API doc comments in `.h` files

### Testing Completed
- [ ] Manual encoder + button test on hardware
- [ ] Manual touch test (if available)
- [ ] Folder navigation test (open/back/root jump)
- [ ] Empty folder test
- [ ] Error state test (invalid path)
- [ ] Scan progress feedback test
- [ ] Menu open/close test
- [ ] Delete confirmation test (without actual deletion first)
- [ ] Focus retention test (open folder, back, check focus)

### Known Limitations / TODO
- [ ] File metadata (size/date) requires `stat()` integration
- [ ] Icon assets are emoji placeholders; replace with real assets
- [ ] File operations (delete, rename, copy) are stub functions
- [ ] Multi-file selection not yet implemented
- [ ] Search/filter functionality not yet implemented
- [ ] Drag-and-drop reorder not implemented

---

## Sign-Off

**Integration Date:** _______________

**Tested By:** _______________

**Status:**
- [ ] All critical phases complete (1-10)
- [ ] Application integration in progress (11)
- [ ] Full test coverage achieved (12)
- [ ] Ready for production use

**Notes/Issues:**
```
[Space for notes]
```

---

**Next Steps After Integration:**
1. Implement file operation backends (delete, rename, copy, move)
2. Add user callback handlers for app-specific actions
3. Integrate with music/video/image players
4. Add settings persistence (last opened folder)
5. Optimize memory footprint for constrained devices
