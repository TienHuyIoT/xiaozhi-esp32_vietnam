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

## Build Workflow Reference

- For every media change that needs validation, follow `esp32-build.instructions.md` for the actual build workflow.
- Prefer the VS Code ESP-IDF build command path over running generic shell build commands.
- If a media change appears broken only in a normal terminal build, verify the ESP-IDF environment before changing source code.

## Managed Component Patches (REQUIRED after re-download)

When IDF Component Manager re-downloads any managed media component (e.g. via
`idf.py update-dependencies` or by deleting `managed_components/`), manually
applied patches are lost. Re-apply the following patches before building.

### `tempotian__media_player` — CMakeLists.txt

**Root cause:** The upstream `CMakeLists.txt` references `media_src` and
`esp_extractor` as top-level IDF components, but they are bundled as local
libraries inside `libs/` and are never registered as standalone components.
CMake fails with *"Failed to resolve component 'media_src'"*.

Replace the entire contents of
`managed_components/tempotian__media_player/CMakeLists.txt` with:

```cmake
list(APPEND COMPONENT_SRCDIRS
    ./src
    ./libs/media_src
    ./libs/esp_extractor
)

set(COMPONENT_ADD_INCLUDEDIRS
    ./include
    ./libs/media_src/include
    ./libs/esp_extractor/include
)

set(COMPONENT_PRIV_REQUIRES
    esp_timer
    esp_http_client
    tempotian__media_lib_sal
    tempotian__av_render
)

register_component()

# Enable all extractor types (mirrors defaults from libs/esp_extractor/Kconfig)
target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    CONFIG_WAV_EXTRACTOR_SUPPORT=1
    CONFIG_MP4_EXTRACTOR_SUPPORT=1
    CONFIG_TS_EXTRACTOR_SUPPORT=1
    CONFIG_HLS_EXTRACTOR_SUPPORT=1
    CONFIG_OGG_EXTRACTOR_SUPPORT=1
    CONFIG_AVI_EXTRACTOR_SUPPORT=1
    CONFIG_AUDIO_ES_EXTRACTOR_SUPPORT=1
    CONFIG_CAF_EXTRACTOR_SUPPORT=1
    CONFIG_FLV_EXTRACTOR_SUPPORT=1
)

# Link prebuilt ESP extractor library
set(EXTRACTOR_LIB "${CMAKE_CURRENT_SOURCE_DIR}/libs/esp_extractor/lib/${IDF_TARGET}/libesp_extractor.a")
target_link_libraries(${COMPONENT_TARGET} "-Wl,--start-group" ${EXTRACTOR_LIB} "-Wl,--end-group")
```

**What changed vs upstream:**
- `COMPONENT_SRCDIRS` includes `./libs/media_src` and `./libs/esp_extractor` so
  their `.c` source files are compiled as part of this component.
- `COMPONENT_ADD_INCLUDEDIRS` exposes the `include/` dirs of both sub-libraries.
- `COMPONENT_PRIV_REQUIRES` replaces the broken `media_src esp_extractor` names
  with the correct IDF component names (`esp_http_client`, `tempotian__media_lib_sal`,
  `tempotian__av_render`).
- All extractor `CONFIG_*` flags are force-enabled via `target_compile_definitions`
  because the sub-library's `Kconfig` is never processed when it is not a
  standalone component.
- The prebuilt `libesp_extractor.a` is linked explicitly per `IDF_TARGET`.

### `main/features/media/media_player_service.cc` — API compatibility fixes

After re-download, also verify these API mismatches with the upstream headers:

| Issue | Wrong | Correct |
|---|---|---|
| Callback type for `media_player_set_callback` | `int (*)(int, void*)` | `int (*)(player_event_t, void*)` |
| `media_player_get_position` / `get_duration` output arg | `int*` | `uint32_t*` |
| Source type enum values | `MEDIA_SRC_TYPE_FILE`, `MEDIA_SRC_TYPE_HTTP` | `MEDIA_SRC_TYPE_STORAGE`, `MEDIA_SRC_TYPE_NETWORK` |
| URI arg of `media_player_set_source` | `const char*` | `char*` — use `const_cast<char*>(...)` |

Recommended callback registration (avoids exposing `player_event_t` in the header):

```cpp
media_player_set_callback(player_, [](player_event_t event, void* ctx) -> int {
    auto* self = static_cast<MediaPlayerService*>(ctx);
    self->HandlePlayerEvent(static_cast<int>(event));
    return 0;
}, this);
```

## Validation and Documentation

- Verify local-file and network-stream error paths.
- Include thread-safety checks for concurrent controls.
- After media-related fixes, rebuild using the ESP-IDF build workflow documented in `esp32-build.instructions.md`.
- After implementation, update matching docs in `docs/` with:
  - What changed
  - Why it changed
  - How to use it
