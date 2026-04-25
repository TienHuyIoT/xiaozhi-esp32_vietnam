# Media Player Implementation Checklist

Target component: `tempotian/media_player` `v0.5.0`
Reference: https://components.espressif.com/components/tempotian/media_player/versions/0.5.0/readme

## 1) Scope and Requirements

- [ ] Confirm first release scope: `audio only` or `audio + video`
- [ ] Confirm supported source types: `SD card file`, `HTTP/HTTPS`, or both
- [ ] Confirm required controls: `play`, `pause`, `resume`, `stop`, `seek`, `speed`, `loop`
- [ ] Define non-functional requirements: startup latency, max RAM budget, stability target

## 2) Platform and Resource Baseline

- [ ] Lock target to `ESP32-S3`
- [ ] Define RAM budget for player buffers (internal RAM vs PSRAM usage policy)
- [ ] Define task priorities and core affinity for media-related tasks
- [ ] Capture baseline metrics before integration (heap free, CPU load, task count)

## 3) Add and Verify Dependencies

- [ ] Add dependency: `idf.py add-dependency "tempotian/media_player^0.5.0"`
- [ ] Verify dependency resolution and lockfile updates
- [ ] Confirm transitive components are available: `esp_extractor`, `av_render`, `media_src`
- [ ] Run clean build and ensure no compile/link errors from new dependencies

## 4) Project Architecture Integration

- [ ] Create wrapper module (for example `MediaPlayerService`) to isolate third-party API
- [ ] Separate command/control path from state/event path
- [ ] Define thread-safe public API for other tasks/modules
- [ ] Ensure lifecycle model is clear: init -> prepare -> play -> stop -> deinit

## 5) Player and Renderer Initialization

- [ ] Initialize `audio_render` (and `video_render` if video scope is enabled)
- [ ] Build `player_cfg_t` with proper `play_mask`
- [ ] Configure `no_accurate_seek` according to latency vs seek precision requirements
- [ ] Open player instance with `media_player_open()`
- [ ] Validate initialization failure paths and cleanup correctness

## 6) Event Callback and State Mapping

- [ ] Register callback using `media_player_set_callback(...)`
- [ ] Handle key events:
- [ ] `PLAYER_EVENT_SRC_CONNECTING`
- [ ] `PLAYER_EVENT_SRC_CONNECTED`
- [ ] `PLAYER_EVENT_PREPARED`
- [ ] `PLAYER_EVENT_PLAYED_DONE`
- [ ] `PLAYER_EVENT_SEEK_DONE`
- [ ] `PLAYER_EVENT_EOS`
- [ ] `PLAYER_EVENT_PLAY_ERROR`
- [ ] Map player events to internal app events/UI state safely
- [ ] Keep callback non-blocking (defer heavy work to worker task/queue)

## 7) Source Integration

- [ ] Local playback path via `media_player_set_source(..., MEDIA_SRC_TYPE_FILE, ...)`
- [ ] Network playback path via `media_player_set_source(..., MEDIA_SRC_TYPE_HTTP, ...)`
- [ ] Optional custom source path via `media_player_set_source_by_callback(...)`
- [ ] Validate URI/path validation and error reporting behavior

## 8) Playback Control API

- [ ] Implement control commands: play, pause, resume, stop
- [ ] Implement seek via `media_player_seek_time(...)`
- [ ] Implement speed control via `media_player_set_speed(...)`
- [ ] Implement loop control via `media_player_set_loop(...)`
- [ ] Expose position/duration using `media_player_get_position(...)` and `media_player_get_duration(...)`

## 9) Performance and Memory Tuning

- [ ] Configure `player_fifo_cfg_t` based on target workload
- [ ] Test multiple FIFO presets and record heap/latency impact
- [ ] Ensure no long blocking operations in event callback context
- [ ] Evaluate `media_player_interrupt()` and `media_player_recover()` for resource handoff cases
- [ ] Validate operation under low-memory conditions

## 10) UI Integration (LVGL if enabled)

- [ ] Bind player state to UI widgets (progress, position, status, error)
- [ ] Handle seek interactions with debounce/throttle strategy
- [ ] Ensure command serialization to avoid race conditions from rapid UI actions
- [ ] Define user-friendly error messages for playback failures

## 11) Functional Test Matrix

- [ ] Local file playback: MP3/AAC/WAV (+ MP4 if video enabled)
- [ ] Network stream playback with valid URL
- [ ] Network failure path (invalid URL, disconnect, timeout)
- [ ] Seek test at start, middle, and near end of media
- [ ] Loop playback behavior validation
- [ ] Repeated stop/play cycles (stability regression)
- [ ] Long-run test >= 30 minutes to observe leaks/crashes

## 12) Thread Safety Validation

- [ ] Multi-task command stress test (concurrent control requests)
- [ ] Validate command serialization (queue/mutex strategy)
- [ ] Verify no deadlock between callback/event and control APIs
- [ ] Verify safe teardown while playback is active

## 13) Logging and Diagnostics

- [ ] Standardize log tags and levels for media module
- [ ] Record key latency points: source set -> prepared -> started
- [ ] Log detailed error context for `PLAYER_EVENT_PLAY_ERROR`
- [ ] Add optional debug counters/metrics for troubleshooting

## 14) Documentation Updates (Project Requirement)

- [ ] Update corresponding file(s) in `docs/` after implementation
- [ ] Document what changed and why
- [ ] Provide usage guide for the project wrapper API
- [ ] Provide sample flows: local file and HTTP stream
- [ ] Document known limitations and next-step roadmap

## Definition of Done (DoD)

- [ ] Build/rebuild passes on target configuration
- [ ] Core playback scenarios pass (local, network, seek, error path)
- [ ] No obvious memory leak in long-run test
- [ ] Thread-safety checks pass under stress scenarios
- [ ] Documentation is updated and review-ready
