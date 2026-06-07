---
mode: ask
description: "Generate automated weather animation/flicker regression plan and test vectors per state (loading/success/offline/error/stale) for LVGL embedded UI."
---

Create a practical regression test package for weather animation and flicker behavior on embedded LVGL UI.

## Input

- Screen name: ${input:screen_name:Idle Weather}
- Resolution (W x H): ${input:resolution:320x240}
- FPS target: ${input:fps_target:30}
- Refresh interval (seconds): ${input:refresh_interval:60}
- States to test: ${input:states:loading,success,offline,error,stale}
- Transition matrix scope: ${input:transition_scope:all_pairs}
- Flicker tolerance: ${input:flicker_tolerance:no visible full-frame blink}
- Output artifact style: ${input:artifact_style:test checklist + pseudo test harness}

## Instructions

Generate regression tests focused on visual stability and animation continuity for each weather UI state and transitions between states.

For each state and transition, include:

1. Setup preconditions (data payload, time, connectivity, previous state).
2. Stimulus (state change trigger, update cadence, user interaction if any).
3. Expected visual behavior:
   - No full-screen flash
   - No icon disappear/reappear glitch unless required by state
   - No counter-reset animation jump
   - Stable text baseline and alignment
4. Expected timing behavior:
   - Max accepted frame skip budget
   - Max accepted update latency
5. Failure signatures and likely root causes.

## Output Sections

1. State-by-state regression table
2. Transition regression table (source -> target)
3. Test vectors
   - Normal update
   - Rapid consecutive updates
   - Out-of-order timestamp updates
   - Unknown weather condition code
4. LVGL instrumentation hints
   - Which redraw/refr flags and object invalidation points to monitor
   - How to detect over-invalidation patterns causing flicker
5. Pseudo harness template
   - Input state injector
   - Frame-diff checker hooks
   - Assertion examples
6. Pass/fail checklist for CI or manual visual verification

## Output Style

- Keep test cases deterministic and copy-paste friendly.
- Favor concise tables and explicit thresholds.
- Assume ESP32-class constraints and avoid desktop-only testing assumptions.
