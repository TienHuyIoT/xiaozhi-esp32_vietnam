---
name: esp32-build-workflow
description: 'Build, flash, monitor, size-check, and diagnose ESP-IDF firmware in this repository. Use when compiling ESP32 firmware, fixing build failures, validating board-specific changes, checking binary size, or preparing firmware flashing from VS Code ESP-IDF commands.'
argument-hint: 'Requested action: build, flash, monitor, size, clean, or diagnose'
user-invocable: true
disable-model-invocation: false
---

# ESP32 Build Workflow

## What This Skill Produces
- A consistent workflow for ESP-IDF build, flash, monitor, size, and rebuild tasks.
- A failure-diagnosis path that distinguishes source errors from environment issues.
- A validation checklist after firmware, board, display, codec, partition, and component changes.
- A concise reporting format for build success, failure root cause, and binary headroom.

## When To Use
- You need to build the current ESP32 firmware.
- You need to flash firmware to the selected board.
- You need to monitor serial output after flashing.
- You need to diagnose ESP-IDF build failures.
- You changed files under `main/**`, `CMakeLists.txt`, `sdkconfig*`, `partitions/**`, `managed_components/**`, or related build-sensitive areas.

## Primary Command Policy
1. Always prefer the VS Code ESP-IDF command path over generic shell commands.
2. Treat ESP-IDF extension-managed actions as the default entry point for:
- build
- flash
- monitor
- buildFlashMonitor
- fullClean
- size
- menuconfig
- setTarget
3. Do not default to bare `idf.py build` in a normal shell unless no ESP-IDF command path is available.

## Environment Rules
1. Do not assume PowerShell, Bash, or VS Code inherited the correct ESP-IDF environment.
2. Prefer ESP-IDF extension-managed terminals and commands.
3. If shell diagnosis is unavoidable:
- verify ESP-IDF environment first
- then run the minimal command needed
4. If extension-managed build works but normal shell build fails, classify it as an environment problem, not a source-code problem.

## Standard Procedures

### Build
1. Run ESP-IDF build command.
2. If build fails, inspect real compiler/linker/configuration errors.
3. Fix the root cause.
4. Run ESP-IDF build again.
5. Report whether build succeeded and summarize the result.

### Flash
1. Build first unless user explicitly confirms using existing artifacts.
2. Run ESP-IDF flash command.
3. Confirm target board/port if needed.
4. Report flash success or the first real blocker.

### Monitor
1. Use ESP-IDF monitor command after flash when runtime validation is needed.
2. Focus on boot logs, crashes, LVGL/runtime assertions, peripheral init failures, and storage/network errors.
3. Summarize key runtime lines instead of dumping full logs.

### Size Check
1. Run size analysis after successful build when memory-heavy changes were made.
2. Report final binary size and free app partition headroom.
3. Call out only warnings that are actionable.

### Clean/Rebuild
1. Prefer targeted cleanup first.
2. Use full clean only when incremental build state is clearly broken.
3. On Windows, if an archive or generated artifact is locked, clean only the affected artifact and retry before escalating.

## Failure Diagnosis Decision Tree

### Code Error
Use this branch when there are:
- compiler errors
- missing symbols
- type mismatches
- LVGL/API misuse
- linker failures caused by source changes

Action:
1. Read the exact error location.
2. Fix source/config issue.
3. Rebuild.

### Configuration Error
Use this branch when there are:
- sdkconfig mismatches
- wrong target
- missing feature flags
- partition conflicts

Action:
1. Identify the configuration mismatch.
2. Adjust config or target.
3. Rebuild.

### Dependency or Managed Component Error
Use this branch when there are:
- component fetch errors
- managed component refresh regressions
- local patch loss after refresh

Action:
1. Confirm whether dependencies were re-downloaded.
2. Re-apply documented local patches if required.
3. Rebuild.

### Environment Error
Use this branch when there are:
- missing ESP-IDF tools
- broken PATH/Python/CMake environment
- extension vs shell mismatch

Action:
1. Fix environment path/problem first.
2. Retry through ESP-IDF command path.

## Validation Rules After Changes
- After changing board, display, codec, media, partition, component-manager, or build-sensitive files, run an ESP-IDF build.
- After memory-heavy feature changes, run size analysis if build succeeds.
- Do not stop at code edits without at least one post-edit build validation when the environment supports it.

## Output Format
When reporting results, include:
1. Build/flash/monitor action performed.
2. Whether it succeeded or failed.
3. Main root cause if it failed.
4. Files changed to fix it.
5. Final binary size and partition headroom when available.
6. Any manual re-apply step if managed components refreshed.

## Repository-Specific Notes
- This repository may require local patch re-application after managed component refresh.
- Media-player related build issues may require extra repository-specific checks.
- Keep build workflow aligned with the actual VS Code ESP-IDF command path used in this workspace.

## Example Prompts
- Use esp32-build-workflow to build the current firmware and summarize any errors.
- Use esp32-build-workflow to flash and monitor the active board.
- Diagnose why this ESP-IDF build failed after my LVGL changes.
- Run a size check after this firmware build and report partition headroom.
