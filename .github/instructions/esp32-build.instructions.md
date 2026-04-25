---
description: "ESP32 build and validation workflow. Use when building, rebuilding, checking size, diagnosing build failures, or changing build-related files."
applyTo: "CMakeLists.txt,main/**,managed_components/**,sdkconfig*,partitions/**,docs/**,scripts/**,.github/instructions/**,README*.md"
---

# ESP32 Build Instructions

Apply these rules for build, rebuild, size check, and build-failure diagnosis in this project.

## Primary Build Path

- Always use the VS Code ESP-IDF command path for build operations.
- For build requests, use the ESP-IDF build command/tool instead of generic shell commands.
- Treat `espIdfCommands.build` as the default build entry point.
- Use ESP-IDF commands for related actions as well:
  - `build`
  - `flash`
  - `monitor`
  - `buildFlashMonitor`
  - `fullClean`
  - `size`
  - `menuconfig`
  - `setTarget`

## Environment Rules

- Do not run bare `idf.py build` from a normal terminal unless there is no ESP-IDF command/tool path available.
- Do not assume PowerShell, Bash, or VS Code inherited the correct ESP-IDF environment.
- If terminal-based diagnosis is unavoidable, first verify the ESP-IDF environment is active before invoking `idf.py`.
- Prefer ESP-IDF extension-managed terminals and commands over manually sourcing export scripts.

## Build Request Handling

When the user asks to build the project:

1. Use the ESP-IDF build command/tool first.
2. Read and summarize actual build errors, not just the final failure line.
3. Fix the code or configuration issue.
4. Run the ESP-IDF build command/tool again to confirm the fix.
5. Report binary output, partition headroom, and any residual warnings that matter.

## Failure Diagnosis

- Distinguish between code errors, configuration errors, dependency errors, and environment errors.
- If the build fails because the ESP-IDF environment is missing or mismatched, fix the environment path/problem first.
- If the build fails due to managed component re-downloads, check whether documented local patches must be re-applied.
- On Windows, if archive outputs are locked and CMake reports that a file cannot be removed, clean only the affected build artifact and retry the ESP-IDF build command.
- Avoid destructive cleanup unless necessary; prefer targeted cleanup before `fullClean`.

## Validation After Changes

- After changing media, board, codec, display, partition, or component-manager files, run an ESP-IDF build to validate the change.
- After changing memory-heavy features, also run size analysis if the build succeeds.
- If a build succeeds with warnings, mention warnings only when they are actionable or likely to become failures later.

## Output Reporting

When reporting build results, include:

- Whether the ESP-IDF build succeeded or failed
- The main root cause if it failed
- The files changed to fix it
- The final binary size and free app partition headroom when available
- Any manual re-apply steps required after managed component refresh

## Project-Specific Notes

- This repository may contain managed media components that require local patch re-application after dependency refresh.
- For media-player related failures, also consult `esp32-media.instructions.md`.
- If the build succeeds only through the ESP-IDF command path but fails from a normal shell, treat that as an environment issue, not a source-code issue.
- When editing build workflow docs or repo instructions, keep this file aligned with the actual ESP-IDF command path used in VS Code.
