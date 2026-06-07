---
description: "Function-level documentation standard. Require concise Doxygen-style comments for non-trivial functions, event handlers, and public APIs so code intent is clear across the project and future agents."
applyTo: "**/*.c,**/*.cc,**/*.cpp,**/*.h,**/*.hpp"
---

# Function Comment Style Instructions

Apply these rules whenever creating or editing functions.

## Goal

- Make each function's intent understandable without reading all internals.
- Keep comments concise, technical, and implementation-relevant.

## Required Comment Format

Use Doxygen-style block comment directly above function declarations/definitions when function is:

- Public API
- Event handler/callback
- Non-trivial internal logic
- State transition/control-flow helper

Template:

/**
 * @brief One-line function purpose.
 * Optional second line: trigger/source/context (event type, caller, assumptions).
 */

## Minimum Expectations

- `@brief` must describe what the function does, not restate the name.
- For event handlers, mention expected event source/type.
- For functions with side effects, mention key side effect (state change, UI navigation, I/O).
- Avoid comments for trivial getters/setters unless behavior is non-obvious.

## Style Rules

- Keep comments short (1-3 lines after `@brief` unless truly needed).
- Use clear English technical wording.
- Do not add vague comments like "Handle event" without context.
- Keep comments synchronized when behavior changes.

## Example

/**
 * @brief Event handler for Refresh button click.
 * Triggers reloading the current folder content.
 */
static void OnRefreshClicked(lv_event_t* e);
