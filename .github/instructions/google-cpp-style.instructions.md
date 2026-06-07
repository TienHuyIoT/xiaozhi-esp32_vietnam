---
description: "Enforce Google C++ Style for C/C++ development across the project."
applyTo: "**/*.c,**/*.cc,**/*.cpp,**/*.h,**/*.hpp"
---

# Google C++ Style Instructions

Apply this instruction whenever creating or editing C/C++ code.

## Baseline

- Follow Google C++ Style Guide for naming, formatting, includes, and API design.
- Keep compatibility with the repository `.clang-format` (Google-based style).
- Prefer consistency with nearby code when strict style rules conflict with existing module conventions.

## Formatting

- Run `clang-format` on modified C/C++ files before finalizing changes.
- Use braces for all control blocks (`if`, `for`, `while`, `switch` branches).
- Keep line length and wrapping behavior aligned with `.clang-format` output.
- Avoid unrelated reformatting outside the edited scope.

## Naming

- Types/classes: `PascalCase`.
- Functions/methods: `PascalCase` unless module already uses another stable convention.
- Variables: `snake_case`.
- Constants/macros: `kCamelCase` for constants, `UPPER_SNAKE_CASE` for macros.
- Private member fields: trailing underscore (example: `buffer_size_`).

## Includes and Headers

- Keep includes minimal and ordered: C/C++ standard, third-party, project headers.
- Prefer forward declarations when practical to reduce header coupling.
- Ensure each header is self-contained and has include guards.

## API and Implementation

- Favor clear, small functions with single responsibility.
- Use `const` correctness for parameters and methods where applicable.
- Avoid hidden side effects; document non-obvious behavior.
- Handle errors explicitly; do not ignore return values of critical APIs.

## Comments and Documentation

- Use concise comments only when intent is not obvious from code.
- Follow project function comment policy in `function-comment-style.instructions.md` for non-trivial functions and callbacks.

## Pull Request Hygiene

- Keep style-only changes separate from behavior changes when possible.
- Do not rename symbols or reformat whole files unless required by the task.
