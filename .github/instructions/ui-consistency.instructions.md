---
description: "UI consistency conventions for embedded file-manager style screens. Use when designing or editing UI in main/display, main/boards, main/assets, or related docs to keep icon mapping, size/date formats, spacing, and typography consistent."
applyTo: "main/display/**,main/boards/**,main/assets/**,main/**/*.h,main/**/*.cc,docs/**"
---

# UI Consistency Instructions

Apply these rules whenever creating or editing embedded UI screens.

## Icon Mapping By File Type

- Folder: use folder icon only.
- Parent/back item: use up-folder or back icon, visually distinct from normal folder.
- Audio files (`mp3`, `wav`, `ogg`, `flac`, `aac`): use audio icon.
- Video files (`mp4`, `mkv`, `avi`, `mov`): use video icon.
- Image files (`jpg`, `jpeg`, `png`, `bmp`, `gif`, `webp`): use image icon.
- Text/config files (`txt`, `json`, `yaml`, `yml`, `ini`, `cfg`, `log`): use document icon.
- Firmware/binary (`bin`, `uf2`): use chip/binary icon.
- Unknown extension: use generic file icon.

Rules:
- Extension matching is case-insensitive.
- If type cannot be resolved, fall back to unknown icon.
- Do not use color alone to indicate file type.

## Size Formatting

- Display units with binary base: B, KiB, MiB, GiB.
- Always keep one space between value and unit (for example `12.4 MiB`).
- Show precision by range:
  - `< 10 KiB`: integer bytes (`842 B`).
  - `10 KiB` to `< 10 MiB`: one decimal (`532.4 KiB`).
  - `>= 10 MiB`: one decimal (`128.7 MiB`).
- For directories where size is unknown, show `--`.

## Date/Time Formatting

- Primary format: `YYYY-MM-DD HH:mm` (24-hour).
- Device-local time only; do not mix local and UTC in the same list.
- If timestamp is unavailable, show `---- -- --:--`.

## Spacing And Layout Rhythm

- Use an 8-point spacing grid.
- Standard spacing tokens:
  - `xs = 4`
  - `sm = 8`
  - `md = 12`
  - `lg = 16`
  - `xl = 24`
- List row minimum height: 40 px on touch screens, 32 px on encoder-first screens.
- Keep left and right list padding symmetrical unless an explicit affordance requires asymmetry.

## Typography

- Use at most 3 text styles per screen:
  - Title
  - Body
  - Caption/meta
- Keep line height between `1.2` and `1.4` for readability.
- Numeric metadata (size/time/index) should use tabular-aligned style when available.
- Truncate long file names with ellipsis and preserve extension when possible.

## Interaction Consistency

- Focused row must have a clear visual state distinct from selected row.
- Long-press should always open contextual actions, not trigger destructive action directly.
- Back action behavior must be consistent across all screens (`go parent` first, then `exit screen`).
- Empty/loading/error states must use consistent structure: icon + short title + actionable hint.

## Validation Checklist

When proposing or changing UI, verify:

1. Icon mapping follows extension rules and fallback behavior.
2. Size/date text strictly follows formatting rules.
3. Spacing aligns to the defined token set.
4. Typography uses only allowed style hierarchy.
5. Focus/selected/disabled states are visually distinguishable.
