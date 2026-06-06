---
mode: ask
description: "Generate a fast embedded screen specification from resolution, input method, and visible item count. Use for LVGL screen planning with layout, interaction, states, and implementation checklist."
---

Create a detailed but implementation-ready screen spec for an embedded UI.

## Input

- Screen name: ${input:screen_name:File Browser}
- Resolution (W x H): ${input:resolution:320x240}
- Input method: ${input:input_method:touch + encoder}
- Visible list items count: ${input:visible_items:5}
- Primary use case: ${input:use_case:Browse files on SD card}
- Must-show metadata: ${input:metadata:name,type,size,date}
- Optional constraints: ${input:constraints:Low RAM, responsive on ESP32-S3}

## Instructions

Produce a screen specification that is practical for LVGL-based embedded firmware.

Return these sections:

1. Goal and user tasks
2. Information architecture
3. Layout blueprint
   - Header
   - Content/list area
   - Footer/action hints
4. Component inventory
   - Each component purpose
   - Required states (default/focused/selected/disabled/loading/error)
5. Interaction model
   - Touch interactions
   - Encoder/button interactions
   - Long-press behavior
6. Navigation and focus rules
7. Empty/loading/error edge states
8. Data formatting rules
   - Size format
   - Date/time format
   - Filename truncation
9. Performance and memory notes for ESP32-class devices
10. Implementation checklist (ordered tasks)

## Output Style

- Keep it concise and engineering-focused.
- Use explicit values derived from resolution and visible item count.
- Avoid generic wording; provide concrete dimensions/spacing proposals.
