# Copilot Customizations for Media Development

This document describes the project-level customizations created for media feature development and how to use them.

## Added Files

- `.github/instructions/esp32-media.instructions.md`
- `.github/prompts/media-player-bootstrap.prompt.md`
- `.github/prompts/media-e2e-delivery.prompt.md`
- `.github/skills/media-player-implementation/SKILL.md`
- `.github/skills/lvgl-media-ui-integration/SKILL.md`
- `.github/skills/media-feature-delivery/SKILL.md`
- `docs/media-player-implementation-checklist.md`
- `docs/media-feature-delivery-workflow.md`
- `docs/media-feature-report-template.md`

## Purpose of Each Customization

### 1) Instructions

File: `.github/instructions/esp32-media.instructions.md`

Use this as always-on guidance for media-related edits in `main/**` and `docs/**`.
It enforces architecture boundaries, thread-safety rules, memory tuning discipline,
and documentation updates.

### 2) Prompts

Files:
- `.github/prompts/media-player-bootstrap.prompt.md`
- `.github/prompts/media-e2e-delivery.prompt.md`

Use prompt files for structured requests with explicit inputs and output contracts.

- `media-player-bootstrap`: fast implementation kickoff with concrete constraints.
- `media-e2e-delivery`: full lifecycle execution across plan, design, implement, review, test, and report.

### 3) Skills

Files:
- `.github/skills/media-player-implementation/SKILL.md`
- `.github/skills/lvgl-media-ui-integration/SKILL.md`
- `.github/skills/media-feature-delivery/SKILL.md`

Use skills for repeatable multi-step workflows. They are stronger than a single prompt
when you need decision logic and quality gates.

## Workflow Docs

- `docs/media-player-implementation-checklist.md`: implementation checklist.
- `docs/media-feature-delivery-workflow.md`: practical usage and slash command examples.
- `docs/media-feature-report-template.md`: reusable delivery report format.

## Role vs Skill vs Agent vs Hook

- Role:
  - Global behavior and quality style for the assistant.
  - Defines how work should be done.
- Skill:
  - Reusable workflow for a specific domain task.
  - Defines what steps to execute and quality checks.
- Agent:
  - Use when you need separated stages, context isolation, or distinct tool policies.
  - Not required for most media implementation tasks in this project.
- Hook:
  - Deterministic command execution around tool lifecycle.
  - Best for enforcing policies (blocking risky operations, auto-checks, formatting).

## Can Multiple Skills Be Used in One Request?

Yes. In practice, you can ask for a combined outcome in one request,
and the assistant can load multiple relevant skills automatically.

Example request:

```text
Implement HTTP audio playback with thread-safe controls and add LVGL progress UI,
following media-player-implementation and lvgl-media-ui-integration workflows.
```

If behavior should be strictly repeatable across the team, consider adding a dedicated
orchestrator skill later.

## Do We Need a Plan?

Yes for non-trivial media features.
Use `docs/media-player-implementation-checklist.md` and
`docs/media-feature-delivery-workflow.md` as planning references.

## Do We Need a Custom Agent Now?

Not mandatory right now.
Current setup (`instructions + prompts + skills + docs`) is sufficient for efficient implementation.
Create a custom agent only when multi-stage automation becomes complex enough to justify it.

## Do We Need Hooks Now?

Not mandatory for initial rollout.
Recommended later if recurring mistakes appear and need deterministic enforcement.

Possible future hooks:
- Block dangerous shell/git patterns.
- Require docs update when media files change.
- Run lightweight checks after media-related edits.
