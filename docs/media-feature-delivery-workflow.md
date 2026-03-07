# Media Feature Delivery Workflow

Practical guide for using prompts and skill to deliver media features with predictable quality.

## Available Customizations

- Prompt: `.github/prompts/media-player-bootstrap.prompt.md`
- Prompt: `.github/prompts/media-e2e-delivery.prompt.md`
- Skill: `.github/skills/media-feature-delivery/SKILL.md`

## Recommended Usage

1. Use bootstrap prompt for quick implementation kickoff.
2. Use e2e prompt when you need full lifecycle output.
3. Use skill for repeatable team workflow and quality gates.
4. Use report template for handoff and tracking.

## Exact Slash Command Examples

- `/media-player-bootstrap scope=audio+video source=both constraints=latency-first`
- `/media-player-bootstrap scope=audio-only source=sd constraints=ram-first`
- `/media-e2e-delivery feature="A/V playback from SD and HTTP with fast startup" scope=audio+video source=both constraints=latency-first target=esp32s3`
- `/media-e2e-delivery feature="HTTP audio playback with robust recovery" scope=audio-only source=http constraints=balanced target=esp32s3`
- `/media-feature-delivery feature="Thread-safe media controls with event mapping" scope=audio-only source=both constraints=balanced`

## How Copilot Resolves Customizations

- `instructions` are loaded automatically when matching files/tasks are relevant.
- `skills` can be auto-selected when the description matches your ask, or called via slash command.
- `prompts` are directly invoked with slash command for predictable behavior.
- For deterministic outcomes, call slash commands explicitly.

## Need Order or Not

- No hard order is required.
- For stable team workflow, use this order:
  1. `media-player-bootstrap`
  2. `media-e2e-delivery`
  3. `media-feature-delivery`

## Agent and Hook Guidance

- Custom agent is optional now; use it when multi-stage automation gets complex.
- Hooks are optional now; add them later for deterministic enforcement.

## Plan Requirement

- Plan is required for non-trivial media features.
- Use `docs/media-player-implementation-checklist.md` as plan baseline.

## Notes

- Keep callback handlers non-blocking.
- Serialize player control commands.
- Tune FIFO using measured data, not assumptions.
- Always document what changed, why, and how to use it.