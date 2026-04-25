Bootstrap a media feature implementation for this ESP-IDF project.

## Inputs

- Scope: `${scope:audio-only|audio+video}`
- Source: `${source:sd|http|both}`
- Constraints: `${constraints:ram-first|latency-first|balanced}`

## Required Deliverables

1. Proposed architecture boundaries (wrapper/service, state flow, callback flow).
2. Concrete implementation plan with target files.
3. Thread-safety strategy for command serialization and callback handling.
4. FIFO and memory tuning proposal with tradeoff rationale.
5. Validation checklist for local and network playback, seek, and error paths.
6. Docs updates in `docs/` with what/why/how.

## Implementation Rules

1. Use or verify dependency `tempotian/media_player^0.5.0`.
2. Keep third-party player APIs behind an internal wrapper.
3. Keep callbacks non-blocking; post heavy work to tasks/queues.
4. Provide control APIs: `play`, `pause`, `resume`, `stop`, `seek`, `speed`, `loop`.
5. Map player events to app-level states/events.
6. Prefer stable playback under pressure over aggressive buffering.
7. Report measurable outcomes (latency, heap delta, stability observations).

## Concrete Example

Input:
- `scope=audio+video`
- `source=both`
- `constraints=latency-first`

Expected direction:
1. Enable audio+video render path with fast-start profile.
2. Configure SD and HTTP source adapters behind one service API.
3. Use small initial FIFO plus adaptive increase on underrun events.
4. Prioritize startup speed and seek responsiveness over max buffering depth.
5. Include risk notes for jittery networks and fallback behavior.

## Output Format

- Files changed
- Design summary
- Validation results
- Risks and next actions

## How To Invoke

- `/media-player-bootstrap scope=audio-only source=sd constraints=ram-first`
- `/media-player-bootstrap scope=audio+video source=both constraints=latency-first`
- `/media-player-bootstrap scope=audio-only source=http constraints=balanced`
