Deliver a media feature end-to-end for this ESP-IDF project.

## Inputs

- Feature: `${feature:<short feature description>}`
- Scope: `${scope:audio-only|audio+video}`
- Source: `${source:sd|http|both}`
- Constraints: `${constraints:ram-first|latency-first|balanced}`
- Target: `${target:esp32s3|other}`

## Phased Workflow

### Phase 1: Plan

1. Confirm goals, assumptions, and non-functional targets.
2. Define acceptance criteria and test matrix.
3. List impacted files and rollback strategy.

### Phase 2: Design

1. Define wrapper boundaries and app-level interfaces.
2. Define event/state mapping and error taxonomy.
3. Define thread-safety model (queue/mutex/task ownership).
4. Define memory/FIFO strategy with expected tradeoffs.

### Phase 3: Implement

1. Add/verify dependency `tempotian/media_player^0.5.0`.
2. Implement source setup and control APIs.
3. Register callbacks and route to app states.
4. Add safeguards for concurrent control calls.
5. Update docs for what changed and how to use.

### Phase 4: Review

1. Check architecture boundaries and API consistency.
2. Check callback non-blocking behavior.
3. Check race/deadlock risks in control flow.
4. Check documentation completeness.

### Phase 5: Test

1. Build on target and run scenario matrix.
2. Validate SD, HTTP, seek, stop/resume, and error paths.
3. Capture latency and heap impact observations.
4. Run stress test for repeated play/pause/seek cycles.

### Phase 6: Report

1. Summarize delivered scope and constraints tradeoffs.
2. List changed files and key design decisions.
3. Publish test outcomes and known limitations.
4. Provide next steps with priority.

## Output Contract

Return sections in this exact order:
1. Plan
2. Design
3. Implementation
4. Review Findings
5. Test Results
6. Final Report