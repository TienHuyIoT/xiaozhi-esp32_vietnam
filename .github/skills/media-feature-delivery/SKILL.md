---
name: media-feature-delivery
description: 'Deliver media features through a phased workflow: plan, design, implement, review, test, and report for ESP-IDF projects.'
argument-hint: 'Provide feature summary, scope (audio-only or audio+video), source (SD/HTTP/both), and constraints (ram-first/latency-first/balanced).'
user-invocable: true
disable-model-invocation: false
---

# Media Feature Delivery

Reusable workflow for complete media feature delivery with consistent quality gates.

## When To Use

- New media playback feature delivery from start to finish.
- Significant refactor of media control/event flow.
- Work requiring clear review, validation, and reporting.

## Inputs

- Feature summary.
- Scope: `audio-only` or `audio+video`.
- Source: `sd`, `http`, or `both`.
- Constraints: `ram-first`, `latency-first`, or `balanced`.

## Phase 1: Plan

1. Confirm objectives and measurable acceptance criteria.
2. Define target files and integration boundaries.
3. Define test matrix and rollout/rollback notes.

## Phase 2: Design

1. Keep third-party media API isolated in a wrapper service.
2. Separate command flow from state/event flow.
3. Define non-blocking callback strategy and task ownership.
4. Define FIFO/memory profile aligned to constraints.

## Phase 3: Implement

1. Add or verify `tempotian/media_player^0.5.0`.
2. Implement source and control APIs.
3. Map player callbacks to app-level states/events.
4. Enforce serialized command handling.
5. Update docs in `docs/`.

## Phase 4: Review

1. Check thread-safety, deadlock, and race risks.
2. Check architecture boundary leakage.
3. Check maintainability and clarity of interfaces.
4. Check docs alignment with behavior.

## Phase 5: Test

1. Validate SD and/or HTTP playback paths.
2. Validate seek, pause/resume, stop, speed, and loop behavior.
3. Validate error and recovery paths.
4. Capture startup latency and heap impact.

## Phase 6: Report

1. Summarize delivered behavior and constraints tradeoffs.
2. List changed files and key decisions.
3. Record validation outcomes and open risks.
4. Propose prioritized next steps.

## Quality Gates

- Build passes on intended target.
- Core scenarios and error paths are validated.
- No obvious race/deadlock in concurrent control operations.
- Docs are updated with what/why/how.

## How To Invoke

- `/media-feature-delivery feature="HTTP stream playback with retry and status mapping" scope=audio-only source=http constraints=balanced`
- `/media-feature-delivery feature="Low-latency A/V local+network playback" scope=audio+video source=both constraints=latency-first`