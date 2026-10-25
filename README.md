# Procedural Motion

Fresh research baseline for a new reference-driven procedural human motion study.

The previous physics-first locomotion implementation (Phases 0-5) is intentionally no longer part of the active working tree. It remains available in Git history for failed-experiment evidence and comparison, but new work must not reuse it as an implementation foundation.

## Current direction

The project is being rebuilt around three clean concepts:

1. **PROC BODY** — a canonical humanoid body definition with explicit hierarchy, rest geometry, immutable bone lengths, coordinate conventions, and joint limits.
2. **PROC MOTION (.procmo)** — a body-independent motion format containing root motion, local joint rotations, timing, and optional contact metadata. It must never redefine or scale target bone lengths.
3. **Retargeting / analysis** — later phases will import professional motion sources such as Mixamo, LAFAN1, and AMASS, convert them into the canonical representation, analyze human locomotion, and eventually generate procedural movement that can be physically interrupted.

## Canonical skeleton

The initial skeleton is `proc_humanoid_v0`.

- `headb`: `neckj -> headj`
- `neckb`: `armrootj -> neckj`
- `spine1b`: `legrootj -> spine1j`
- `spine2b`: `spine1j -> spine2j`
- `spine3b`: `spine2j -> armrootj`
- `l/rlegrootb`: `legrootj -> l/rleguppj`
- `l/rleguppb`: `l/rleguppj -> l/rleglowj`
- `l/rleglowb`: `l/rleglowj -> l/rfootj`
- `l/rfootb`: `l/rfootj -> l/rtoesj`
- `l/rarmrootb`: `armrootj -> l/rarmuppj`
- `l/rarmuppb`: `l/rarmuppj -> l/rarmlowj`
- `l/rarmlowb`: `l/rarmlowj -> l/rhandj`
- `l/rhandb`: `l/rhandj -> l/rfingj`

Coordinate convention:

- right-handed
- +X = character right
- +Y = character forward
- +Z = up
- meters internally
- quaternions stored as `[x, y, z, w]`

## Non-negotiable invariant

Motion may rotate joints and translate/rotate the root, but it must **never scale bones or translate non-root joints to force them onto source-animation positions**.

All target bone lengths are owned by the loaded PROC BODY and remain constant during playback.

## Status

Repository reset completed before the new merged foundation phase begins.
