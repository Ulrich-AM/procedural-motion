# Architecture

Phase 1–3 foundation is architecturally frozen. This document records the dependency direction a later generator has to keep.

## Ownership

```text
PROC BODY      what the human is        .procbody.json
PROC MOTION    what the human did       .procmo
PROC ANALYSIS  what was measured         .procanalysis.json
```

A motion file does not store contacts, cycles, or relationships. A body file does not store frames. An analysis file does not store a second copy of the motion.

## Dependency direction

```text
BODY
  MOTION
    SOURCE
      IMPORT          BVH, FBX / Assimp, axis conversion
        RETARGET      profile, mapping, SourceSemanticMap
          ANALYSIS    signals, contacts, events, cycles, relationships
            DATASET   index, LAFAN filenames, batch analyzer
              APP / CLI
```

Lower layers do not include the GUI. Analysis does not include ImGui or `ImportSession`. It receives a `SourceClip`, a `ProcMotion`, and a `SourceSemanticMap`.

## One source-joint authority

`RetargetProfile` and a manual mapping produce `std::array<int, kJointCount>`. `semanticsFromMapping` reads that array through `JointId` and fills `SourceSemanticMap`.

```text
profile or manual mapping
          ↓
   SourceSemanticMap
      ↙         ↘
 retarget      analysis
```

Analysis does not keep its own alias list. A toe that the mapping leaves at `-1` copies the ankle, and the file records that provenance. `JointId` matches the frozen `kJointNames` order. The serialized skeleton order does not change.

## Derived analysis state

```text
signals
  ↓
automatic contacts + contact overrides, applied in order
  ↓
events
  ↓
cycles, then canonical mirrors of accepted right cycles
  ↓
measurements and phase curves
  ↓
relationships, excluding mirrored cycles
```

`rebuildDerivedAnalysis` starts at the edited layer:

- A contact edit reapplies every override from `automatic_contact`, then rebuilds events, cycles, mirrors, and relationships. Previous event and cycle overrides are cleared. The note says so.
- An event edit keeps that event, including `automatic_time` and `automatic_type`, and rebuilds cycles onward.
- Accepting or rejecting a cycle updates that raw cycle, drops mirrors, and recreates mirrors only for accepted right cycles. Relationships are refit. A rejected cycle does not keep an accepted mirror.

The same overrides rebuilt twice produce the same document. `PROC_ANALYSIS` stays version 1. `derivation_note` is an optional field on that version.

## Phase 4 boundary

Phase 4 is not started. A generator must not include `SourceClip`, the BVH importer, the FBX importer, Assimp, `DatasetIndex`, LAFAN filenames, the analysis panel, or `proc_motion_analyze`.

```text
datasets → import / retarget → PROC ANALYSIS → reference model
────────────────────────────────────────────────────────────────
PROC BODY + reference model + speed / direction / phase
  → generator → PROC Pose / PROC MOTION
```

The GUI import panel and the batch analyzer still each load and retarget a clip. They share the mapping and `SourceSemanticMap`. A single `processSourceMotion` helper is not worth a rewrite of the import UI.
