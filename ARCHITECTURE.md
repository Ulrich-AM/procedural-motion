# Architecture

Phase 1–3 foundation is architecturally frozen. This document records the dependency direction a later generator has to keep.

## Ownership

```text
PROC BODY              what the human is                         .procbody.json
PROC MOTION            what the human did                       .procmo
PROC ANALYSIS          what was measured                         .procanalysis.json
PROC LOCOMOTION MODEL  gait relationships learned from cycles    .procloco.json
```

A motion file does not store contacts, cycles, relationships, or a locomotion model. A body file does not store frames. An analysis file does not store a second copy of the motion. The locomotion model does not store a source clip.

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

Phase 4 generates locomotion. The offline builder may read `PROC_ANALYSIS`. The runtime generator may not.

```text
OFFLINE
PROC_ANALYSIS
  → accepted straight cycles, canonicalized in the builder
  → PROC_LOCOMOTION_MODEL v1

──────────────── RUNTIME BOUNDARY ────────────────

proc_runtime
  PROC BODY + PROC_LOCOMOTION_MODEL + desired speed + desired heading
    → locomotion generator → PROC pose → FK

proc_core
  import, Assimp, retarget, analysis, dataset, model builder
```

`proc_runtime` links the body, motion core, FK, the locomotion model, and the generator. It does not link Assimp, the BVH importer, the FBX importer, retarget profiles, the dataset index, analysis, or ImGui. `locomotion_generator` does not include headers from `import/`, `retarget/`, `analysis/`, `dataset/`, or `app/`.

`proc_motion_model` is the offline builder. The runtime loader reads only `PROC_LOCOMOTION_MODEL`.

The normalized speed coordinate is `dimensionless_speed = speed_mps / sqrt(g * leg_length)`. `PROC_ANALYSIS` v1 still stores that quantity under the field name `froude_number`. The builder does not read that field. It recomputes dimensionless speed from cycle speed, source leg length, and gravity. The squared Froude number is not what the model samples.

Runtime phase samples are 100 unique values on `[0, 1)`. Phase advance uses `cadence_spm / 120` because one same-side cycle is two steps. Desired speed moves the root. Cadence moves the phase. Model stride length is a reference, not a second root controller.

Walk, run, and sprint each store their observed dimensionless-speed support. Overlap blends the two regimes at the same phase. A gap becomes an inferred transition band. Hysteresis keeps a regime from flickering. Phase is not reset on a transition. Idle is a fade to the bind pose and does not need a clip.

Heading is a runtime yaw of an otherwise straight gait. There is no turning step. Generated contacts are model probabilities written into `.procmo` with `source.type = procedural`. Foot slip is measured during those contacts and is not corrected. There is no foot IK, foot lock, terrain, or physics.

The generator pose and the body-constrained pose stay separate:

```text
PROC_LOCOMOTION_MODEL
  → raw generated pose
  → PROC BODY joint constraints (WARN or CLAMP)
  → runtime pose
```

WARN is the lab default. It reports a violation and leaves the requested pose unchanged. CLAMP rewrites the runtime pose for that frame and does not write the clamped values back into `.procloco`. The offline builder may audit a finished model against a reference body. It does not reshape curves to that body's limits, so the same model can be checked on another body later.

`locomotion_audit` samples each regime, speed knot, and phase, runs the raw generator, and collects WARN violations. `proc_synthetic_model` is the only tool that regenerates `assets/models/synthetic_locomotion.procloco.json`. Unit tests write temporary models. The public file is synthetic repository data, not a measured gait.

Hinge sign follows the axis stored on the PROC BODY. On the canonical skeleton both knees use `+X`, and negative rotation about that axis is flexion. Elbow axes point in opposite directions, so the same numeric quaternion sign is not assumed for both sides. The stored ranges are placeholders.

In-place preview, phase scrubbing, and quarter-cycle ghosts live in the lab. They copy a pose or a state for drawing. They do not change `LocomotionState.root_position`, the baked motion, or the slip sample. Playback has the same display-only horizontal lock. The analyzer and the bone monitor keep the traveling pose. The synthetic walk, run, and sprint templates are offline curve contents in the same `PROC_LOCOMOTION_MODEL` v1 file. Run and sprint are not scaled copies of the walk. Their phase keys are synthetic modeling values. Foot slip stays a measurement.

`proc_motion_validate_gait` is an offline experiment. It may read a local dataset and write `analysis_output/`. It does not link into `proc_runtime`, and it does not install a third-party model into `assets/models/`. A knot built from fewer than three real cycles is low-confidence. The synthetic model remains the deterministic public fixture.

Reference Capture is a separate authoring path in `proc_core` and the lab. It selects analyzed cycles, may label their regime, and may supply a manual speed for an in-place source. Those overrides are not written into `ProcAnalysis`. The same canonicalization, mirroring, phase sampling, and median aggregation build the model. `proc_runtime` does not include the reference set, the `.procref.json` session, or the provenance sidecar. A one-clip model is a single-speed reference. It does not claim a general gait dataset.

The GUI import panel and the batch analyzer still each load and retarget a clip. They share the mapping and `SourceSemanticMap`. A single `processSourceMotion` helper is not worth a rewrite of the import UI. The lab separates PLAYBACK from GENERATOR so those controls stay apart.
