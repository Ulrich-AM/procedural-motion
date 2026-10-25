# v0.3.0 — Reference-Driven Procedural Locomotion

Phase 4 generates a straight walk, run, or sprint from a body-independent locomotion model. This release also corrects the public synthetic demonstration so the generated pose flexes the knees, hangs the arms, and moves the ankles, elbows, pelvis, and spine.

The public model is synthetic repository data. It demonstrates the generator, interpolation, regime changes, and joint constraints. It is not a measured human gait, and it is not derived from LAFAN1 or Mixamo.

## What this release adds

- `PROC_LOCOMOTION_MODEL` version 1 (`.procloco.json`).
- A runtime-only `proc_runtime` library. `proc_runtime_link` checks that this library does not pull in Assimp or the importers.
- Dimensionless speed sampling, `speed / sqrt(g * leg_length)`, with gravity `9.80665`.
- Walk, run, and sprint regimes, a periodic phase oscillator, and speed-knot interpolation.
- Body-independent locomotion. The same model runs on another PROC body. Bone lengths stay locked.
- Root-speed control and regime transitions that keep the current phase.
- A generated `.procmo` bake with `source.type = procedural`.
- Foot-slip diagnostics. Slip is measured and not corrected.
- Runtime joint constraints. WARN reports a bad request and leaves it. CLAMP rewrites only the runtime pose. The lab default is WARN.
- A model constraint audit that samples every regime, knot, and phase and does not edit the model.

## Synthetic demonstration correction

- Knee swing uses negative flexion about each knee hinge. On the canonical skeleton that hinge is `+X`.
- The arms leave the T-pose. A rotation from the rest upper-arm direction toward mostly down, plus a contralateral swing, a mild elbow bend on each elbow's own axis, and a small ankle, pelvis, and spine cycle.
- `proc_synthetic_model` regenerates `assets/models/synthetic_locomotion.procloco.json`. Tests write temporary models and do not replace that file.
- The offline builder can print an audit with `--body`. It does not clamp curve samples to one body.

## Checks

Release build, the Phase 1–4 suite, and `proc_runtime_link` pass. The synthetic model audits clean on the neutral body: 300 walk samples, 200 run samples, and 200 sprint samples, with no joint-limit violations. Closed-loop generation at 1.30 m/s measures speed 1.30 m/s, cadence 120 steps/min, and stride 1.30 m. Bone-length error stays under the runtime tolerance. The phase seam of the synthetic model is 0.

## Known limitations

- The committed locomotion model is synthetic. There is no real LAFAN1 or Mixamo gait model in this release.
- There is no foot IK and no foot lock. Contact slip remains. Detected stance is shorter than the authored contact fraction because a straight stance leg on an arc lifts the ankle.
- There is no terrain, balance, physics, active ragdoll, or true turning gait. Heading only yaws the straight cycle.
- Joint limits are preliminary placeholders, not clinical measurements.
- Neck, hands, and fingers stay at identity.

## Package

`proc-motion-lab-v0.3.0-windows-x64.zip` contains the lab, the analyzer, the model builder, the synthetic-model generator, the neutral body, the repository-owned sample motions, the synthetic locomotion model, retarget profiles, the proportion rig, and these notes. It does not contain LAFAN1, Mixamo, analysis output, or local third-party models.

# v0.2.0 — Motion Analysis & Gait Extraction

Phase 3 observes imported locomotion. It does not generate a walk or a run.

`.procmo` is unchanged as a motion file. Measurements, contacts, cycles, and relationships are a separate `PROC_ANALYSIS` version 1 document (`.procanalysis.json`).

## What this release adds

- Source signals for the root, feet, toes, and a virtual heel. The heel is inferred from the source foot. `proc_humanoid_v0` has no heel bone.
- PROC rotational signals as hemisphere-continuous quaternion logs.
- One ground height plus a confidence. A spread-out lower envelope is reported as low confidence.
- Contact confidence with hysteresis. Double support and flight are both kept.
- `initial_contact` and `toe_off` events, same-side stride cycles, explicit rejection reasons, 101-sample phase curves, and canonical mirrors of right strides.
- Source leg length from the source skeleton. Metric fields and normalized fields are separate. Normalized speed is the Froude number.
- Per-regime median, percentile, binned, and straight-line relationships for walk, run, and sprint.
- Dataset index, including LAFAN filename theme / subject / take. A bad file does not stop the scan.
- `proc_motion_analyze.exe` for batch analysis.
- An ANALYSIS section in the lab: plots, timeline marks, and manual event or cycle edits that keep the automatic value.

## Checks

Synthetic walk recovers contacts, stride time, speed, and stride length. Synthetic run keeps a flight phase. Synthetic irregular footwork does not become a set of accepted gait cycles. Source speed, stride, contact time, and source leg length stay the same when the retarget body changes. Phase 1 and Phase 2 tests still pass.

## Known limitations

Ground is a single height. Turning cycles are stored and excluded from the straight-walk and straight-run fits. There is no foot IK, no foot lock, and no procedural locomotion. Real LAFAN1 and Mixamo gait files are not part of this repository. v0.1.0 remains the import and bone-preserving retarget release.

## Package

`proc-motion-lab-v0.2.0-windows-x64.zip` contains `proc_motion_lab.exe`, `proc_motion_analyze.exe`, the neutral body, sample motions, retarget profiles, the proportion rig, and these notes. It does not contain LAFAN1, Mixamo characters, or third-party analysis output.
