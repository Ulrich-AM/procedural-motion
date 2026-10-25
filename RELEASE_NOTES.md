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
