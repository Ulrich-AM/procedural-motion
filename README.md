# Procedural Motion

Phase 1 is a canonical body and a body-independent motion format. The application loads a PROC BODY and a `.procmo` clip, plays the clip on a sphere-and-cylinder humanoid, and checks that every bone keeps the length stored in the body.

Motion may rotate joints and may translate or rotate the root. Motion never scales a bone and never places a non-root joint by writing its position. The body owns hierarchy, rest geometry, bone lengths, joint axes, joint limits, and proportions. The motion owns timing, root translation, root rotation, local joint rotations, and optional contact metadata.

Earlier physics-first work in this repository (Phases 0–5: MuJoCo, balance, stepping, and Y-Bot playback experiments) is historical failed research. It is not the foundation of this code. Those sources are not present on `master`.

## Phase 1 scope

This phase does not try to walk. It freezes the skeleton, the coordinate system, and the two file formats so later importers cannot stretch a limb or invent a second set of proportions.

The runtime character is joints drawn as spheres and bones drawn as cylinders. There is no mesh, no skin, and no physics engine.

## Coordinate convention

Right-handed.

- `+X` is the character's right
- `+Y` is the character's forward
- `+Z` is up

Internal units are meters. Quaternions are stored as `[x, y, z, w]`. The bind pose is a T-pose. Importers convert their source coordinates into this convention before a clip is retargeted or saved as a `.procmo`.

## Canonical skeleton

Skeleton id: `proc_humanoid_v0`.

22 joints, 21 bones. `legrootj` is the root and the pelvis center. `armrootj` is the upper-torso / shoulder center. No clavicles, heels, extra toes, finger chains, or individual vertebrae.

```text
spine1b   legrootj -> spine1j
spine2b   spine1j  -> spine2j
spine3b   spine2j  -> armrootj
neckb     armrootj -> neckj
headb     neckj    -> headj

llegrootb legrootj -> lleguppj
lleguppb  lleguppj -> lleglowj
lleglowb  lleglowj -> lfootj
lfootb    lfootj   -> ltoesj

rlegrootb legrootj -> rleguppj
rleguppb  rleguppj -> rleglowj
rleglowb  rleglowj -> rfootj
rfootb    rfootj   -> rtoesj

larmrootb armrootj -> larmuppj
larmuppb  larmuppj -> larmlowj
larmlowb  larmlowj -> lhandj
lhandb    lhandj   -> lfingj

rarmrootb armrootj -> rarmuppj
rarmuppb  rarmuppj -> rarmlowj
rarmlowb  rarmlowj -> rhandj
rhandb    rhandj   -> rfingj
```

A local rotation lives on the parent joint and orients that joint's outgoing bone. `spine1b` is rigid in the root frame because its parent is `legrootj`. Bending the lower spine is a rotation of `spine1j`, which orients `spine2b`. The rotation at the top of `spine3b` is `armrootj`. There is no `spine3j`.

## PROC BODY

Extension: `.procbody.json`.

```json
{
  "format": "PROC_BODY",
  "version": 1,
  "skeleton": "proc_humanoid_v0"
}
```

Three separate blocks:

1. **Anthropometry** — external reference measurements such as stature, shoulder breadth, hip breadth, hand length, and foot length. They are not joint-center distances and they do not drive bone lengths.
2. **Skeleton geometry** — the authoritative runtime data: joint rest positions, the bone hierarchy, `length_m`, `rest_direction`, and `rest_vector_m`.
3. **Constraints** — joint type (`root`, `hinge`, `ball`, `endpoint`), axes, and limits.

Each bone stores `parent_joint`, `child_joint`, `length_m`, `rest_direction`, `rest_vector_m`, and `"locked": true`. The loader requires:

```text
length(rest_vector_m) == length_m
rest_direction == normalize(rest_vector_m)
child rest position == parent rest position + rest_vector_m
```

Authoring tolerance is `1e-4` m. A locked length cannot be changed by a motion file.

`assets/bodies/neutral.procbody.json` is the sample body. The bind pose is a T-pose with small sagittal offsets through the spine, hip centers offset from `legrootj`, shoulder centers offset from `armrootj`, a slight inward femur, shanks returning toward the ankles, and feet pointing forward with the toe endpoints near the ground. It is a plausible joint-center layout for animation and later retargeting. It is not a clinical model.

Joint limits in that file are marked `"authority": "placeholder"`. They are conservative stand-ins, not measured ranges. Knees are hinges about `+X` from -150 to +10 degrees (negative flexion). Ankles are hinges about `+X` from -40 to +30 degrees. Elbows are hinges from 0 to 150 degrees, left axis `(0,0,-1)` and right axis `(0,0,1)`. Hips, shoulders, spine, neck, and wrists are ball joints with swing and twist caps. Endpoints (`headj`, `ltoesj`, `rtoesj`, `lfingj`, `rfingj`) reject a non-identity rotation. `legrootj` is `root` and is not limited as a local joint.

`tools/human_proportion_rig.html` is a single offline file. It draws this hierarchy, orbits and zooms, and offers Front, Side, and 3/4 views. Anthropometry, bone length, and rest direction are edited separately. Length stays locked while the direction angles change. It validates, imports PROC BODY JSON, and exports a file the C++ loader accepts. Joint labels and bone names with lengths can be toggled.

## `.procmo`

```json
{
  "format": "PROC_MOTION",
  "version": 1,
  "skeleton": "proc_humanoid_v0"
}
```

Animated channels are root translation, root rotation, and local joint rotations. Rotations are normalized quaternions `[x, y, z, w]` in the parent joint's frame. A file must not contain animated scale, per-frame bone length, or a position for any joint other than the root.

A missing `contacts` object means the contact state is unknown. `"left_heel": false` means that contact is explicitly open. Those two states are not the same. Known names are `left_heel`, `left_toe`, `right_heel`, and `right_toe`.

Root translation is an offset from the body's rest position of `legrootj`. `[0, 0, 0]` is the bind pose, so the feet stay where the body put them. Root rotation replaces the root orientation. Only `legrootj` receives that translation. It is applied in world space and is not multiplied by the root rotation.

Forward kinematics:

```text
world position of child = world position of parent + rotate(parent world rotation, rest vector)
world rotation of joint = parent world rotation * local rotation
```

The child translation always comes from the loaded body's rest vector. A hip rotation plus the body's thigh vector produces the knee. The motion never writes the knee position.

Playback samples at the render frame rate. Between source frames, root translation is linearly interpolated and rotations use shortest-path quaternion slerp. Quaternions are normalized on load and again during sampling. Contacts are held from the earlier frame and are not blended. Omitted joint channels are identity. While looping, the interval from the last frame to `duration_s` interpolates toward frame 0.

`leg_test.procmo` drives the right knee to -165 degrees, past the placeholder hinge, so WARN and CLAMP are visible. WARN reports the request and leaves the source rotation unchanged. CLAMP rewrites only the pose used for that frame. The clip in memory is not modified.

## Bone-length invariant

Every rendered pose measures, for every bone:

```text
expected length
world distance from parent joint to child joint
error
```

The runtime tolerance is `1e-5` m. The status line reports `BONE LENGTH VALIDATION: PASS` or `FAIL`, and the side panel lists the bone with the largest error. The same check runs over many random rotations and over every synthetic clip in `proc_motion_tests`.

## Synthetic motions

All five clips are hand-authored. None of them is imported mocap.

| File | What it exercises |
| --- | --- |
| `assets/motions/arm_test.procmo` | Shoulder, elbow, and the hand following the forearm. |
| `assets/motions/leg_test.procmo` | Hip, knee, ankle. The right knee exceeds the placeholder limit. |
| `assets/motions/spine_test.procmo` | `spine1j`, `spine2j`, `armrootj`, `neckj`. |
| `assets/motions/root_motion_test.procmo` | The whole body translates and yaws. Local lengths stay fixed. |
| `assets/motions/walkish_test.procmo` | A looping fake walk: hip swing, knee flexion, opposite arm swing, forward root travel, alternating foot contacts. |

`walkish_test` is an architecture check. It is not a gait. While looping, root translation blends from the end of the clip back to the first frame, so the pelvis slides backward across that gap.

## Build

Requirements: Windows, CMake, and Visual Studio with the C++ workload. The first configure downloads GLFW, GLM, Dear ImGui, nlohmann/json, and Assimp 5.4.3. Assimp is built as a static FBX importer only.

```text
build_windows.bat
run_windows.bat
```

`build_windows.bat` configures and builds Release, writes `build_log.txt`, pauses if configure or build fails, and prints the executable path on success:

```text
build\Release\proc_motion_lab.exe
```

`run_windows.bat` starts that executable from the repository root. If the executable is missing it says so and names the paths it looked for. Tests:

```text
build\Release\proc_motion_tests.exe
ctest --test-dir build -C Release
```

Passing a `.procbody.json` path to the test executable loads that file and prints the body name, or prints the validation errors.

On startup the lab loads `assets/bodies/neutral.procbody.json` when it can find `assets` by walking up from the working directory or from the executable. A `.fbx` or `.bvh` path on the command line is imported after that and shown in the overlay view. A `.procmo` path is loaded as motion. Another path is loaded as a body.

## Controls

The window is a compact research tool: Arial, grayscale, sharp corners, a dark viewport, and a flat side panel.

- Left mouse drag in the viewport orbits. The wheel zooms.
- Front, Side, 3/4, and Reset camera.
- Load or reload a `.procbody.json` and a `.procmo`. Dropping a `.procmo` loads motion. Dropping a `.fbx` or `.bvh` imports source motion. Dropping other JSON loads a body.
- Transport: jump to start, previous source frame, play/pause, next source frame, jump to end. Space toggles play.
- Timeline scrubber, loop toggle, and speeds 0.25×, 0.5×, 1×, 2×.
- Joint limits: WARN or CLAMP.
- Optional root trajectory. Foot contacts draw as discs. A heel marker is drawn at the foot joint because this skeleton has no heel bone. A toe marker is drawn at the toe joint. White means contact, dark means explicitly open, and a missing contact is not drawn.
- A short peg on the head points along local `+Y`. It is a facing marker, not a bone.

The status line shows the source clip, the target body, frame, bone-length error, rest-retarget error, joint-limit warnings, and PASS or FAIL.

## Validation

Body load rejects a bad format or version, a wrong skeleton id, a missing or duplicate required joint or bone, a broken parent or child name, a cycle, a disconnected hierarchy, a non-positive length, a non-finite rest vector, a length that disagrees with the rest vector, a rest direction that is not that vector's normal, and a root that is not `legrootj`.

Motion load rejects a bad format or version, a different skeleton id, times that do not strictly increase, a non-finite root transform, a quaternion that is non-finite or degenerate, an unknown joint name, animated scale, a translation on a non-root joint, and a contact value that is not a boolean. A quaternion that is finite but not unit length is normalized, and a warning is recorded when it was far from unit length.

Failures are reported as text. The process does not abort.

## Tests

`proc_motion_tests` covers:

- the neutral body loads, and a missing bone, a length that disagrees with its rest vector, a non-positive length, and a wrong parent are rejected
- identity local rotations reproduce the rest pose
- rotating one joint moves its descendants and leaves another branch still
- root translation moves every joint by the same offset
- root rotation orbits the hierarchy around `legrootj`
- many random valid rotations keep every world bone length within `1e-5` m
- a valid `.procmo` loads, a zero quaternion is rejected, a non-unit quaternion is normalized, scale and non-root translation are rejected
- the interpolation midpoint, slerp length, exact frame hits, and the difference between a missing contact and `false`
- loading the same body and motion twice yields the same sampled positions
- all five clips, sampled under both WARN and CLAMP, keep every bone length

## Phase 2 — motion import and bone-preserving retargeting

Phase 2 imports a foreign humanoid clip, reconstructs that clip's own rest pose and animated transforms, and writes a `.procmo`. The PROC BODY still owns bone lengths. A source thigh of 0.47 m retargeted onto a PROC thigh of 0.39 m copies the source thigh's orientation onto the target hip and keeps the target's 0.39 m bone. The target knee is not moved to the source knee. If the target foot does not land where the source foot landed, that is expected. Foot correction is not part of this phase.

```text
Mixamo FBX ─┐
            ├─> SourceClip ─> Retargeter ─> ProcMotion ─> .procmo ─> any PROC BODY
LAFAN1 BVH ─┘
```

FBX and BVH never write a `ProcMotion` directly. The retargeter does not contain Mixamo, FBX, or BVH coordinate branches. Each importer finishes in the PROC convention: right-handed, +X right, +Y forward, +Z up, meters, quaternions.

`SourceClip` stores, for every source joint, the rest local and rest global transform, and for every frame the animated local and global transform. Non-root source translations stay in the clip so the source skeleton can be previewed. They are logged and are not written into `.procmo`.

### BVH

`src/import/bvh_importer.cpp` is a native parser. It reads `HIERARCHY`, `ROOT`, `JOINT`, `End Site`, `OFFSET`, `CHANNELS`, `MOTION`, `Frames`, and `Frame Time`. Rotation channels are applied in the order the file declares and converted to quaternions immediately. Downstream code does not see the original Euler order. A position channel replaces the corresponding component of `OFFSET`. It is not added on top of the offset.

### FBX

FBX is read with Assimp. The importer keeps the node hierarchy, rest transforms, translation keys, rotation keys, and timing. Meshes, materials, textures, skin weights, and blend shapes are ignored. If the rest pose extends past 10 units on any axis, translations are treated as centimeters and scaled to meters inside the FBX importer. A retarget profile can still name the file's up and forward axes. Local FBX quaternions are not copied into PROC.

### Bind-relative retarget

For a mapped source joint `j`:

```text
D(j) = animated_global(j) * inverse(rest_global(j))
target_global(j) = D(j) * target_rest_global(j)
target_local(j) = inverse(target_global(parent)) * target_global(j)
```

`target_local` is what `.procmo` stores. The target rest basis comes from forward kinematics of the loaded body, so the math does not assume that basis is identity. A PROC joint with no source mapping inherits its parent's desired global rotation. That lets `larmuppj` take the global rotational delta of Mixamo `LeftArm` without adding a clavicle to `proc_humanoid_v0`. The source arm's global orientation already includes the shoulder ancestors.

### Profiles and mapping

`config/retarget/mixamo.json` and `config/retarget/lafan1.json` map PROC joint names to source semantic names, with aliases. `mixamorig:` is stripped for matching and the original source name is kept. A profile is selected only when every required mapping is present. Mixamo detection also requires a `mixamorig` token, so a bare LAFAN skeleton is not labeled Mixamo. The lab can auto-detect, load a profile, edit each PROC joint from a source-joint list, save a custom mapping JSON, and reset the mapping. A different humanoid does not require a C++ change.

Before the animation is trusted, the source rest pose is retargeted and compared with the PROC rest pose. The panel reports each mapped joint's angular error and the maximum. A large rest error, such as a sideways knee or a twisted shoulder, is a mapping or basis problem.

### Root motion and export

Root translation is the source root's animated position minus its rest position, in meters, after the importer's coordinate conversion. It is not scaled to the target's height or leg length. Two preview options exist and start off: rebase the horizontal start to the origin, and rebase the initial facing direction to +Y. Exported `.procmo` files use `PROC_MOTION` version 1: source metadata, timing, root translation, root rotation, and local joint rotations. No bone positions, no non-root translation, no scale, and no bone lengths. Joint-limit CLAMP affects the preview only. The file keeps the raw retarget so a second body can clamp the same clip with its own limits. Contacts are omitted unless a future source actually provides them. A missing contact stays unknown. The importer does not write `false`.

### Viewer

The viewport is TARGET, SOURCE, or OVERLAY. TARGET draws the PROC skeleton. SOURCE draws the imported skeleton at its own proportions. OVERLAY shifts the source root onto the PROC root and draws the source as a thinner, darker chain under the thicker PROC skeleton. The shift is a translation. It does not rotate the source, so an axis error stays visible. There is no mesh and no IK.

The source panel shows filename, format, clip name, duration, nominal FPS, joint count, root, detected units, coordinate conversion, profile, mapped and unmapped joints, animated translation notes, and a hierarchy browser. Diagnostics include required mappings, rest angular error, bone-length validation, joint-limit warnings, source translation notes, and `PROC non-root translations exported: 0`. A segment-direction error compares source and target bone directions and ignores length, so a proportion difference is not reported as an orientation error.

### Local Mixamo and LAFAN1

Third-party files stay on the machine that owns the license. The repository does not contain Mixamo FBX, LAFAN1 BVH, AMASS, SMPL, or converted third-party `.procmo` files. `datasets/`, `local_assets/`, `converted/`, `imports/`, `*.fbx`, and `*.bvh` are ignored, except the synthetic fixtures under `tests/fixtures/`.

Mixamo:

```text
download an FBX locally
drop it on the lab, or use Load FBX/BVH
confirm the profile reads Mixamo and the required mapping is complete
inspect TARGET, SOURCE, and OVERLAY
export .procmo if you want a file
```

LAFAN1:

```text
point the lab at a local .bvh
the BVH importer and config/retarget/lafan1.json do the rest
preview, then export a .procmo only if you need one outside the repository
```

The LAFAN profile treats the file as Y-up, +Z forward, centimeters. That axis choice has not been checked against a local LAFAN clip. The Mixamo profile treats Assimp's node axes as Y-up, +Z forward, and leaves the unit scale at 1 because the FBX importer already converts a centimeter-sized file. The right axis is not stored separately. It is `forward × up`, which keeps the frame right-handed (`right × forward = up`). On the local Mixamo character that puts character right on -X, the left arm on -X, and the legs down -Z. If a real file disagrees, fix the profile's `coordinates` block. Do not add a special case inside the retargeter.

### Expected limitations

These are part of Phase 2, not defects to hide:

- Feet can slide. There is no foot IK and no foot lock.
- `.procmo` does not store contacts. Phase 3 writes detected contacts into a separate `PROC_ANALYSIS` file.
- A shorter or longer PROC leg will not reproduce the source stride length.
- Root travel is the source's metric travel. It is not scaled to the target.
- Joint limits are still the Phase 1 placeholders.
- There is no mesh, no AMASS importer, and no procedural locomotion.

## Phase 2 tests

`proc_motion_tests` also covers:

- `tests/fixtures/synthetic_source.bvh` parses hierarchy, offsets, root translation, a non-XYZ channel order, an animated non-root position, and the expected quaternions
- a synthetic source with a twisted hip rest retargets to the anatomical PROC rotation rather than a copy of the source local quaternion
- one `.procmo` played on the neutral body and on a body with shorter thighs and wider hips keeps the same local rotations, moves the knee in world space, and preserves each body's own bone lengths
- writing a retargeted `.procmo` and loading it again matches root transforms and joint rotations
- the Mixamo profile matches a `mixamorig:` skeleton, and the synthetic FBX fixture imports to a `SourceClip`

## Non-goals

This tree does not implement MuJoCo, Box2D, rigid-body physics, balance or stepping controllers, SIMBICON, procedural walking, foot IK, foot locking, terrain reconstruction, a motion-matching database, active ragdoll, AMASS, SMPL, video pose detection, MediaPipe, machine learning, mesh skinning, a Y-Bot mesh, or auto-rigging.

Phase 3 observes gait. It does not generate a walk or a run.

## Roadmap

Phase 4 is not started. The next step is an architectural audit of PROC BODY, PROC MOTION, SourceClip, the importers, the retargeter, MotionPlayer, PROC ANALYSIS, the dataset layer, and the GUI/CLI boundary. A procedural generator waits until that review.

## Phase 3 — motion analysis and gait extraction

Phase 3 watches imported locomotion. It detects contacts, splits same-side strides, normalizes them by phase, and stores measurements and simple speed relationships. It does not synthesize a pose.

`.procmo` stays a motion file. Analysis lives in `PROC_ANALYSIS` version 1 (`.procanalysis.json`). Detected contacts, confidence, rejected cycles, and fitted relationships are not written back into the motion.

### What is measured where

Source geometry is measured on the `SourceClip`: foot and toe position, the virtual heel, root travel, ground height, contacts, and source leg length. Those numbers stay the same if the loaded PROC body changes from Neutral to an ANSUR preset.

Joint rotation is measured on the retargeted PROC pose: pelvis, spine, hips, knees, ankles, shoulders, and elbows. Rotations are stored as a normalized quaternion plus a hemisphere-continuous log vector. If `dot(previous, current)` is negative, `current` is negated before the log. Scalar plots use the local X component of that log as sagittal flexion. Pelvis roll is the local Y component. Raw quaternion components are not averaged.

Source leg length is the mean of the available sides of rest `(hip → knee) + (knee → ankle)` on the source skeleton. Normalized stride, step, clearance, and pelvis height divide by that length. Normalized speed is the Froude number `speed / sqrt(9.80665 * source_leg_length)`. Metric and normalized values use different fields.

### Signals, ground, contacts

Positions are smoothed with a short symmetric window specified in seconds (default 50 ms), then differentiated with a central difference, then the velocity is smoothed again (default 30 ms).

Ground height is one number. It comes from the lower envelope of the foot, toe, and heel heights, preferring samples that are also slow. The file stores a confidence. A spread-out lower envelope (stairs, obstacles, jumps) is reported as low confidence. There is no terrain mesh.

PROC has no heel bone. The analyzer places a virtual heel behind the source ankle, opposite the toe, at `heel_fraction` (default 0.22) of the source foot length. The analysis records `inferred_virtual`.

Contact is a confidence in `[0, 1]` from height above the estimated ground, horizontal speed, vertical speed, and a short temporal window, then a binary state with enter/exit hysteresis. Single support, double support, and flight are all valid. A run is not forced onto the ground.

### Events, cycles, phase

Events are `initial_contact` and `toe_off`, with time, frame, side, and confidence. Timing is refined inside a short window around the threshold crossing. A stride is one side's initial contact to the next initial contact on that side. A cycle is accepted or rejected with a reason (duration, missing event, standing, insufficient travel, ambiguous contacts, low confidence, jump interruption, turning). Turning cycles stay in the file and are left out of the straight-regime fits.

Accepted cycles are resampled to 101 phase samples. Phase 0 and 1 are successive initial contacts on the same side. Right cycles also get a canonical copy: roll is negated and the stance leg is swapped into the reference-side curves. The original side is kept.

Relationships are per regime (`walk`, `run`, `sprint`), from accepted straight cycles. Each fit stores the sample count, speed range, median, percentiles, 0.25 m/s bins, and a straight-line fit with `r²` when there are at least two speeds. Walk and run are not pooled. There is no machine learning.

Quality is stored as contact, timing, heading, continuity, and overall.

Manual edits set `manual_override` and keep the automatic time, type, acceptance, or contact state beside the edited value.

### Dataset index and batch tool

`proc_motion_analyze.exe` scans a directory, imports each clip, retargets it, and writes:

```text
analysis_output/
  index.json
  clips/<clip>.procanalysis.json
  tables/gait_cycles.csv
  models/walk_reference.json
  models/run_reference.json
  models/sprint_reference.json
  summary.txt
```

Example:

```text
proc_motion_analyze.exe --input datasets\lafan1 --profile lafan1 --output analysis_output\lafan1 --theme Walk
```

One bad file is recorded and the scan continues. LAFAN names such as `walk1_subject2` and `fallAndGetUp1_subject3` fill theme, take, and subject. A dance clip is not turned into walk cycles just because it imported. `datasets/`, `local_assets/`, `converted/`, `imports/`, and `analysis_output/` stay untracked. Repository tests build `synthetic_walk`, `synthetic_run`, and `synthetic_non_gait` in `tests/synthetic_gait.h`.

The lab's ANALYSIS section runs the same core on the loaded source clip, plots the main signals, draws event marks on the timeline, and can step an event, change its type, or accept or reject a cycle.

### Known limitations

- Ground is one height. Uneven terrain is a low-confidence flag, not a surface.
- Relationships are for approximately straight locomotion. Turning is rejected from those fits and kept in the cycle list.
- The heel is not a tracked joint.
- Smoothing can move a contact edge by about one or two samples. On the synthetic walk that offset is 40 ms.
- Filename theme does not override the contact evidence.
- Phase 3 does not generate locomotion, lock the feet, or build a controller.
