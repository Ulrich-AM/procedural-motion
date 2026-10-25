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

Internal units are meters. Quaternions are stored as `[x, y, z, w]`. The bind pose is a T-pose. Future importers convert their source coordinates into this convention before anything is saved as a PROC BODY or a `.procmo`.

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

Requirements: Windows, CMake, and Visual Studio with the C++ workload. The first configure downloads GLFW, GLM, Dear ImGui, and nlohmann/json.

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

On startup the lab loads `assets/bodies/neutral.procbody.json` when it can find `assets` by walking up from the working directory or from the executable.

## Controls

The window is a compact research tool: Arial, grayscale, sharp corners, a dark viewport, and a flat side panel.

- Left mouse drag in the viewport orbits. The wheel zooms.
- Front, Side, 3/4, and Reset camera.
- Load or reload a `.procbody.json` and a `.procmo`. Dropping a `.procmo` loads motion. Dropping other JSON loads a body.
- Transport: jump to start, previous source frame, play/pause, next source frame, jump to end. Space toggles play.
- Timeline scrubber, loop toggle, and speeds 0.25×, 0.5×, 1×, 2×.
- Joint limits: WARN or CLAMP.
- Optional root trajectory. Foot contacts draw as discs. A heel marker is drawn at the foot joint because this skeleton has no heel bone. A toe marker is drawn at the toe joint. White means contact, dark means explicitly open, and a missing contact is not drawn.
- A short peg on the head points along local `+Y`. It is a facing marker, not a bone.

The status line shows frame, time, max bone-length error, whether a joint limit was violated, and PASS or FAIL.

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

## Non-goals

Phase 1 does not implement, and does not add scaffolding for, MuJoCo, Box2D, rigid-body physics, balance or stepping controllers, SIMBICON, procedural walking, active ragdoll, IK locomotion, FBX, BVH, Mixamo, LAFAN1, AMASS, SMPL, video pose detection, MediaPipe, machine learning, mesh skinning, a Y-Bot mesh, or auto-rigging.

## Roadmap

Phase 2 is motion import and bone-preserving retargeting: Mixamo FBX, BVH, and LAFAN1. An importer converts the source into this coordinate system and writes local rotations onto `proc_humanoid_v0`. It does not copy source bone lengths onto the target body.

Later work, after that, can cover AMASS conversion, motion analysis, gait-cycle extraction, reference-derived procedural locomotion, different body proportions, physical compliance, a disturbance and reaction layer, and an active ragdoll. None of that is in this tree.
