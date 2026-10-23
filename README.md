# Procedural Motion

Experimental fully procedural 3D humanoid motion. The long-term aim is walking, running, crouching, crawling, jumping, terrain adaptation, balance recovery, hit reactions, stumbling, and falling without authored animation clips.

**This project does not play animation.** The Y-Bot FBX is only a source of skeleton hierarchy and rest-pose proportions. No clips, keyframes, mocap, or pose playback are loaded into the simulator.

## Phase 2 — One recovery step

Phase 2 keeps the Phase 0 ragdoll and the Phase 1 stand. Recovery stepping is a separate switch.

```text
Active Control off                         passive ragdoll
Active Control on, Recovery Step off       Phase 1 stand only
Active Control on, Recovery Step on        stand, plus at most one recovery step
```

The pelvis stays a free body. The step is not a clip. Each frame the planner reads `BodyState`, picks a landing from the capture point, builds a swing curve from the start foot, the landing, and a phase in `[0, 1]`, and asks a two-bone IK solver for hip, knee, and ankle angles. Those angles become torque targets. Nothing writes a joint position, welds a foot, or teleports the root.

### Flat sole

The physical foot is a box instead of a capsule to the toe. Its length is inferred from the ankle and the toe point: 9 cm ahead of the ankle and 9 cm behind it, 8 cm wide, 2.2 cm thick, with the sole bottom 7.4 cm below the ankle so it clears the shin capsule. Foot mass stays 1.15 kg. The drawn toe capsule is unchanged. Quiet standing now holds the ankles near 0.30 rad, inside the ±0.70 rad range, instead of sitting on the dorsiflexion stop.

### Capture point

In the character's horizontal frame, with `h` the pelvis height:

```text
omega = sqrt(g / h)
capture = horizontalCOM + horizontalCOMVelocity / omega
```

A step is requested only when that point stays more than 2.8 cm outside the support polygon of the feet that are actually loaded, for about 60 ms, and the pelvis is still upright. Inside the polygon for 0.2 s rearms the planner. A second step does not start by itself. If the pelvis is already low, the planner resets instead of firing a long step.

### Placement and swing

```text
desiredFoot = capturePoint + stanceOffset
```

The offset keeps a minimum stance width and refuses a crossing step. The result is clamped to the horizontal reach of the measured thigh and shin, and the ankle height is the sole drop so the target sits on the flat floor.

The swing curve is not stored:

```text
s = phase^2 * (3 - 2 * phase)
horizontal = start + (target - start) * s
height = lerp(startZ, soleHeight, s) + clearance * 4 * s * (1 - s)
```

Clearance is 8 cm. Duration is about 0.30 to 0.48 s depending on distance. States are Idle, Prepare, Swing, Plant, Settle, then Idle. Plant waits for a real foot load. It does not treat phase 1 as contact. Prepare and the stance leg stay on the standing torques. The swing foot is kept roughly level by the ankle target, not by a constraint.

`solveLeg` is a geometric two-bone IK. It returns angles only. At the start of Prepare it measures the gap between that solution and the current hip angles and keeps the difference as a bias, so a "hold the foot" solution does not straighten the crouched hip. After the foot has been down for the settle interval, those landed angles become the new standing references so the pose spring does not yank the hip back to the pre-step target.

### What a push does today

A quiet stand with Recovery Step on does not step. A forward chest push of 110 N for 0.14 s, and a lateral push of 120 N for 0.14 s, each lift one foot about 10 cm and move it toward the computed landing. The body still falls afterward. An extreme push (400 N for 0.30 s) falls without a useful save. With Recovery Step off, 220 N for 0.20 s on the chest still knocks the stand over. The old 40 N toe-brace knockdown does not.

## Phase 1 — Active standing

Phase 1 keeps the Phase 0 ragdoll and adds a standing controller that acts only through limited hinge torques. The pelvis stays a free body. Nothing writes its position, orientation, or velocity during a step.

Active Control starts off. With it off, the humanoid is the Phase 0 ragdoll and collapses under gravity. With it on, a torque-limited PD holds a rest pose with a small knee bend, and an ankle/hip term nudges the center of mass back toward the feet.

There is no walk, step, gait, crouch, jump, hit reaction, reinforcement learning, mocap, or authored stand pose. If a push is large enough, the character falls. That fall is the intended failure.

## Phase 0

Phase 0 is the physical lab this controller sits on:

- a Windows CMake project
- a flat test floor with a square grid
- a canonical humanoid extracted from the Y-Bot skeleton
- a MuJoCo ragdoll with a free pelvis
- primitive joint spheres and bone capsules
- pause, single-step, reset, and a small debug panel
- fixed 120 Hz physics, Y-Bot proportions, joint limits, contacts, and a center-of-mass marker

With gravity on and Active Control off, the body falls and collapses. Nothing holds it upright.

## How standing works

`StandingController` reads `BodyState` and writes hinge torques. It does not touch rendering, and the viewer does not know the control law.

Each hinge uses

```text
torque = Kp * (target - angle) - Kd * angularVelocity
```

clamped to a per-joint limit. The target is the reset angle, except the knees, which bend about 10 degrees toward the flexion side of their joint range. The arms stay in the imported T-pose and use much smaller gains than the legs.

When a foot is on the floor, a capture-point term adds ankle and hip torque from the horizontal offset between the support center and the whole-body center of mass, plus the horizontal center-of-mass velocity. The support center is the midpoint of the ankles of the feet that are loaded. Chasing the contact point on a flat sole walks the body off the foot. After the landing transient, the ankle-pitch reference eases onto the angle the sole actually settled at, so the pose spring does not cancel the capture torque. Spine and neck hinges add a smaller upright term. No torque is applied to the free root. While a recovery step is in swing, that leg keeps its pose torque and skips the two-foot balance term.

Gains and limits live in `StandingController::tuneFor`:

| Joints | Kp (Nm/rad) | Kd (Nm·s/rad) | Limit (Nm) |
| --- | --- | --- | --- |
| Ankle pitch and roll | 80 | 8 | 70 |
| Hip abduction | 650 | 50 | 120 |
| Hip flexion | 300 | 24 | 100 |
| Hip twist | 80 | 8 | 30 |
| Knee | 420 | 36 | 110 |
| Spine | 80 | 8 | 40 |
| Neck | 20 | 2 | 12 |
| Shoulder | 55 | 6 | 28 |
| Elbow | 24 | 2.2 | 14 |
| Wrist | 6 | 0.6 | 5 |

Hip and knee hinges also have extra viscous damping in the MuJoCo model (8 and 6) so the landing does not whip those joints. The passive fall still reaches the floor.

The controller clears its torques every step and writes them into MuJoCo's generalized applied force. There are no motor actuators (`nu` stays 0). A chest push is a separate Cartesian force on the chest, not a joint torque and not a root teleport.

## Controls

| Input | Action |
| --- | --- |
| Left mouse | Orbit |
| Right mouse | Pan |
| Wheel | Zoom |
| R | Reset the humanoid and clear velocities |
| Space | Pause / resume |
| N | One physics step while paused |
| G | Gravity on / off |
| C | Collision shape debug view |
| J | Joint axis debug view |
| A | Active standing on / off |
| S | Recovery stepping on / off |
| P | Push the chest forward |
| Home | Reset the camera |

The panel has the same switches, plus Forward / Back / Left / Right push buttons and a magnitude slider (20 to 250 N, 0.12 s). The button only sets the Cartesian force. The step planner never reads which button was pressed. The panel shows balance, step state, swing foot, capture error, step distance, and step phase. Floor markers: COM sphere, gray support center, a small cross at the capture point, a ring at the landing, and a thin line for the swing curve.

The camera orbits a fixed point near the pelvis and chest. It does not follow the ragdoll.

## Dependencies

Installed on the machine:

- Windows
- CMake 3.24 or newer
- Visual Studio 2022 or newer, with the Desktop development with C++ workload

Fetched automatically the first time you configure (network required):

| Library | Version | Used for |
| --- | --- | --- |
| MuJoCo | 3.17.0 | Rigid bodies, joints, contacts |
| Assimp | 5.4.3 | FBX skeleton import tool only |
| GLFW | 3.4 | Window |
| Dear ImGui | 1.91.8 | Small debug panel |
| nlohmann/json | 3.11.3 | `skeleton.json` and `lab.json` |

The running lab does not link Assimp and does not read the FBX.

## Build and run

From the repository root, in a normal command prompt:

```bat
build_windows.bat
```

Then:

```bat
build\Release\procedural_motion.exe
```

If CMake picked a single-config generator, the executable is `build\procedural_motion.exe` instead.

Physics check without a window:

```bat
build\Release\procedural_motion.exe --self-test
```

The self-test runs the Phase 0 passive fall first, then the active stand with recovery off. The stand must keep the pelvis upright for the last 4 seconds of a 10 second run, with both feet down and the center of mass near the support. A 220 N chest push for 0.20 s must still knock that stand over. Recovery is then turned on: three quiet seconds must not request a step, and the ankle pitch must stay off its range stop. A 110 N forward push and a 120 N lateral push must each lift one foot and move it toward the computed landing, using physical contact rather than a phase threshold. A 400 N push must still end in a fall. Root generalized force stays zero, and every hinge torque stays inside its limit.

Or, after a build:

```bat
ctest --test-dir build -C Release --output-on-failure
```

The first configure downloads MuJoCo and the source of the other libraries. The first build also compiles Assimp, which takes several minutes. Later builds are incremental.

`assets/source/ybot.fbx` is required. The build runs `fbx_skeleton_import` and writes:

- `assets/generated/skeleton.json` — rest pose the lab loads
- `assets/generated/hierarchy.txt` — full source node tree
- `assets/generated/skeleton_mapping.md` — source bone to canonical bone
- `humanoid.xml` beside the loaded JSON, written when the lab starts

Developer flags:

```bat
procedural_motion.exe --self-test
procedural_motion.exe --screenshot out\frame
procedural_motion.exe --run-seconds 5
```

`--screenshot` writes `out\frame_rest.bmp` and `out\frame_settled.bmp`.

## Repository

```text
procedural-motion/
  CMakeLists.txt
  build_windows.bat
  README.md
  assets/source/ybot.fbx
  assets/generated/          skeleton.json and mapping, produced by the build
  config/lab.json            timestep, gravity, spawn clearance
  src/main.cpp
  src/app/                   window, input, fixed timestep, debug UI
  src/control/               standing controller, capture-point step planner, leg IK
  src/skeleton/              canonical bone list and JSON loader
  src/physics/               MuJoCo bodies, joints, state queries, hinge torques
  src/render/                orbit camera, primitive view, floor grid
  tools/fbx_skeleton_import/ Assimp converter, not linked into the lab
```

Skeleton description, physical bodies, the standing controller, rendering, and app control stay separate.

## FBX import

```text
assets/source/ybot.fbx
    -> tools/fbx_skeleton_import
    -> assets/generated/skeleton.json
    -> MuJoCo model built at startup
```

Assimp reads node names, parents, and rest transforms. The tool prints the source hierarchy while it runs. It records animation clip names only to mark them ignored. It does not write curves, and it does not export the mesh.

This Y-Bot file's bind pose is a T-pose: the arms are straight out to the sides. That pose is kept. It is not an animation, and nothing plays a clip to change it.

Positions are converted to a Z-up, meters, right-handed frame and placed so the lowest rest joint is at z = 0. The lab then lifts the free pelvis so collision shapes start about 8 cm above the floor. That clearance is `spawn_clearance_meters` in `config/lab.json`.

Physics steps at a fixed `fixed_timestep_seconds` (default 1/120) with an accumulator. Rendering is not the physics clock.

## Canonical mapping

Mixamo names are matched with or without the `mixamorig:` prefix.

| Canonical | Source | Role |
| --- | --- | --- |
| pelvis | Hips | Free root body |
| spine_lower | Spine | Abdomen body |
| spine_upper | Spine1 | Chest body origin |
| chest | Spine2 | Visual joint on the chest |
| neck | Neck | Head body origin |
| head | Head | Skull |
| left_shoulder / right_shoulder | LeftShoulder / RightShoulder | Clavicle, visual only |
| left_upper_arm / right_upper_arm | LeftArm / RightArm | Upper arm body, shoulder joint |
| left_lower_arm / right_lower_arm | LeftForeArm / RightForeArm | Forearm, elbow hinge |
| left_hand / right_hand | LeftHand | Hand body |
| left_upper_leg / right_upper_leg | LeftUpLeg / RightUpLeg | Thigh, hip |
| left_lower_leg / right_lower_leg | LeftLeg / RightLeg | Shin, knee hinge |
| left_foot / right_foot | LeftFoot / RightFoot | Foot, ankle |

Fingers are not bones in the canonical skeleton. A finger knuckle, if present, is used only to set palm length. `HeadTop_End` sizes the head collision sphere. The drawn head is the same small joint sphere as the other bones. `LeftToe_End` or `LeftToeBase` sizes the foot. Those auxiliary points are not joints.

Physical bodies: pelvis, abdomen, chest, head, both upper arms, forearms, hands, thighs, shins, and feet (16). Adjacent bodies are connected by hinge joints. Multi-axis joints are three limited hinges. Knees and elbows are single hinges whose range blocks hyperextension and allows flexion. The pelvis free joint is not damped and is not kinematically driven.

Target mass is about 75 kg, distributed roughly like a human (torso heaviest, hands lightest). Collision shapes are capsules, a sphere for the head, and a box sole on each foot. Visual capsules are thinner than the colliders. Parent/child contacts are left to MuJoCo's default exclusion, and a few extra sibling pairs are excluded so the A-pose does not start interpenetrating.

The measured source names and positions are written to `assets/generated/skeleton_mapping.md` by the importer.

## Limitations

- Quiet standing on the flat sole is stable for the 10 second self-test, with ankles near 0.30 rad rather than the ±0.70 rad stop. It is still a narrow ankle strategy. The sole is only 18 cm long.
- A recovery step does lift a foot, move it toward the capture-point target, and wait for real contact. In the pushes tried here (110 N forward, 120 N to the side, both for about 0.14 s) the body still falls after that step. The lateral step plants while the pelvis is still high, then the settle loses it. One step is not yet a reliable save.
- The planner will not chain steps. There is no walk cycle.
- The leg IK matches knee flexion well and needs a measured hip bias because the crouched hip angle is not the geometric thigh-from-vertical angle. Swing tracking is torque PD, so the foot arrives late and short of a far target.
- Joint limits are simple hinges, not anatomical joint surfaces. Ankle pitch is only ±40 degrees, which limits how far the stance shin can tilt while the sole stays flat.
- Clavicles are not rigid bodies.
- Masses, radii, and ranges are approximate.
- The floor is a flat infinite plane. There is no terrain raycast.
- Some nearby body pairs do not collide, so a collapse stays stable.
- Markers are diagnostic only.
- Windows is the supported build. The UI uses Arial when `C:\Windows\Fonts\arial.ttf` is present.

## Next phase

Phase 3 should make this one step actually catch the center of mass, still without a gait cycle. The useful next work is a faster swing that reaches the landing while the pelvis is still high, and a settle that keeps the landed posture instead of falling after contact. Repeated stepping, turning, and rough terrain come after a single save is reliable.
