# Procedural Motion

Experimental fully procedural 3D humanoid motion. The long-term aim is walking, running, crouching, crawling, jumping, terrain adaptation, balance recovery, hit reactions, stumbling, and falling without authored animation clips.

**This project does not play animation.** The Y-Bot FBX is only a source of skeleton hierarchy and rest-pose proportions. No clips, keyframes, mocap, or pose playback are loaded into the simulator.

## Phase 5 — Balance foundation

Phase 5 does not retune the walk. Phase 4's load transfer worked, and the body still fell, because the quiet stand was a bad equilibrium. The feet settled about 13 cm behind the pelvis. One hip then had to hold about `75 kg * 9.81 * 0.13 m`, near 95 Nm, against a 100 Nm flexion limit. Two hips could share that moment. One hip could not, so unloading a foot folded the stance hip. The imported Y-Bot rest pose does not have this offset: the pelvis is at y = 0 and each foot is at about y = 0.026, roughly 2.6 cm, not 13 cm. The offset is produced by the controller.

The standing reference is rebuilt from the skeleton instead of `hipTarget = -0.06` and `ankleTarget = -(hip + knee)`. `solveLeg` places each ankle under its hip with a 10 degree knee bend. On this skeleton that is hip flexion -0.070 rad, knee -0.175 rad, ankle pitch 0.244 rad, hip-to-ankle span 0.824 m. Nothing is played back. Posture PD is only a regularizer. `BalanceController` owns the support objective: capture point of the whole-body COM relative to `SupportState`, ankle torque clamped to `foot load * 0.09 m` (the sole half-length), and a hip assist clamped to 10 Nm so the hip law itself cannot live on the 100 Nm limit. The hip flexion limit is unchanged.

`mj_jacBody` on the stance foot is wired, and any generalized force on the six free-root DOFs is discarded. That virtual force is currently held at zero. Mapping it through the hip folds the floating pelvis: the same positive hip torque that stops a fast flexion also throws the pelvis backward off the soles. The capture-point ankle law is what actually commands sagittal torque.

The 10 second stand does not pass. With the geometric reference, at 0.5 s the pelvis is at 0.945 m, up axis 0.998, and the pelvis/foot sagittal offset is already 0.161 m, with 16 Nm at the hip. By 2.5 s the offset is 0.384 m and the body is still upright. By 3.5 s it is on the ground (pelvis z 0.073, up axis 0.131). The 6–10 s window is that fallen pose: mean offset 0.887 m, mean hip torque 15 Nm, ankle torque sitting on the 70 Nm clamp, upright ratio 0, both feet 0/480. Root generalized force stayed 0. A bind-pose experiment reached an offset near 0 at about 2 s with the pelvis still at 0.96 m, then fell backward; that live hip-target follow is not kept, because it is the same settled-angle trap as Phase 4 and it still fell.

Weight shift, single-leg stance, the single-support push, and the one-step check do not run. The self-test stops at the stand. Phase 5 is not complete.

## Phase 4 — Support transfer

Phase 4 does not play a walk cycle. Support is computed from the planted soles, not from the pelvis. `SupportState` records each foot, the support center, the sole bounds, and the left/right load fraction. Locomotion and recovery both use it. The gait states are Standing, Transfer, LeftSwing, RightSwing, Landing, Stopping, and Fallen.

A step is not supposed to start because a timer expired. Transfer moves the balance target toward the stance sole and waits until that foot is Planted, its load fraction is above about 0.58, and the swing foot is under about 0.45. A 0.55 s timeout can still lift if the fraction is above 0.52 and the pelvis is upright. After the new sole is down, Landing waits until that foot is carrying load before the other leg may swing. Single support uses the stance sole as the balance target. Sagittal capture gain is lower than lateral gain so a forward fall is allowed and a sideways fall is not. Swing hip torque is capped near 18 Nm so the swing cannot pitch the pelvis by itself.

That transfer does what it was asked to do. From a quiet stand, a 0.4 m/s forward request starts in Transfer with the load near 49/51 and the pelvis at 0.94 m, up axis 0.97. At 0.5 s the stance foot carries about 98 percent, the pelvis is still at 0.90 m, and the up axis is 0.84. The step is counted by 1.0 s, but the up axis is already 0.65 and the body then falls. Earlier in the same setup the stance hip ran from about -0.3 rad to past -0.7 rad during that swing. The one-step regression fails there. The 0.4 m/s and 0.8 m/s walks are not reached.

The measured reason is the quiet-stand posture, not the missing swing. In that stand the feet sit about 13 cm behind the pelvis. Ground reaction through one sole is then a moment of about `75 kg * 9.81 * 0.13 m`, near 95 Nm, at the hip. The hip flexion limit is 100 Nm, and the quiet stand is already operating against that clamp: raising the limit to 120 Nm makes the 10 s stand fall. Two hips can share the moment. One hip cannot, once the other foot has actually unloaded. Crouching or straightening the hips during double support also drops the stand, because those targets fight the ankle equilibrium the 10 s test depends on. The failure is reported by the self-test. It is not a hidden skip.

Load thresholds used for the lift are stance fraction 0.58 and swing fraction 0.45, with a 0.55 s fallback at 0.52. Landing accepts the new foot at fraction 0.40 after 0.16 s. None of those gates are the reason the one-step falls. The stance hip is already past its margin when the fraction first hits the gate.

## Phase 3 — Landing handoff and continuous walking

Phase 3 does not play a walk cycle. There is still no clip, keyframe, or root motion. A desired horizontal velocity is turned into foot targets, a procedural swing, geometric IK, and limited hinge torques. Contacts stay physical.

### What was shaking

After a Phase 2 landing the swing leg kept chasing one world-space ankle position. The sole was already on the floor, so that target fought friction and the pose spring then froze the deep swing crouch. The handoff now fades. While the foot is in the air the swing command owns the leg. Once contact has been down long enough to count as planted, the swing target becomes the joint angles at that frame, and its weight goes from 1 to 0 in about 0.08 s. Standing balance takes the leg back. The achieved foot position is not held by a world-space spring.

Foot contact is no longer one frame of force. Each foot is Air, Touching, Planted, or Releasing, with a short filter and different enter and exit loads.

### Commands

Standing and the step planners write a `ControlFrame`. Each hinge command has a target, gains, a torque limit, an optional balance feedforward, and a swing weight. `JointServo` is the only stage that turns that into `qfrc_applied`. The free root is still never commanded. Timers use the physics timestep from the model, not a hardcoded 1/120.

Sole length, width, and ankle height live in `PhysicalMetrics` and are shared by the body builder and the planners. Thigh and shin lengths are measured from the generated skeleton.

### Walking math

With the pelvis height `h` and a desired horizontal velocity:

```text
omega = sqrt(g / h)
capture = COM + COMVelocity / omega
velocityError = desiredVelocity - COMVelocity

target = capture
       + desiredVelocity * 0.28
       + velocityError * 0.10
       + lateralStanceOffset
```

The target is then pulled back to the live leg reach. A minimum lead along the desired direction keeps the step from landing under the foot that is already there. Step time is

```text
duration = clamp(0.62 - 0.12 * speed, 0.42, 0.70)
```

so a faster request is a shorter step, inside about 0.42 to 0.70 s. The swing curve is the same smoothstep used in Phase 2, with a little extra clearance as speed rises. Nothing is stored.

While a nonzero velocity is requested, the standing support center is shifted a few centimeters along that velocity so the body is allowed to fall toward the next foot instead of being pinned between both soles. Arms leave the T-pose by a shoulder-abduction target that eases in when a step starts. A small opposite-phase shoulder flex is added during the swing. Desired velocity is capped at 1.4 m/s. There is no run and no yaw turn.

Recovery stepping still exists with Walking off. It may take up to three capture steps if the first one leaves the capture point outside. An 80 N forward push lifts a foot about 18 cm, moves it about 9 cm toward a 16 cm landing, and plants while the pelvis is still about 0.80 m high. The body still falls afterward. A quiet stand with recovery on does not creep: about 2 cm of drift in 4 s, foot speed near zero. An extreme push still falls.

Continuous walking is wired through the same swing and IK path. In the self-test a 0.8 m/s forward request produces repeated foot lifts, then the pelvis drops. It does not yet hold a gait. That failure is a real test failure, not a hidden one.

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

A quiet stand with Recovery Step on does not step. With Recovery Step off, 220 N for 0.20 s on the chest still knocks the stand over. The old 40 N toe-brace knockdown does not. The Phase 3 section is the current push behavior.

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
| W A S D | Desired walk velocity, up to 1.4 m/s, in the character's horizontal frame |
| V | Walking on / off |
| T | Active standing on / off |
| Y | Recovery stepping on / off (used when Walking is off) |
| P | Push the chest forward |
| Home | Reset the camera |

The panel has the same switches, plus Forward / Back / Left / Right push buttons and a magnitude slider (20 to 250 N, 0.12 s). The button only sets the Cartesian force. The step planner never reads which button was pressed. The panel shows the locomotion state, desired and actual horizontal velocity, swing foot, step phase, distance, duration, capture error, both foot contact states, and peak torque. Floor markers: COM sphere, gray support center, a small cross at the capture point, a ring at the landing, a thin swing curve, and two short arrows for desired velocity and COM velocity.

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

The self-test runs the Phase 0 passive fall first, then the active stand with recovery off. The stand must keep the pelvis upright for the last 4 seconds of a 10 second run, with both feet down and the center of mass near the support. A 220 N chest push for 0.20 s must still knock that stand over. Recovery is then turned on: three quiet seconds must not request a step, and the ankle pitch must stay off its range stop. An 80 N forward push must lift one foot, move it toward the computed landing, and plant while the pelvis is still above 0.65 m. A 140 N lateral push must lift a foot and plant while the pelvis is still that high. A 400 N push must still end in a fall. A quiet recovery stand must not creep. Walking checks then request 0.8 m/s forward and require repeated physical steps and real forward travel. That walk check currently fails because the body falls. Root generalized force stays zero, and every hinge torque stays inside its limit.

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
  src/control/               standing, recovery, locomotion, contact, command frame, servo, leg IK
  src/tests/                 headless regressions
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

- The 10 second quiet stand does not currently pass. The geometric reference is hip -0.070 rad, knee -0.175 rad, ankle 0.244 rad, but the pelvis still runs forward of the feet and the body is down by about 3.5 s. See Phase 5. The sole is only 18 cm long, so once the offset is past about 9 cm the ankles cannot hold the moment.
- A recovery step lifts a foot, moves it toward the capture-point target, and plants on a real contact. For an 80 N forward push the pelvis is still about 0.80 m high at that contact, then the stand after landing falls. Up to three capture steps are allowed. They do not yet save that push.
- Single support and the one-step walk are not reached. The self-test fails on the quiet stand before those checks. Phase 4's one-step failure is still the underlying problem: one hip cannot hold a pelvis that is not over the stance sole.
- The leg IK matches knee flexion well and needs a measured hip bias because the crouched hip angle is not the geometric thigh-from-vertical angle. Swing tracking is torque PD, so the foot arrives late and short of a far target.
- Joint limits are simple hinges, not anatomical joint surfaces. Ankle pitch is only ±40 degrees, which limits how far the stance shin can tilt while the sole stays flat.
- Clavicles are not rigid bodies.
- Masses, radii, and ranges are approximate.
- The floor is a flat infinite plane. There is no terrain raycast.
- Some nearby body pairs do not collide, so a collapse stays stable.
- Markers are diagnostic only.
- Windows is the supported build. The UI uses Arial when `C:\Windows\Fonts\arial.ttf` is present.

## Next phase

Do not start Phase 6, and do not retune step timing. The geometric stand is in place and the 10 second test fails on it. The next change has to keep the pelvis over the soles without a hip torque that either folds the torso or throws it backward. Until that two-foot stand holds with the hip well under 100 Nm, a weight shift and a one-leg stand will not be meaningful, and a walk will not be either.
