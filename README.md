# Procedural Motion

Experimental fully procedural 3D humanoid motion. The long-term aim is walking, running, crouching, crawling, jumping, terrain adaptation, balance recovery, hit reactions, stumbling, and falling without authored animation clips.

**This project does not play animation.** The Y-Bot FBX is only a source of skeleton hierarchy and rest-pose proportions. No clips, keyframes, mocap, or pose playback are loaded into the simulator.

## Phase 0

Phase 0 is a physical humanoid lab, not locomotion. It builds:

- a Windows CMake project
- a flat test floor with a square grid
- a canonical humanoid extracted from the Y-Bot skeleton
- a MuJoCo ragdoll with a free pelvis
- primitive joint spheres and bone capsules
- pause, single-step, reset, and a small debug panel

With gravity on and no controllers, the body falls and collapses. Nothing holds it upright.

Not in this phase: standing, walking, IK, balance, gait, hit reactions, reinforcement learning, or animation playback.

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
| Home | Reset the camera |

The camera orbits a fixed point near the pelvis and chest. It does not follow the ragdoll.

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
  src/skeleton/              canonical bone list and JSON loader
  src/physics/               MuJoCo bodies, joints, state queries
  src/render/                orbit camera, primitive view, floor grid
  tools/fbx_skeleton_import/ Assimp converter, not linked into the lab
```

The four pieces stay separate: skeleton description, physical bodies, rendering, and app control. Later controllers can sit beside them. They are not implemented yet.

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

Target mass is about 75 kg, distributed roughly like a human (torso heaviest, hands lightest). Collision shapes are capsules, plus a sphere for the head. Visual capsules are thinner than the colliders. Parent/child contacts are left to MuJoCo's default exclusion, and a few extra sibling pairs are excluded so the A-pose does not start interpenetrating.

The measured source names and positions are written to `assets/generated/skeleton_mapping.md` by the importer.

## Limitations

- No muscles, motors, or pose springs. The ragdoll only falls.
- Joint limits are simple hinges, not anatomical joint surfaces.
- Clavicles are not rigid bodies.
- Masses, radii, and ranges are approximate.
- Feet are capsules on a flat infinite plane.
- Some nearby body pairs do not collide, so the collapse stays stable.
- The center-of-mass marker is diagnostic. A vertical tick marks its ground projection. Nothing balances around it.
- Windows is the supported build. The UI uses Arial when `C:\Windows\Fonts\arial.ttf` is present.

## Next phase

Phase 1 should add a joint-torque interface and a standing experiment: hold a rest pose and recover small pushes using contact forces and the center of mass, still with no animation clips. Do not start gait until a passive fall and a controlled stand are both reliable.
