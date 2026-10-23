# Y-Bot to canonical skeleton

The importer keeps rest-pose joint positions only. Animation curves are not stored and the mesh is not rendered.

Scale to meters: 0.01. Up conversion mode: 0.

| Canonical | Parent | Source bone | Position (m) |
| --- | --- | --- | --- |
| pelvis | - | mixamorig:Hips | 0.0000 0.0000 0.8930 |
| spine_lower | pelvis | mixamorig:Spine | -0.0000 0.0123 0.9922 |
| spine_upper | spine_lower | mixamorig:Spine1 | -0.0000 0.0265 1.1087 |
| chest | spine_upper | mixamorig:Spine2 | -0.0000 0.0428 1.2423 |
| neck | chest | mixamorig:Neck | -0.0000 0.0348 1.3926 |
| head | neck | mixamorig:Head | -0.0000 0.0034 1.4958 |
| left_shoulder | chest | mixamorig:LeftShoulder | 0.0610 0.0357 1.3334 |
| left_upper_arm | left_shoulder | mixamorig:LeftArm | 0.1876 0.0617 1.3307 |
| left_lower_arm | left_upper_arm | mixamorig:LeftForeArm | 0.4616 0.0617 1.3308 |
| left_hand | left_lower_arm | mixamorig:LeftHand | 0.7378 0.0617 1.3308 |
| right_shoulder | chest | mixamorig:RightShoulder | -0.0611 0.0357 1.3334 |
| right_upper_arm | right_shoulder | mixamorig:RightArm | -0.1876 0.0617 1.3307 |
| right_lower_arm | right_upper_arm | mixamorig:RightForeArm | -0.4617 0.0617 1.3307 |
| right_hand | right_lower_arm | mixamorig:RightHand | -0.7378 0.0617 1.3307 |
| left_upper_leg | pelvis | mixamorig:LeftUpLeg | 0.0912 0.0006 0.8264 |
| left_lower_leg | left_upper_leg | mixamorig:LeftLeg | 0.0937 0.0057 0.4205 |
| left_foot | left_lower_leg | mixamorig:LeftFoot | 0.0912 0.0263 0.0000 |
| right_upper_leg | pelvis | mixamorig:RightUpLeg | -0.0912 0.0006 0.8264 |
| right_lower_leg | right_upper_leg | mixamorig:RightLeg | -0.0937 0.0057 0.4205 |
| right_foot | right_lower_leg | mixamorig:RightFoot | -0.0912 0.0263 0.0000 |

## Auxiliary points

These are not simulated joints. They size the skull, palms, and feet.

- `head_top` from `mixamorig:HeadTop_End` at -0.0001 -0.0630 1.6806 (head length)
- `left_hand_end` from `mixamorig:LeftHandMiddle1` at 0.8478 0.0617 1.3308 (palm length)
- `right_hand_end` from `mixamorig:RightHandMiddle1` at -0.8478 0.0617 1.3307 (palm length)
- `left_toe_end` from `mixamorig:LeftToe_End` at 0.0936 -0.2133 -0.0738 (foot length)
- `right_toe_end` from `mixamorig:RightToe_End` at -0.0936 -0.2133 -0.0738 (foot length)

## Not separate rigid bodies

- `left_shoulder` / `right_shoulder` are clavicles. The physical shoulder joint is at the upper arm.
- Fingers, facial bones, IK targets, and unused end sites are omitted.

## Source bones under the hips that were not mapped

- mixamorig:LeftHandThumb1
- mixamorig:LeftHandThumb2
- mixamorig:LeftHandThumb3
- mixamorig:LeftHandThumb4
- mixamorig:LeftHandIndex1
- mixamorig:LeftHandIndex2
- mixamorig:LeftHandIndex3
- mixamorig:LeftHandIndex4
- mixamorig:LeftHandMiddle2
- mixamorig:LeftHandMiddle3
- mixamorig:LeftHandMiddle4
- mixamorig:LeftHandRing1
- mixamorig:LeftHandRing2
- mixamorig:LeftHandRing3
- mixamorig:LeftHandRing4
- mixamorig:LeftHandPinky1
- mixamorig:LeftHandPinky2
- mixamorig:LeftHandPinky3
- mixamorig:LeftHandPinky4
- mixamorig:RightHandThumb1
- mixamorig:RightHandThumb2
- mixamorig:RightHandThumb3
- mixamorig:RightHandThumb4
- mixamorig:RightHandIndex1
- mixamorig:RightHandIndex2
- mixamorig:RightHandIndex3
- mixamorig:RightHandIndex4
- mixamorig:RightHandMiddle2
- mixamorig:RightHandMiddle3
- mixamorig:RightHandMiddle4
- mixamorig:RightHandRing1
- mixamorig:RightHandRing2
- mixamorig:RightHandRing3
- mixamorig:RightHandRing4
- mixamorig:RightHandPinky1
- mixamorig:RightHandPinky2
- mixamorig:RightHandPinky3
- mixamorig:RightHandPinky4
- mixamorig:LeftToeBase
- mixamorig:RightToeBase

