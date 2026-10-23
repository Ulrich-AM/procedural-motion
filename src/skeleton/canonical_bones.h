#pragma once

namespace pm {

// Source keys are Mixamo bone names with the mixamorig prefix removed.
// Shoulders are clavicle joints. Upper arms start at the glenohumeral joint.
struct CanonicalBone {
  const char* name;
  const char* parent;
  const char* sourceKey;
};

inline constexpr CanonicalBone kCanonicalBones[] = {
    {"pelvis", "", "Hips"},
    {"spine_lower", "pelvis", "Spine"},
    {"spine_upper", "spine_lower", "Spine1"},
    {"chest", "spine_upper", "Spine2"},
    {"neck", "chest", "Neck"},
    {"head", "neck", "Head"},
    {"left_shoulder", "chest", "LeftShoulder"},
    {"left_upper_arm", "left_shoulder", "LeftArm"},
    {"left_lower_arm", "left_upper_arm", "LeftForeArm"},
    {"left_hand", "left_lower_arm", "LeftHand"},
    {"right_shoulder", "chest", "RightShoulder"},
    {"right_upper_arm", "right_shoulder", "RightArm"},
    {"right_lower_arm", "right_upper_arm", "RightForeArm"},
    {"right_hand", "right_lower_arm", "RightHand"},
    {"left_upper_leg", "pelvis", "LeftUpLeg"},
    {"left_lower_leg", "left_upper_leg", "LeftLeg"},
    {"left_foot", "left_lower_leg", "LeftFoot"},
    {"right_upper_leg", "pelvis", "RightUpLeg"},
    {"right_lower_leg", "right_upper_leg", "RightLeg"},
    {"right_foot", "right_lower_leg", "RightFoot"},
};

inline constexpr int kCanonicalBoneCount =
    static_cast<int>(sizeof(kCanonicalBones) / sizeof(kCanonicalBones[0]));

}  // namespace pm
