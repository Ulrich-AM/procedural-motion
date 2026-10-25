#include "body/skeleton.h"

namespace proc {
namespace {

constexpr bool nameIs(const char* text, const char* expected) {
  while (*text != '\0' && *text == *expected) {
    ++text;
    ++expected;
  }
  return *text == *expected;
}

static_assert(static_cast<int>(JointId::Count) == kJointCount);
static_assert(jointIndex(JointId::LegRoot) == kRootJoint);
static_assert(nameIs(kJointNames[jointIndex(JointId::LegRoot)], "legrootj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::Spine1)], "spine1j"));
static_assert(nameIs(kJointNames[jointIndex(JointId::Spine2)], "spine2j"));
static_assert(nameIs(kJointNames[jointIndex(JointId::ArmRoot)], "armrootj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::Neck)], "neckj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::Head)], "headj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftHip)], "lleguppj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftKnee)], "lleglowj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftFoot)], "lfootj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftToe)], "ltoesj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightHip)], "rleguppj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightKnee)], "rleglowj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightFoot)], "rfootj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightToe)], "rtoesj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftShoulder)], "larmuppj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftElbow)], "larmlowj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftHand)], "lhandj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::LeftFinger)], "lfingj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightShoulder)], "rarmuppj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightElbow)], "rarmlowj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightHand)], "rhandj"));
static_assert(nameIs(kJointNames[jointIndex(JointId::RightFinger)], "rfingj"));

}  // namespace

std::optional<int> findJoint(std::string_view name) {
  for (int index = 0; index < kJointCount; ++index) {
    if (name == kJointNames[index]) return index;
  }
  return std::nullopt;
}

std::optional<int> findBone(std::string_view name) {
  for (int index = 0; index < kBoneCount; ++index) {
    if (name == kCanonicalBones[index].name) return index;
  }
  return std::nullopt;
}

bool jointIsEndpoint(int index) {
  for (const CanonicalBone& bone : kCanonicalBones) {
    if (bone.parent == index) return false;
  }
  return true;
}

}  // namespace proc
