#pragma once

#include <array>
#include <optional>
#include <string_view>

namespace proc {

inline constexpr const char* kSkeletonId = "proc_humanoid_v0";
inline constexpr int kJointCount = 22;
inline constexpr int kBoneCount = 21;
inline constexpr int kRootJoint = 0;

inline constexpr std::array<const char*, kJointCount> kJointNames = {
    "legrootj", "spine1j", "spine2j", "armrootj", "neckj", "headj",
    "lleguppj", "lleglowj", "lfootj",  "ltoesj",   "rleguppj", "rleglowj",
    "rfootj",   "rtoesj",  "larmuppj", "larmlowj", "lhandj",   "lfingj",
    "rarmuppj", "rarmlowj", "rhandj",  "rfingj"};

struct CanonicalBone {
  const char* name;
  int parent;
  int child;
};

inline constexpr std::array<CanonicalBone, kBoneCount> kCanonicalBones = {{
    {"spine1b", 0, 1},   {"spine2b", 1, 2},   {"spine3b", 2, 3},
    {"neckb", 3, 4},     {"headb", 4, 5},     {"llegrootb", 0, 6},
    {"lleguppb", 6, 7},  {"lleglowb", 7, 8},  {"lfootb", 8, 9},
    {"rlegrootb", 0, 10}, {"rleguppb", 10, 11}, {"rleglowb", 11, 12},
    {"rfootb", 12, 13},  {"larmrootb", 3, 14}, {"larmuppb", 14, 15},
    {"larmlowb", 15, 16}, {"lhandb", 16, 17}, {"rarmrootb", 3, 18},
    {"rarmuppb", 18, 19}, {"rarmlowb", 19, 20}, {"rhandb", 20, 21},
}};

std::optional<int> findJoint(std::string_view name);
std::optional<int> findBone(std::string_view name);
bool jointIsEndpoint(int index);

}  // namespace proc
