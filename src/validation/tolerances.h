#pragma once

namespace proc {

// Authored files are checked loosely enough to accept decimal JSON.
// The live pose check is stricter: motion is not allowed to stretch a bone.
inline constexpr double kAuthoringLengthToleranceM = 1e-4;
inline constexpr double kAuthoringDirectionTolerance = 1e-4;
inline constexpr double kAuthoringPositionToleranceM = 1e-4;
inline constexpr double kRuntimeBoneToleranceM = 1e-5;

}  // namespace proc
