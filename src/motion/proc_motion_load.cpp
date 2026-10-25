#include "motion/proc_motion.h"

#include "validation/motion_validator.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <initializer_list>
#include <sstream>

namespace proc {
namespace {

using json = nlohmann::json;

std::string quoteName(std::string_view text) { return "\"" + std::string(text) + "\""; }

void fail(Status& status, std::string_view label, const std::string& message) {
  status.error(std::string(label) + ": " + message);
}

void rejectUnknown(const json& object, std::initializer_list<const char*> allowed, std::string_view label,
                   const std::string& where, Status& status) {
  if (!object.is_object()) return;
  for (auto it = object.begin(); it != object.end(); ++it) {
    bool known = false;
    for (const char* key : allowed) {
      if (it.key() == key) known = true;
    }
    if (!known) {
      if (it.key() == "scale" || it.key() == "scales" || it.key() == "bone_lengths" || it.key() == "joint_positions" ||
          it.key() == "translation") {
        fail(status, label, where + " cannot carry animated scale or non-root joint translation (" + quoteName(it.key()) + ")");
      } else {
        fail(status, label, where + " has unknown field " + quoteName(it.key()));
      }
    }
  }
}

bool readVec3(const json& value, glm::dvec3& out, std::string_view label, const std::string& where, Status& status) {
  if (!value.is_array() || value.size() != 3 || !value[0].is_number() || !value[1].is_number() || !value[2].is_number()) {
    fail(status, label, where + " must be an array of 3 numbers");
    return false;
  }
  out = glm::dvec3(value[0].get<double>(), value[1].get<double>(), value[2].get<double>());
  if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z)) {
    fail(status, label, where + " must be finite");
    return false;
  }
  return true;
}

bool readQuat(const json& value, glm::dquat& out, std::string_view label, const std::string& where, Status& status) {
  if (!value.is_array() || value.size() != 4) {
    fail(status, label, where + " must be a quaternion [x, y, z, w]");
    return false;
  }
  double components[4];
  for (int i = 0; i < 4; ++i) {
    if (!value[i].is_number()) {
      fail(status, label, where + " must contain only numbers");
      return false;
    }
    components[i] = value[i].get<double>();
    if (!std::isfinite(components[i])) {
      fail(status, label, where + " must be finite");
      return false;
    }
  }
  const glm::dquat raw = quatFromXYZW(components[0], components[1], components[2], components[3]);
  const double length = quatLength(raw);
  if (length < 1e-6 || length > 1e3) {
    fail(status, label, where + " quaternion cannot be normalized");
    return false;
  }
  if (std::abs(length - 1.0) > 1e-3) {
    status.warn(std::string(label) + ": " + where + " quaternion was not unit length and was normalized");
  }
  out = quatNormalize(raw);
  return true;
}

void readContacts(const json& value, ContactSample& out, std::string_view label, int frame_index, Status& status) {
  if (!value.is_object()) {
    fail(status, label, "frame " + std::to_string(frame_index) + " contacts must be an object");
    return;
  }
  out.block_present = true;
  const char* names[] = {"left_heel", "left_toe", "right_heel", "right_toe"};
  for (auto it = value.begin(); it != value.end(); ++it) {
    bool known = false;
    for (const char* name : names) {
      if (it.key() == name) known = true;
    }
    if (!known) {
      fail(status, label, "frame " + std::to_string(frame_index) + " has unknown contact " + quoteName(it.key()));
      continue;
    }
    if (!it.value().is_boolean()) {
      fail(status, label, "frame " + std::to_string(frame_index) + " contact " + quoteName(it.key()) + " must be a boolean");
      continue;
    }
    const bool flag = it.value().get<bool>();
    if (it.key() == "left_heel") out.left_heel = flag;
    else if (it.key() == "left_toe") out.left_toe = flag;
    else if (it.key() == "right_heel") out.right_heel = flag;
    else out.right_toe = flag;
  }
}

}  // namespace

Status loadProcMotionFromString(std::string_view json_text, ProcMotion& out, std::string_view label) {
  Status status;
  json document;
  try {
    document = json::parse(json_text.begin(), json_text.end());
  } catch (const json::exception& error) {
    fail(status, label, std::string("JSON parse error: ") + error.what());
    return status;
  }
  if (!document.is_object()) {
    fail(status, label, "root value must be an object");
    return status;
  }
  rejectUnknown(document, {"format", "version", "skeleton", "source", "timing", "frames"}, label, "motion", status);
  if (!document.contains("format") || !document["format"].is_string() || document["format"] != "PROC_MOTION") {
    fail(status, label, "format must be \"PROC_MOTION\"");
  }
  if (!document.contains("version") || !document["version"].is_number() || document["version"].get<double>() != 1.0) {
    fail(status, label, "version must be 1");
  }

  ProcMotion motion;
  if (!document.contains("skeleton") || !document["skeleton"].is_string()) {
    fail(status, label, "skeleton must be a string");
  } else {
    motion.skeleton = document["skeleton"].get<std::string>();
    if (motion.skeleton != kSkeletonId) {
      fail(status, label, "skeleton is " + quoteName(motion.skeleton) + ", expected " + quoteName(kSkeletonId));
    }
  }

  if (!document.contains("source") || !document["source"].is_object()) {
    fail(status, label, "source must be an object");
  } else {
    const json& source = document["source"];
    rejectUnknown(source, {"type", "dataset", "clip"}, label, "source", status);
    if (!source.contains("type") || !source["type"].is_string()) fail(status, label, "source.type must be a string");
    else motion.source.type = source["type"].get<std::string>();
    if (!source.contains("clip") || !source["clip"].is_string()) fail(status, label, "source.clip must be a string");
    else motion.source.clip = source["clip"].get<std::string>();
    if (!source.contains("dataset") || source["dataset"].is_null()) {
      motion.source.dataset.reset();
    } else if (!source["dataset"].is_string()) {
      fail(status, label, "source.dataset must be a string or null");
    } else {
      motion.source.dataset = source["dataset"].get<std::string>();
    }
  }

  if (!document.contains("timing") || !document["timing"].is_object()) {
    fail(status, label, "timing must be an object");
  } else {
    const json& timing = document["timing"];
    rejectUnknown(timing, {"fps", "frame_count", "duration_s"}, label, "timing", status);
    if (!timing.contains("fps") || !timing["fps"].is_number()) fail(status, label, "timing.fps must be a number");
    else motion.fps = timing["fps"].get<double>();
    if (!timing.contains("frame_count") || !timing["frame_count"].is_number_integer()) {
      fail(status, label, "timing.frame_count must be an integer");
    } else {
      motion.frame_count = timing["frame_count"].get<int>();
    }
    if (!timing.contains("duration_s") || !timing["duration_s"].is_number()) {
      fail(status, label, "timing.duration_s must be a number");
    } else {
      motion.duration_s = timing["duration_s"].get<double>();
    }
  }

  if (!document.contains("frames") || !document["frames"].is_array()) {
    fail(status, label, "frames must be an array");
  } else {
    int frame_index = 0;
    for (const json& frame_json : document["frames"]) {
      if (!frame_json.is_object()) {
        fail(status, label, "frame " + std::to_string(frame_index) + " must be an object");
        ++frame_index;
        continue;
      }
      rejectUnknown(frame_json, {"time", "root", "joints", "contacts"}, label, "frame " + std::to_string(frame_index), status);
      Pose pose;
      if (!frame_json.contains("time") || !frame_json["time"].is_number()) {
        fail(status, label, "frame " + std::to_string(frame_index) + " is missing time");
      } else {
        pose.time = frame_json["time"].get<double>();
      }
      if (!frame_json.contains("root") || !frame_json["root"].is_object()) {
        fail(status, label, "frame " + std::to_string(frame_index) + " is missing root");
      } else {
        const json& root = frame_json["root"];
        rejectUnknown(root, {"translation", "rotation"}, label, "frame " + std::to_string(frame_index) + " root", status);
        if (!root.contains("translation")) {
          fail(status, label, "frame " + std::to_string(frame_index) + " root is missing translation");
        } else {
          readVec3(root["translation"], pose.root_translation, label,
                   "frame " + std::to_string(frame_index) + " root translation", status);
        }
        if (!root.contains("rotation")) {
          fail(status, label, "frame " + std::to_string(frame_index) + " root is missing rotation");
        } else {
          readQuat(root["rotation"], pose.root_rotation, label,
                   "frame " + std::to_string(frame_index) + " root rotation", status);
        }
      }
      if (frame_json.contains("joints")) {
        if (!frame_json["joints"].is_object()) {
          fail(status, label, "frame " + std::to_string(frame_index) + " joints must be an object");
        } else {
          for (auto it = frame_json["joints"].begin(); it != frame_json["joints"].end(); ++it) {
            const std::optional<int> joint = findJoint(it.key());
            if (!joint) {
              fail(status, label, "frame " + std::to_string(frame_index) + " has unknown joint " + quoteName(it.key()));
              continue;
            }
            if (*joint == kRootJoint) {
              fail(status, label, "frame " + std::to_string(frame_index) +
                                      " must not animate legrootj inside joints; use the root block");
              continue;
            }
            if (!it.value().is_array()) {
              fail(status, label, "frame " + std::to_string(frame_index) + " joint " + quoteName(it.key()) +
                                      " must be a quaternion; non-root joint translation is not allowed");
              continue;
            }
            readQuat(it.value(), pose.local_rotation[*joint], label,
                     "frame " + std::to_string(frame_index) + " joint " + quoteName(it.key()), status);
          }
        }
      }
      if (frame_json.contains("contacts")) {
        readContacts(frame_json["contacts"], pose.contacts, label, frame_index, status);
      }
      motion.frames.push_back(pose);
      ++frame_index;
    }
  }

  if (status.ok()) validateProcMotion(motion, status);
  if (!status.ok()) return status;
  out = std::move(motion);
  return status;
}

Status loadProcMotionFromFile(const std::filesystem::path& path, ProcMotion& out) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    Status status;
    status.error("could not open " + path.string());
    return status;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  std::string text = buffer.str();
  if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
      static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF) {
    text.erase(0, 3);
  }
  Status status = loadProcMotionFromString(text, out, path.filename().string());
  if (status.ok()) out.source_path = path.string();
  return status;
}

}  // namespace proc
