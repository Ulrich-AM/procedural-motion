#include "body/proc_body.h"

#include "validation/skeleton_validator.h"

#include <nlohmann/json.hpp>

#include <glm/geometric.hpp>

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

bool finite3(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool readVec3(const json& value, glm::dvec3& out, std::string_view label, const std::string& where, Status& status) {
  if (!value.is_array() || value.size() != 3 || !value[0].is_number() || !value[1].is_number() ||
      !value[2].is_number()) {
    fail(status, label, where + " must be an array of 3 numbers");
    return false;
  }
  out = glm::dvec3(value[0].get<double>(), value[1].get<double>(), value[2].get<double>());
  if (!finite3(out)) {
    fail(status, label, where + " must be finite");
    return false;
  }
  return true;
}

void rejectUnknown(const json& object, std::initializer_list<const char*> allowed, std::string_view label,
                   const std::string& where, Status& status) {
  if (!object.is_object()) return;
  for (auto it = object.begin(); it != object.end(); ++it) {
    bool known = false;
    for (const char* key : allowed) {
      if (it.key() == key) known = true;
    }
    if (!known) fail(status, label, where + " has unknown field " + quoteName(it.key()));
  }
}

bool allowedKey(std::string_view key, std::initializer_list<const char*> allowed) {
  for (const char* name : allowed) {
    if (key == name) return true;
  }
  return false;
}

void linkHierarchy(ProcBody& body) {
  for (JointGeom& joint : body.joints) {
    joint.parent = -1;
    joint.incoming_bone = -1;
    joint.children.clear();
  }
  for (int index = 0; index < kBoneCount; ++index) {
    const BoneGeom& bone = body.bones[index];
    if (bone.parent_joint < 0 || bone.child_joint < 0) continue;
    body.joints[bone.child_joint].parent = bone.parent_joint;
    body.joints[bone.child_joint].incoming_bone = index;
    body.joints[bone.parent_joint].children.push_back(bone.child_joint);
  }
  body.joints[kRootJoint].parent = -1;
}

}  // namespace

const char* jointTypeName(JointType type) {
  switch (type) {
    case JointType::Root:
      return "root";
    case JointType::Hinge:
      return "hinge";
    case JointType::Ball:
      return "ball";
    case JointType::Endpoint:
      return "endpoint";
  }
  return "unknown";
}

const double* Anthropometry::find(std::string_view key) const {
  for (const auto& measurement : measurements) {
    if (measurement.first == key) return &measurement.second;
  }
  return nullptr;
}

Status loadProcBodyFromString(std::string_view json_text, ProcBody& out, std::string_view label) {
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
  rejectUnknown(document, {"format", "version", "skeleton", "name", "anthropometry", "joints", "bones", "constraints"},
                label, "body", status);

  if (!document.contains("format") || !document["format"].is_string() || document["format"] != "PROC_BODY") {
    fail(status, label, "format must be \"PROC_BODY\"");
  }
  if (!document.contains("version") || !document["version"].is_number() || document["version"].get<double>() != 1.0) {
    fail(status, label, "version must be 1");
  }
  if (!document.contains("skeleton") || !document["skeleton"].is_string()) {
    fail(status, label, "skeleton must be a string");
  }

  ProcBody body;
  body.skeleton = document.contains("skeleton") && document["skeleton"].is_string()
                      ? document["skeleton"].get<std::string>()
                      : "";
  body.name = "unnamed";
  if (document.contains("name")) {
    if (!document["name"].is_string()) fail(status, label, "name must be a string");
    else body.name = document["name"].get<std::string>();
  }

  if (!document.contains("anthropometry") || !document["anthropometry"].is_object()) {
    fail(status, label, "anthropometry must be an object of external reference measurements");
  } else {
    const json& anthro = document["anthropometry"];
    const char* required[] = {"stature_m", "shoulder_breadth_m", "hip_breadth_m", "hand_length_m", "foot_length_m"};
    for (const char* key : required) {
      if (!anthro.contains(key)) fail(status, label, std::string("anthropometry is missing ") + key);
    }
    for (auto it = anthro.begin(); it != anthro.end(); ++it) {
      if (it.key() == "description") {
        if (!it.value().is_string()) fail(status, label, "anthropometry.description must be a string");
        else body.anthropometry.description = it.value().get<std::string>();
        continue;
      }
      if (!it.value().is_number()) {
        fail(status, label, "anthropometry." + it.key() + " must be a number");
        continue;
      }
      const double value = it.value().get<double>();
      if (!std::isfinite(value) || value <= 0.0) {
        fail(status, label, "anthropometry." + it.key() + " must be a positive finite measurement");
        continue;
      }
      body.anthropometry.measurements.emplace_back(it.key(), value);
    }
  }

  std::array<bool, kJointCount> joint_seen{};
  if (!document.contains("joints") || !document["joints"].is_array()) {
    fail(status, label, "joints must be an array");
  } else {
    for (const json& entry : document["joints"]) {
      if (!entry.is_object()) {
        fail(status, label, "each joint must be an object");
        continue;
      }
      rejectUnknown(entry, {"name", "rest_position_m"}, label, "joint", status);
      if (!entry.contains("name") || !entry["name"].is_string()) {
        fail(status, label, "joint is missing a name");
        continue;
      }
      const std::string name = entry["name"].get<std::string>();
      const std::optional<int> index = findJoint(name);
      if (!index) {
        fail(status, label, "unknown joint " + quoteName(name) + " for " + std::string(kSkeletonId));
        continue;
      }
      if (joint_seen[*index]) fail(status, label, "duplicate joint " + quoteName(name));
      joint_seen[*index] = true;
      body.joints[*index].name = name;
      if (!entry.contains("rest_position_m")) {
        fail(status, label, "joint " + quoteName(name) + " is missing rest_position_m");
      } else {
        readVec3(entry["rest_position_m"], body.joints[*index].rest_position, label,
                 "joint " + quoteName(name) + " rest_position_m", status);
      }
    }
  }
  for (int index = 0; index < kJointCount; ++index) {
    if (!joint_seen[index]) fail(status, label, "missing joint " + quoteName(kJointNames[index]));
  }

  std::array<bool, kBoneCount> bone_seen{};
  if (!document.contains("bones") || !document["bones"].is_array()) {
    fail(status, label, "bones must be an array");
  } else {
    for (const json& entry : document["bones"]) {
      if (!entry.is_object()) {
        fail(status, label, "each bone must be an object");
        continue;
      }
      rejectUnknown(entry,
                    {"name", "parent_joint", "child_joint", "length_m", "rest_direction", "rest_vector_m", "locked"},
                    label, "bone", status);
      if (!entry.contains("name") || !entry["name"].is_string()) {
        fail(status, label, "bone is missing a name");
        continue;
      }
      const std::string name = entry["name"].get<std::string>();
      const std::optional<int> index = findBone(name);
      if (!index) {
        fail(status, label, "unknown bone " + quoteName(name) + " for " + std::string(kSkeletonId));
        continue;
      }
      if (bone_seen[*index]) fail(status, label, "duplicate bone " + quoteName(name));
      bone_seen[*index] = true;
      BoneGeom& bone = body.bones[*index];
      bone.name = name;
      auto read_joint = [&](const char* key, int& slot) {
        if (!entry.contains(key) || !entry[key].is_string()) {
          fail(status, label, "bone " + quoteName(name) + " is missing " + key);
          return;
        }
        const std::string joint_name = entry[key].get<std::string>();
        const std::optional<int> joint = findJoint(joint_name);
        if (!joint) {
          fail(status, label, "bone " + quoteName(name) + " " + key + " is unknown joint " + quoteName(joint_name));
          return;
        }
        slot = *joint;
      };
      read_joint("parent_joint", bone.parent_joint);
      read_joint("child_joint", bone.child_joint);
      if (bone.parent_joint >= 0 && bone.child_joint >= 0 &&
          (bone.parent_joint != kCanonicalBones[*index].parent || bone.child_joint != kCanonicalBones[*index].child)) {
        fail(status, label,
             "invalid hierarchy: bone " + quoteName(name) + " parent_joint/child_joint do not match " +
                 std::string(kSkeletonId));
      }
      if (!entry.contains("length_m") || !entry["length_m"].is_number()) {
        fail(status, label, "bone " + quoteName(name) + " is missing length_m");
      } else {
        bone.length_m = entry["length_m"].get<double>();
      }
      if (entry.contains("rest_direction")) {
        readVec3(entry["rest_direction"], bone.rest_direction, label, "bone " + quoteName(name) + " rest_direction", status);
      } else {
        fail(status, label, "bone " + quoteName(name) + " is missing rest_direction");
      }
      if (entry.contains("rest_vector_m")) {
        readVec3(entry["rest_vector_m"], bone.rest_vector, label, "bone " + quoteName(name) + " rest_vector_m", status);
      } else {
        fail(status, label, "bone " + quoteName(name) + " is missing rest_vector_m");
      }
      if (!entry.contains("locked") || !entry["locked"].is_boolean()) {
        fail(status, label, "bone " + quoteName(name) + " must set locked to true");
      } else {
        bone.locked = entry["locked"].get<bool>();
      }
    }
  }
  for (int index = 0; index < kBoneCount; ++index) {
    if (!bone_seen[index]) fail(status, label, "missing bone " + quoteName(kCanonicalBones[index].name));
  }

  std::array<bool, kJointCount> constraint_seen{};
  if (!document.contains("constraints") || !document["constraints"].is_array()) {
    fail(status, label, "constraints must be an array");
  } else {
    for (const json& entry : document["constraints"]) {
      if (!entry.is_object()) {
        fail(status, label, "each constraint must be an object");
        continue;
      }
      if (!entry.contains("joint") || !entry["joint"].is_string()) {
        fail(status, label, "constraint is missing a joint name");
        continue;
      }
      if (!entry.contains("type") || !entry["type"].is_string()) {
        fail(status, label, "constraint is missing a type");
        continue;
      }
      const std::string joint_name = entry["joint"].get<std::string>();
      const std::optional<int> index = findJoint(joint_name);
      if (!index) {
        fail(status, label, "constraint refers to unknown joint " + quoteName(joint_name));
        continue;
      }
      if (constraint_seen[*index]) fail(status, label, "duplicate constraint for " + quoteName(joint_name));
      constraint_seen[*index] = true;
      const std::string type_name = entry["type"].get<std::string>();
      JointConstraint& constraint = body.constraints[*index];
      if (type_name == "root") constraint.type = JointType::Root;
      else if (type_name == "hinge") constraint.type = JointType::Hinge;
      else if (type_name == "ball") constraint.type = JointType::Ball;
      else if (type_name == "endpoint") constraint.type = JointType::Endpoint;
      else {
        fail(status, label, "joint " + quoteName(joint_name) + " has unknown constraint type " + quoteName(type_name));
        continue;
      }
      const auto common = {"joint", "type", "authority", "note"};
      if (constraint.type == JointType::Hinge) {
        rejectUnknown(entry, {"joint", "type", "authority", "note", "axis", "min_deg", "max_deg"}, label,
                      "constraint " + quoteName(joint_name), status);
        if (!entry.contains("axis")) fail(status, label, "hinge " + quoteName(joint_name) + " is missing axis");
        else readVec3(entry["axis"], constraint.axis, label, "hinge " + quoteName(joint_name) + " axis", status);
        if (!entry.contains("min_deg") || !entry["min_deg"].is_number() || !entry.contains("max_deg") ||
            !entry["max_deg"].is_number()) {
          fail(status, label, "hinge " + quoteName(joint_name) + " needs numeric min_deg and max_deg");
        } else {
          constraint.min_deg = entry["min_deg"].get<double>();
          constraint.max_deg = entry["max_deg"].get<double>();
        }
      } else if (constraint.type == JointType::Ball) {
        rejectUnknown(entry, {"joint", "type", "authority", "note", "twist_axis", "max_swing_deg", "max_twist_deg"},
                      label, "constraint " + quoteName(joint_name), status);
        if (!entry.contains("twist_axis")) fail(status, label, "ball " + quoteName(joint_name) + " is missing twist_axis");
        else readVec3(entry["twist_axis"], constraint.axis, label, "ball " + quoteName(joint_name) + " twist_axis", status);
        if (!entry.contains("max_swing_deg") || !entry["max_swing_deg"].is_number() || !entry.contains("max_twist_deg") ||
            !entry["max_twist_deg"].is_number()) {
          fail(status, label, "ball " + quoteName(joint_name) + " needs max_swing_deg and max_twist_deg");
        } else {
          constraint.max_swing_deg = entry["max_swing_deg"].get<double>();
          constraint.max_twist_deg = entry["max_twist_deg"].get<double>();
        }
      } else {
        for (auto it = entry.begin(); it != entry.end(); ++it) {
          if (!allowedKey(it.key(), common)) {
            fail(status, label, "constraint " + quoteName(joint_name) + " has unknown field " + quoteName(it.key()));
          }
        }
      }
      if (entry.contains("authority")) {
        if (!entry["authority"].is_string()) fail(status, label, "constraint authority must be a string");
        else constraint.authority = entry["authority"].get<std::string>();
      }
      if (entry.contains("note")) {
        if (!entry["note"].is_string()) fail(status, label, "constraint note must be a string");
        else constraint.note = entry["note"].get<std::string>();
      }
      if (glm::length(constraint.axis) > 1e-8 && finite3(constraint.axis)) {
        constraint.axis = glm::normalize(constraint.axis);
      }
    }
  }
  for (int index = 0; index < kJointCount; ++index) {
    if (!constraint_seen[index]) {
      fail(status, label, "missing constraint for " + quoteName(kJointNames[index]));
    }
  }

  linkHierarchy(body);
  if (status.ok()) validateProcBody(body, status);
  if (!status.ok()) return status;
  out = std::move(body);
  return status;
}

Status loadProcBodyFromFile(const std::filesystem::path& path, ProcBody& out) {
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
  Status status = loadProcBodyFromString(text, out, path.filename().string());
  if (status.ok()) out.source_path = path.string();
  return status;
}

}  // namespace proc
