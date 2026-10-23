#include "physics/physical_skeleton.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <numbers>
#include <sstream>
#include <vector>

namespace pm {
namespace {

constexpr double kPi = std::numbers::pi;

struct Xml {
  std::string text;
  int indent = 0;

  void raw(const std::string& line) {
    text.append(static_cast<size_t>(indent), ' ');
    text += line;
    text += '\n';
  }
  void open(const std::string& line) {
    raw(line);
    indent += 2;
  }
  void close(const std::string& line) {
    indent = std::max(0, indent - 2);
    raw(line);
  }
};

struct Primitive {
  std::string body;
  std::string name;
  Vec3 a;
  Vec3 b;
  Vec3 half{0.05, 0.05, 0.05};
  double quat[4]{1, 0, 0, 0};
  double radius = 0.05;
  bool sphere = false;
  bool box = false;
  bool collision = false;
  double density = 0;
  std::string rgba = "0.75 0.75 0.75 1";
};

std::string num(double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(6);
  stream << value;
  return stream.str();
}

std::string vec(Vec3 value) { return num(value.x) + " " + num(value.y) + " " + num(value.z); }

double capsuleVolume(double length, double radius) {
  return kPi * radius * radius * length + (4.0 / 3.0) * kPi * radius * radius * radius;
}

double sphereVolume(double radius) { return (4.0 / 3.0) * kPi * radius * radius * radius; }

double boxVolume(Vec3 half) { return 8.0 * half.x * half.y * half.z; }

void matToQuat(Vec3 x, Vec3 y, Vec3 z, double q[4]) {
  const double m00 = x.x, m01 = y.x, m02 = z.x;
  const double m10 = x.y, m11 = y.y, m12 = z.y;
  const double m20 = x.z, m21 = y.z, m22 = z.z;
  const double trace = m00 + m11 + m22;
  if (trace > 0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q[0] = 0.25 * s;
    q[1] = (m21 - m12) / s;
    q[2] = (m02 - m20) / s;
    q[3] = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q[0] = (m21 - m12) / s;
    q[1] = 0.25 * s;
    q[2] = (m01 + m10) / s;
    q[3] = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q[0] = (m02 - m20) / s;
    q[1] = (m01 + m10) / s;
    q[2] = 0.25 * s;
    q[3] = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q[0] = (m10 - m01) / s;
    q[1] = (m02 + m20) / s;
    q[2] = (m12 + m21) / s;
    q[3] = 0.25 * s;
  }
}

void asymmetricRange(int flexSign, double flexDeg, double extDeg, double& minimum, double& maximum) {
  if (flexSign >= 0) {
    minimum = -extDeg;
    maximum = flexDeg;
  } else {
    minimum = -flexDeg;
    maximum = extDeg;
  }
}

int kneeFlexSign(Vec3 axis, Vec3 knee, Vec3 ankle, Vec3 forward) {
  const Vec3 shin = ankle - knee;
  const Vec3 moved = rotateAround(shin, axis, 0.2);
  const double along = dot(moved - shin, forward);
  return along < 0.0 ? 1 : -1;
}

int elbowFlexSign(Vec3 axis, Vec3 shoulder, Vec3 elbow, Vec3 wrist, Vec3 forward) {
  const Vec3 forearm = wrist - elbow;
  const Vec3 upper = elbow - shoulder;
  const double bend = length(cross(normalize(upper), normalize(forearm)));
  const Vec3 positive = rotateAround(forearm, axis, 0.25);
  const Vec3 negative = rotateAround(forearm, axis, -0.25);
  const double distancePositive = length((elbow + positive) - shoulder);
  const double distanceNegative = length((elbow + negative) - shoulder);
  if (bend > 0.05 && std::abs(distancePositive - distanceNegative) > 1e-5) {
    return distancePositive < distanceNegative ? 1 : -1;
  }
  const double forwardMove = dot(positive - forearm, forward);
  return forwardMove > 0.0 ? 1 : -1;
}

const char* kBodyNames[] = {
    "pelvis",         "abdomen",        "chest",          "head",
    "left_upper_arm", "left_lower_arm", "left_hand",      "right_upper_arm",
    "right_lower_arm","right_hand",     "left_upper_leg", "left_lower_leg",
    "left_foot",      "right_upper_leg","right_lower_leg","right_foot",
};

double targetMass(const std::string& body) {
  if (body == "pelvis") return 11.0;
  if (body == "abdomen") return 9.0;
  if (body == "chest") return 13.0;
  if (body == "head") return 5.0;
  if (body.find("upper_arm") != std::string::npos) return 2.4;
  if (body.find("lower_arm") != std::string::npos) return 1.4;
  if (body.find("hand") != std::string::npos) return 0.5;
  if (body.find("upper_leg") != std::string::npos) return 9.2;
  if (body.find("lower_leg") != std::string::npos) return 3.8;
  if (body.find("foot") != std::string::npos) return 1.15;
  return 1.0;
}

std::string physicsBodyForBone(const std::string& bone) {
  if (bone == "spine_lower") return "abdomen";
  if (bone == "spine_upper" || bone == "chest" || bone == "left_shoulder" || bone == "right_shoulder") {
    return "chest";
  }
  if (bone == "neck" || bone == "head") return "head";
  return bone;
}

std::string originBoneForBody(const std::string& body) {
  if (body == "abdomen") return "spine_lower";
  if (body == "chest") return "spine_upper";
  if (body == "head") return "neck";
  return body;
}

std::string rgbaForBone(const std::string& bone) {
  if (bone.find("head") != std::string::npos || bone == "neck") return "0.84 0.84 0.84 1";
  if (bone.find("arm") != std::string::npos || bone.find("hand") != std::string::npos ||
      bone.find("shoulder") != std::string::npos) {
    return "0.62 0.62 0.62 1";
  }
  if (bone.find("leg") != std::string::npos || bone.find("foot") != std::string::npos) {
    return "0.52 0.52 0.52 1";
  }
  return "0.74 0.74 0.74 1";
}

class ModelBuilder {
 public:
  explicit ModelBuilder(const SkeletonDefinition& skeleton) : skeleton_(skeleton) {
    up_ = normalize(skeleton.up);
    if (length(up_) < 0.5) up_ = {0, 0, 1};
    right_ = skeleton.right - up_ * dot(skeleton.right, up_);
    right_ = normalize(right_);
    forward_ = normalize(cross(up_, right_));
    right_ = normalize(cross(forward_, up_));
  }

  bool build(const SimConfig& config, std::string& xml, std::string& error) {
    if (length(right_) < 0.5 || length(forward_) < 0.5) {
      error = "Skeleton axes are degenerate";
      return false;
    }
    for (const char* name : kBodyNames) {
      if (!point(originBoneForBody(name))) {
        error = std::string("Missing origin bone for body ") + name;
        return false;
      }
    }
    const char* requiredAuxiliary[] = {"head_top", "left_hand_end", "right_hand_end", "left_toe_end", "right_toe_end"};
    for (const char* name : requiredAuxiliary) {
      if (!point(name)) {
        error = std::string("Missing auxiliary point ") + name;
        return false;
      }
    }

    const double scale = proportionScale();
    addVisuals(scale);
    addCollisions(scale);
    if (!assignDensities(error)) return false;

    const double shift = config.spawnClearance - lowestCollisionZ();
    Xml xmlOut;
    writeXml(xmlOut, config, shift);
    xml = xmlOut.text;
    std::cout << "Spawn lift: " << shift << " m (clearance " << config.spawnClearance << " m)\n";
    std::cout << "Character axes right=(" << formatVec(right_) << ") forward=(" << formatVec(forward_)
              << ") up=(" << formatVec(up_) << ")\n";
    return true;
  }

 private:
  const Bone* point(const std::string& name) const { return skeleton_.findAny(name); }

  Vec3 at(const std::string& name) const {
    const Bone* bone = point(name);
    return bone ? bone->position : Vec3{};
  }

  Vec3 origin(const std::string& body) const { return at(originBoneForBody(body)); }

  Vec3 localPoint(const std::string& body, Vec3 world) const { return world - origin(body); }

  Vec3 localOf(const std::string& body, const std::string& bone) const { return localPoint(body, at(bone)); }

  Vec3 bodyDelta(const std::string& child, const std::string& parent) const {
    return origin(child) - origin(parent);
  }

  double proportionScale() const {
    const Bone* top = point("head_top");
    const double crown = top ? top->position.z : at("head").z + 0.18;
    const double height = std::max(0.8, crown);
    return std::clamp(height / 1.75, 0.8, 1.25);
  }

  void addVisualSphere(const std::string& body, const std::string& name, Vec3 local, double radius,
                       const std::string& rgba) {
    Primitive primitive;
    primitive.body = body;
    primitive.name = name;
    primitive.a = local;
    primitive.radius = radius;
    primitive.sphere = true;
    primitive.rgba = rgba;
    visuals_.push_back(primitive);
  }

  void addVisualCapsule(const std::string& body, const std::string& name, Vec3 a, Vec3 b, double radius,
                        const std::string& rgba) {
    if (length(b - a) < 0.008) return;
    Primitive primitive;
    primitive.body = body;
    primitive.name = name;
    primitive.a = a;
    primitive.b = b;
    primitive.radius = radius;
    primitive.rgba = rgba;
    visuals_.push_back(primitive);
  }

  void addCollisionCapsule(const std::string& body, const std::string& name, Vec3 a, Vec3 b, double radius) {
    const Vec3 delta = b - a;
    const double len = length(delta);
    if (len < 0.02) return;
    const double inset = std::min(radius * 0.4, len * 0.18);
    const Vec3 dir = delta / len;
    Primitive primitive;
    primitive.body = body;
    primitive.name = name;
    primitive.a = a + dir * inset;
    primitive.b = b - dir * inset;
    primitive.radius = radius;
    primitive.collision = true;
    collisions_.push_back(primitive);
  }

  void addCollisionBox(const std::string& body, const std::string& name, Vec3 center, Vec3 half, const double quat[4]) {
    Primitive primitive;
    primitive.body = body;
    primitive.name = name;
    primitive.a = center;
    primitive.half = half;
    primitive.quat[0] = quat[0];
    primitive.quat[1] = quat[1];
    primitive.quat[2] = quat[2];
    primitive.quat[3] = quat[3];
    primitive.box = true;
    primitive.collision = true;
    collisions_.push_back(primitive);
  }

  void addCollisionSphere(const std::string& body, const std::string& name, Vec3 center, double radius) {
    Primitive primitive;
    primitive.body = body;
    primitive.name = name;
    primitive.a = center;
    primitive.radius = radius;
    primitive.sphere = true;
    primitive.collision = true;
    collisions_.push_back(primitive);
  }

  void addVisuals(double scale) {
    const double boneRadius = 0.012 * scale;
    const double jointRadius = 0.026 * scale;
    for (const Bone& bone : skeleton_.bones) {
      const std::string body = physicsBodyForBone(bone.name);
      addVisualSphere(body, "vis_joint_" + bone.name, localOf(body, bone.name), jointRadius, "0.93 0.93 0.93 1");
      if (bone.parent.empty()) continue;
      const std::string parentBody = physicsBodyForBone(bone.parent);
      addVisualCapsule(parentBody, "vis_bone_" + bone.name, localOf(parentBody, bone.parent),
                       localPoint(parentBody, bone.position), boneRadius, rgbaForBone(bone.name));
    }

    auto limbEnd = [&](const std::string& body, const std::string& from, const std::string& to, const char* name) {
      if (!point(to)) return;
      addVisualCapsule(body, name, localOf(body, from), localOf(body, to), boneRadius, rgbaForBone(from));
    };
    limbEnd("left_hand", "left_hand", "left_hand_end", "vis_left_palm");
    limbEnd("right_hand", "right_hand", "right_hand_end", "vis_right_palm");
    limbEnd("left_foot", "left_foot", "left_toe_end", "vis_left_foot");
    limbEnd("right_foot", "right_foot", "right_toe_end", "vis_right_foot");
  }

  void addCollisions(double scale) {
    const double hipWidth = std::max(0.16, length(at("left_upper_leg") - at("right_upper_leg")));
    const double pelvisRadius = std::min(0.10 * scale, hipWidth * 0.33);
    addCollisionCapsule("pelvis", "col_pelvis", localOf("pelvis", "left_upper_leg"),
                        localOf("pelvis", "right_upper_leg"), std::max(0.05, pelvisRadius));
    addCollisionCapsule("abdomen", "col_abdomen", localOf("abdomen", "spine_lower"),
                        localOf("abdomen", "spine_upper"), 0.105 * scale);
    addCollisionCapsule("chest", "col_chest", localOf("chest", "spine_upper"), localOf("chest", "neck"),
                        0.115 * scale);
    addCollisionCapsule("chest", "col_shoulders", localOf("chest", "left_upper_arm"),
                        localOf("chest", "right_upper_arm"), 0.055 * scale);

    Vec3 skull = at("head");
    double skullRadius = 0.10 * scale;
    if (point("head_top")) {
      skull = (at("head") + at("head_top")) * 0.5;
      skullRadius = std::max(0.085 * scale, length(at("head_top") - at("head")) * 0.58);
    }
    addCollisionSphere("head", "col_head", localPoint("head", skull), skullRadius);

    auto limb = [&](const std::string& body, const std::string& from, const std::string& to, double radius) {
      addCollisionCapsule(body, "col_" + body, localOf(body, from), localOf(body, to), radius * scale);
    };
    limb("left_upper_arm", "left_upper_arm", "left_lower_arm", 0.046);
    limb("left_lower_arm", "left_lower_arm", "left_hand", 0.038);
    limb("left_hand", "left_hand", "left_hand_end", 0.030);
    limb("right_upper_arm", "right_upper_arm", "right_lower_arm", 0.046);
    limb("right_lower_arm", "right_lower_arm", "right_hand", 0.038);
    limb("right_hand", "right_hand", "right_hand_end", 0.030);
    limb("left_upper_leg", "left_upper_leg", "left_lower_leg", 0.066);
    limb("left_lower_leg", "left_lower_leg", "left_foot", 0.048);
    limb("right_upper_leg", "right_upper_leg", "right_lower_leg", 0.066);
    limb("right_lower_leg", "right_lower_leg", "right_foot", 0.048);
    // A flat sole. The imported toe sits below the ankle, so a capsule to that
    // point rolls onto the toe and pins the ankle at its dorsiflexion stop.
    auto sole = [&](const std::string& side) {
      const std::string foot = side + "_foot";
      const Vec3 ankle = localOf(foot, foot);
      Vec3 along = localOf(foot, side + "_toe_end") - ankle;
      along.z = 0;
      along = normalize(along);
      if (length(along) < 0.5) along = {0, -1, 0};
      constexpr double toe = 0.09;
      constexpr double heel = 0.09;
      constexpr double width = 0.08;
      constexpr double height = 0.022;
      // Keep the sole below the shin capsule so the round shin cannot become the contact.
      constexpr double drop = 0.074;
      const Vec3 center = ankle + along * ((toe - heel) * 0.5) + Vec3{0, 0, -drop + height * 0.5};
      const Vec3 yAxis = along;
      Vec3 zAxis{0, 0, 1};
      Vec3 xAxis = normalize(cross(yAxis, zAxis));
      if (length(xAxis) < 0.5) xAxis = {1, 0, 0};
      zAxis = normalize(cross(xAxis, yAxis));
      double quat[4] = {1, 0, 0, 0};
      matToQuat(xAxis, yAxis, zAxis, quat);
      addCollisionBox(foot, "col_" + foot, center, {width * 0.5, (toe + heel) * 0.5, height * 0.5}, quat);
    };
    sole("left");
    sole("right");
  }

  bool assignDensities(std::string& error) {
    std::map<std::string, double> volume;
    for (const Primitive& primitive : collisions_) {
      const double piece = primitive.sphere ? sphereVolume(primitive.radius)
                           : primitive.box  ? boxVolume(primitive.half)
                                            : capsuleVolume(length(primitive.b - primitive.a), primitive.radius);
      if (piece < 1e-8) {
        error = "Collision primitive has no volume: " + primitive.name;
        return false;
      }
      volume[primitive.body] += piece;
    }
    for (const char* name : kBodyNames) {
      if (volume[name] < 1e-8) {
        error = std::string("Body has no collision volume: ") + name;
        return false;
      }
    }
    for (Primitive& primitive : collisions_) {
      primitive.density = targetMass(primitive.body) / volume[primitive.body];
    }
    return true;
  }

  double lowestCollisionZ() const {
    double lowest = 1e9;
    for (const Primitive& primitive : collisions_) {
      const double originZ = origin(primitive.body).z;
      if (primitive.sphere) {
        lowest = std::min(lowest, originZ + primitive.a.z - primitive.radius);
      } else if (primitive.box) {
        lowest = std::min(lowest, originZ + primitive.a.z - primitive.half.z);
      } else {
        lowest = std::min(lowest, originZ + primitive.a.z - primitive.radius);
        lowest = std::min(lowest, originZ + primitive.b.z - primitive.radius);
      }
    }
    return lowest;
  }

  void addHinge(const std::string& body, const std::string& name, Vec3 axis, double minimum, double maximum,
                double damping) {
    axis = normalize(axis);
    std::ostringstream line;
    line.setf(std::ios::fixed);
    line.precision(6);
    line << "<joint name=\"" << name << "\" type=\"hinge\" limited=\"true\" axis=\"" << vec(axis)
         << "\" range=\"" << minimum << " " << maximum << "\" damping=\"" << damping
         << "\" armature=\"0.02\" solreflimit=\"0.02 1\" solimplimit=\"0.9 0.95 0.001\"/>";
    joints_[body].push_back(line.str());
  }

  void addTriaxial(const std::string& body, const std::string& prefix, double flex, double lateral, double twist,
                   double damping) {
    addHinge(body, prefix + "_flex", right_, -flex, flex, damping);
    addHinge(body, prefix + "_abd", forward_, -lateral, lateral, damping);
    addHinge(body, prefix + "_twist", up_, -twist, twist, damping);
  }

  // T-pose arms run along the body's side axis, so the elbow cannot use that axis.
  // The hinge is perpendicular to the forearm and bends the hand toward forward.
  Vec3 elbowAxis(Vec3 elbow, Vec3 wrist) const {
    const Vec3 bone = normalize(wrist - elbow);
    Vec3 axis = cross(bone, forward_);
    if (length(axis) < 0.2) axis = cross(bone, up_);
    return normalize(axis);
  }

  Vec3 wristDeviationAxis(Vec3 elbowAxisValue, Vec3 elbow, Vec3 wrist) const {
    return normalize(cross(elbowAxisValue, normalize(wrist - elbow)));
  }

  void addLimbHinge(const std::string& body, const std::string& name, Vec3 axis, Vec3 joint, Vec3 end, bool knee,
                    double damping) {
    const int sign = knee ? kneeFlexSign(axis, joint, end, forward_) : elbowFlexSign(axis, joint, joint, end, forward_);
    double minimum = 0;
    double maximum = 0;
    if (knee) {
      asymmetricRange(sign, 150.0, 6.0, minimum, maximum);
    } else {
      const Vec3 forearm = end - joint;
      const Vec3 moved = rotateAround(forearm, axis, 0.3);
      const int towardForward = dot(moved - forearm, forward_) >= 0.0 ? 1 : -1;
      asymmetricRange(towardForward, 145.0, 12.0, minimum, maximum);
      (void)sign;
    }
    addHinge(body, name, axis, minimum, maximum, damping);
    std::cout << "  " << name << " axis (" << formatVec(normalize(axis)) << ") range deg [" << minimum << ", "
              << maximum << "]\n";
  }

  void buildJoints() {
    addTriaxial("abdomen", "spine_lower", 30, 25, 25, 2.5);
    addTriaxial("chest", "spine_upper", 30, 25, 25, 2.5);
    addTriaxial("head", "neck", 40, 30, 40, 0.8);
    addTriaxial("left_upper_arm", "left_shoulder", 150, 140, 90, 1.2);
    addTriaxial("right_upper_arm", "right_shoulder", 150, 140, 90, 1.2);
    const Vec3 leftElbowAxis = elbowAxis(at("left_lower_arm"), at("left_hand"));
    const Vec3 rightElbowAxis = elbowAxis(at("right_lower_arm"), at("right_hand"));
    addLimbHinge("left_lower_arm", "left_elbow", leftElbowAxis, at("left_lower_arm"), at("left_hand"), false, 0.7);
    addLimbHinge("right_lower_arm", "right_elbow", rightElbowAxis, at("right_lower_arm"), at("right_hand"), false, 0.7);
    addHinge("left_hand", "left_wrist_flex", leftElbowAxis, -60, 60, 0.25);
    addHinge("left_hand", "left_wrist_yaw", wristDeviationAxis(leftElbowAxis, at("left_lower_arm"), at("left_hand")),
             -35, 35, 0.25);
    addHinge("right_hand", "right_wrist_flex", rightElbowAxis, -60, 60, 0.25);
    addHinge("right_hand", "right_wrist_yaw",
             wristDeviationAxis(rightElbowAxis, at("right_lower_arm"), at("right_hand")), -35, 35, 0.25);
    addTriaxial("left_upper_leg", "left_hip", 105, 55, 45, 8.0);
    addTriaxial("right_upper_leg", "right_hip", 105, 55, 45, 8.0);
    addLimbHinge("left_lower_leg", "left_knee", right_, at("left_lower_leg"), at("left_foot"), true, 6.0);
    addLimbHinge("right_lower_leg", "right_knee", right_, at("right_lower_leg"), at("right_foot"), true, 6.0);
    addHinge("left_foot", "left_ankle_pitch", right_, -40, 40, 0.6);
    addHinge("left_foot", "left_ankle_roll", forward_, -25, 25, 0.6);
    addHinge("right_foot", "right_ankle_pitch", right_, -40, 40, 0.6);
    addHinge("right_foot", "right_ankle_roll", forward_, -25, 25, 0.6);
  }

  void emitPrimitives(Xml& xml, const std::string& body) {
    for (const std::string& joint : joints_[body]) xml.raw(joint);
    auto emit = [&](const Primitive& primitive) {
      if (primitive.body != body) return;
      std::ostringstream line;
      line.setf(std::ios::fixed);
      line.precision(6);
      if (primitive.collision) {
        line << "<geom class=\"collision\" name=\"" << primitive.name << "\" density=\"" << primitive.density
             << "\" ";
      } else {
        line << "<geom class=\"visual\" name=\"" << primitive.name << "\" rgba=\"" << primitive.rgba << "\" ";
      }
      if (primitive.sphere) {
        line << "type=\"sphere\" pos=\"" << vec(primitive.a) << "\" size=\"" << primitive.radius << "\"/>";
      } else if (primitive.box) {
        line << "type=\"box\" pos=\"" << vec(primitive.a) << "\" size=\"" << vec(primitive.half) << "\" quat=\""
             << num(primitive.quat[0]) << " " << num(primitive.quat[1]) << " " << num(primitive.quat[2]) << " "
             << num(primitive.quat[3]) << "\"/>";
      } else {
        line << "type=\"capsule\" fromto=\"" << vec(primitive.a) << " " << vec(primitive.b) << "\" size=\""
             << primitive.radius << "\"/>";
      }
      xml.raw(line.str());
    };
    for (const Primitive& primitive : visuals_) emit(primitive);
    for (const Primitive& primitive : collisions_) emit(primitive);
  }

  static std::string bodyTag(const std::string& name, Vec3 pos) {
    return "<body name=\"" + name + "\" pos=\"" + vec(pos) + "\">";
  }

  void emitArm(Xml& xml, const std::string& side) {
    const std::string upper = side + "_upper_arm";
    const std::string lower = side + "_lower_arm";
    const std::string hand = side + "_hand";
    xml.open(bodyTag(upper, bodyDelta(upper, "chest")));
    emitPrimitives(xml, upper);
    xml.open(bodyTag(lower, bodyDelta(lower, upper)));
    emitPrimitives(xml, lower);
    xml.open(bodyTag(hand, bodyDelta(hand, lower)));
    emitPrimitives(xml, hand);
    xml.close("</body>");
    xml.close("</body>");
    xml.close("</body>");
  }

  void emitLeg(Xml& xml, const std::string& side) {
    const std::string upper = side + "_upper_leg";
    const std::string lower = side + "_lower_leg";
    const std::string foot = side + "_foot";
    xml.open(bodyTag(upper, bodyDelta(upper, "pelvis")));
    emitPrimitives(xml, upper);
    xml.open(bodyTag(lower, bodyDelta(lower, upper)));
    emitPrimitives(xml, lower);
    xml.open(bodyTag(foot, bodyDelta(foot, lower)));
    emitPrimitives(xml, foot);
    xml.close("</body>");
    xml.close("</body>");
    xml.close("</body>");
  }

  void writeXml(Xml& xml, const SimConfig& config, double shift) {
    buildJoints();
    xml.raw("<mujoco model=\"phase0_humanoid\">");
    xml.indent += 2;
    xml.raw("<!-- Phase 0 passive ragdoll. No actuators, springs, or animation. -->");
    xml.raw("<compiler angle=\"degree\" boundmass=\"0.05\" boundinertia=\"0.0008\" autolimits=\"true\"/>");
    xml.raw("<option timestep=\"" + num(config.timestep) + "\" gravity=\"0 0 " + num(config.gravityZ) +
            "\" integrator=\"implicitfast\" solver=\"Newton\" iterations=\"" +
            std::to_string(config.solverIterations) + "\" tolerance=\"1e-8\" cone=\"pyramidal\"/>");
    xml.raw("<visual>");
    xml.indent += 2;
    xml.raw("<rgba fog=\"0.93 0.93 0.93 1\"/>");
    xml.raw("<headlight ambient=\"0.68 0.68 0.68\" diffuse=\"0.42 0.42 0.42\" specular=\"0 0 0\"/>");
    xml.raw("<map znear=\"0.02\" zfar=\"60\" fogstart=\"40\" fogend=\"58\"/>");
    xml.indent -= 2;
    xml.raw("</visual>");
    xml.raw("<default>");
    xml.indent += 2;
    xml.raw("<default class=\"visual\">");
    xml.raw("  <geom density=\"0\" contype=\"0\" conaffinity=\"0\" group=\"1\" rgba=\"0.75 0.75 0.75 1\"/>");
    xml.raw("</default>");
    xml.raw("<default class=\"collision\">");
    xml.raw("  <geom group=\"3\" contype=\"1\" conaffinity=\"1\" rgba=\"0.35 0.35 0.35 0.28\" "
            "friction=\"0.9 0.02 0.001\" solref=\"0.02 1\" solimp=\"0.9 0.95 0.001\" margin=\"0.001\"/>");
    xml.raw("</default>");
    xml.indent -= 2;
    xml.raw("</default>");
    xml.raw("<worldbody>");
    xml.indent += 2;
    xml.raw("<geom name=\"floor\" type=\"plane\" size=\"24 24 0.1\" pos=\"0 0 0\" group=\"0\" "
            "contype=\"1\" conaffinity=\"1\" rgba=\"0.86 0.86 0.86 1\" friction=\"1.1 0.01 0.001\" "
            "solref=\"0.03 1\" solimp=\"0.9 0.95 0.001\"/>");

    Vec3 pelvis = origin("pelvis");
    pelvis.z += shift;
    xml.open(bodyTag("pelvis", pelvis));
    xml.raw("<freejoint name=\"root\"/>");
    emitPrimitives(xml, "pelvis");

    xml.open(bodyTag("abdomen", bodyDelta("abdomen", "pelvis")));
    emitPrimitives(xml, "abdomen");
    xml.open(bodyTag("chest", bodyDelta("chest", "abdomen")));
    emitPrimitives(xml, "chest");
    xml.open(bodyTag("head", bodyDelta("head", "chest")));
    emitPrimitives(xml, "head");
    xml.close("</body>");
    emitArm(xml, "left");
    emitArm(xml, "right");
    xml.close("</body>");
    xml.close("</body>");
    emitLeg(xml, "left");
    emitLeg(xml, "right");
    xml.close("</body>");
    xml.indent -= 2;
    xml.raw("</worldbody>");
    xml.raw("<contact>");
    xml.indent += 2;
    const char* excludes[][2] = {
        {"left_upper_leg", "right_upper_leg"}, {"left_upper_leg", "abdomen"},
        {"right_upper_leg", "abdomen"},        {"left_upper_arm", "head"},
        {"right_upper_arm", "head"},           {"left_lower_arm", "chest"},
        {"right_lower_arm", "chest"},          {"left_hand", "chest"},
        {"right_hand", "chest"},
    };
    for (const auto& pair : excludes) {
      xml.raw(std::string("<exclude body1=\"") + pair[0] + "\" body2=\"" + pair[1] + "\"/>");
    }
    xml.indent -= 2;
    xml.raw("</contact>");
    xml.indent -= 2;
    xml.raw("</mujoco>");
  }

  const SkeletonDefinition& skeleton_;
  Vec3 right_{1, 0, 0};
  Vec3 forward_{0, 1, 0};
  Vec3 up_{0, 0, 1};
  std::vector<Primitive> visuals_;
  std::vector<Primitive> collisions_;
  std::map<std::string, std::vector<std::string>> joints_;
};

const char* jointTypeName(int type) {
  switch (type) {
    case mjJNT_FREE:
      return "free";
    case mjJNT_BALL:
      return "ball";
    case mjJNT_SLIDE:
      return "slide";
    case mjJNT_HINGE:
      return "hinge";
    default:
      return "unknown";
  }
}

}  // namespace

PhysicalSkeleton::~PhysicalSkeleton() { destroy(); }

void PhysicalSkeleton::destroy() {
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

bool PhysicalSkeleton::load(const SkeletonDefinition& skeleton, const SimConfig& config,
                            const std::filesystem::path& xmlPath) {
  destroy();
  error_.clear();
  gravityZ_ = config.gravityZ;
  gravityEnabled_ = true;

  ModelBuilder builder(skeleton);
  std::string xml;
  if (!builder.build(config, xml, error_)) return false;

  std::filesystem::create_directories(xmlPath.parent_path());
  {
    std::ofstream output(xmlPath);
    if (!output) {
      error_ = "Could not write " + xmlPath.string();
      return false;
    }
    output << xml;
  }

  char mjError[4096] = {};
  model_ = mj_loadXML(xmlPath.string().c_str(), nullptr, mjError, sizeof(mjError));
  if (!model_) {
    error_ = std::string("MuJoCo rejected the humanoid model: ") + mjError;
    return false;
  }
  data_ = mj_makeData(model_);
  if (!data_) {
    error_ = "mj_makeData failed";
    destroy();
    return false;
  }
  mj_forward(model_, data_);

  if (model_->nu != 0) {
    error_ = "Phase 0 model must not contain actuators";
    destroy();
    return false;
  }
  const int root = mj_name2id(model_, mjOBJ_JOINT, "root");
  if (root < 0 || model_->jnt_type[root] != mjJNT_FREE) {
    error_ = "Pelvis root is not a free joint";
    destroy();
    return false;
  }

  std::cout << "Physical bodies (target vs simulated mass):\n";
  for (const char* name : kBodyNames) {
    const int id = mj_name2id(model_, mjOBJ_BODY, name);
    if (id < 0) {
      error_ = std::string("Model is missing body ") + name;
      destroy();
      return false;
    }
    std::cout << "  " << name << "  " << model_->body_mass[id] << " kg  target " << targetMass(name) << " kg\n";
  }
  std::cout << "Total mass: " << mj_getTotalmass(model_) << " kg\n";
  std::cout << "Degrees of freedom: " << model_->nv << "\n";
  std::cout << "Wrote " << xmlPath.string() << "\n";
  indexHinges();
  return true;
}

void PhysicalSkeleton::indexHinges() {
  hinges_.clear();
  rootDof_ = 0;
  if (!model_) return;
  const int root = mj_name2id(model_, mjOBJ_JOINT, "root");
  if (root >= 0) rootDof_ = model_->jnt_dofadr[root];
  for (int j = 0; j < model_->njnt; ++j) {
    if (model_->jnt_type[j] != mjJNT_HINGE) continue;
    const int dof = model_->jnt_dofadr[j];
    if (dof < rootDof_ + 6 || dof >= model_->nv) continue;
    const char* name = mj_id2name(model_, mjOBJ_JOINT, j);
    if (!name || name[0] == '\0') continue;
    HingeSlot slot;
    slot.dof = dof;
    hinges_.emplace(name, slot);
  }
}

void PhysicalSkeleton::clearJointTorques() {
  for (auto& entry : hinges_) entry.second.command = 0;
}

bool PhysicalSkeleton::setJointTorque(const std::string& name, double torque, double absoluteLimit) {
  if (name.empty() || name == "root") return false;
  if (!std::isfinite(torque) || !std::isfinite(absoluteLimit) || absoluteLimit <= 0) return false;
  const auto found = hinges_.find(name);
  if (found == hinges_.end()) return false;
  const double limit = absoluteLimit;
  found->second.command = std::clamp(torque, -limit, limit);
  return true;
}

double PhysicalSkeleton::appliedJointTorque(const std::string& name) const {
  const auto found = hinges_.find(name);
  if (found == hinges_.end()) return 0;
  return found->second.applied;
}

bool PhysicalSkeleton::startBodyForcePulse(const std::string& bodyName, Vec3 force, double durationSeconds) {
  if (!model_ || !std::isfinite(durationSeconds) || durationSeconds <= 0) return false;
  if (!std::isfinite(force.x) || !std::isfinite(force.y) || !std::isfinite(force.z)) return false;
  const int body = mj_name2id(model_, mjOBJ_BODY, bodyName.c_str());
  if (body <= 0) return false;
  pulseBody_ = body;
  pulseForce_ = force;
  pulseRemaining_ = durationSeconds;
  pulseActive_ = true;
  return true;
}

void PhysicalSkeleton::reset() {
  if (!model_ || !data_) return;
  mj_resetData(model_, data_);
  clearJointTorques();
  for (auto& entry : hinges_) entry.second.applied = 0;
  lastPeakTorque_ = 0;
  lastRootApplied_ = 0;
  pulseActive_ = false;
  pulseRemaining_ = 0;
  if (data_->qfrc_applied) mju_zero(data_->qfrc_applied, model_->nv);
  if (data_->xfrc_applied) mju_zero(data_->xfrc_applied, 6 * model_->nbody);
  if (!gravityEnabled_) model_->opt.gravity[2] = 0;
  mj_forward(model_, data_);
}

void PhysicalSkeleton::step() {
  if (!model_ || !data_) return;
  mju_zero(data_->qfrc_applied, model_->nv);
  lastPeakTorque_ = 0;
  lastRootApplied_ = 0;
  for (auto& entry : hinges_) {
    HingeSlot& slot = entry.second;
    data_->qfrc_applied[slot.dof] = slot.command;
    slot.applied = slot.command;
    lastPeakTorque_ = std::max(lastPeakTorque_, std::abs(slot.command));
    slot.command = 0;
  }
  for (int i = 0; i < 6; ++i) {
    const int dof = rootDof_ + i;
    if (dof >= 0 && dof < model_->nv) {
      lastRootApplied_ = std::max(lastRootApplied_, std::abs(data_->qfrc_applied[dof]));
    }
  }

  mju_zero(data_->xfrc_applied, 6 * model_->nbody);
  if (pulseActive_ && pulseBody_ > 0 && pulseBody_ < model_->nbody) {
    mjtNum* wrench = data_->xfrc_applied + 6 * pulseBody_;
    wrench[3] = pulseForce_.x;
    wrench[4] = pulseForce_.y;
    wrench[5] = pulseForce_.z;
    pulseRemaining_ -= model_->opt.timestep;
    if (pulseRemaining_ <= 0) pulseActive_ = false;
  }

  mj_step(model_, data_);
  mju_zero(data_->qfrc_applied, model_->nv);
  mju_zero(data_->xfrc_applied, 6 * model_->nbody);
}

void PhysicalSkeleton::setGravityEnabled(bool enabled) {
  gravityEnabled_ = enabled;
  if (model_) model_->opt.gravity[2] = enabled ? gravityZ_ : 0;
}

double PhysicalSkeleton::timestep() const {
  return model_ ? model_->opt.timestep : 0;
}

BodyState PhysicalSkeleton::sample() {
  BodyState state;
  if (!model_ || !data_) return state;
  state.time = data_->time;

  Vec3 weighted{0, 0, 0};
  Vec3 weightedVelocity{0, 0, 0};
  double mass = 0;
  for (int i = 1; i < model_->nbody; ++i) {
    BodyKinematics body;
    const char* name = mj_id2name(model_, mjOBJ_BODY, i);
    body.name = name ? name : "";
    body.position = {data_->xpos[3 * i], data_->xpos[3 * i + 1], data_->xpos[3 * i + 2]};
    body.com = {data_->xipos[3 * i], data_->xipos[3 * i + 1], data_->xipos[3 * i + 2]};
    for (int k = 0; k < 9; ++k) body.rotation[k] = data_->xmat[9 * i + k];
    for (int k = 0; k < 4; ++k) body.quat[k] = data_->xquat[4 * i + k];

    mjtNum velocity[6] = {};
    mj_objectVelocity(model_, data_, mjOBJ_BODY, i, velocity, 0);
    body.angularVelocity = {velocity[0], velocity[1], velocity[2]};
    body.linearVelocity = {velocity[3], velocity[4], velocity[5]};
    state.bodies.push_back(body);

    const double bodyMass = model_->body_mass[i];
    mass += bodyMass;
    weighted = weighted + body.com * bodyMass;
    weightedVelocity = weightedVelocity + body.linearVelocity * bodyMass;
  }
  state.mass = mass;
  if (mass > 1e-8) {
    state.centerOfMass = weighted / mass;
    state.centerOfMassVelocity = weightedVelocity / mass;
  }

  for (int j = 0; j < model_->njnt; ++j) {
    JointState joint;
    const char* name = mj_id2name(model_, mjOBJ_JOINT, j);
    joint.name = name ? name : "";
    joint.type = jointTypeName(model_->jnt_type[j]);
    const int address = model_->jnt_qposadr[j];
    const int dof = model_->jnt_dofadr[j];
    if (model_->jnt_type[j] == mjJNT_HINGE || model_->jnt_type[j] == mjJNT_SLIDE) {
      joint.angle = data_->qpos[address];
      if (dof >= 0 && dof < model_->nv) joint.velocity = data_->qvel[dof];
    }
    if (model_->jnt_limited[j]) {
      joint.rangeMin = model_->jnt_range[2 * j];
      joint.rangeMax = model_->jnt_range[2 * j + 1];
    }
    joint.anchor = {data_->xanchor[3 * j], data_->xanchor[3 * j + 1], data_->xanchor[3 * j + 2]};
    joint.axis = {data_->xaxis[3 * j], data_->xaxis[3 * j + 1], data_->xaxis[3 * j + 2]};
    state.joints.push_back(joint);
  }

  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact& contact = data_->contact[i];
    ContactState item;
    item.position = {contact.pos[0], contact.pos[1], contact.pos[2]};
    item.normal = {contact.frame[0], contact.frame[1], contact.frame[2]};
    mjtNum force[6] = {};
    mj_contactForce(model_, data_, i, force);
    item.normalForce = force[0];
    const char* geom1 = mj_id2name(model_, mjOBJ_GEOM, contact.geom[0]);
    const char* geom2 = mj_id2name(model_, mjOBJ_GEOM, contact.geom[1]);
    item.geom1 = geom1 ? geom1 : "";
    item.geom2 = geom2 ? geom2 : "";
    state.contacts.push_back(item);
  }
  return state;
}

}  // namespace pm
