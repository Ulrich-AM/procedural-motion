#include "import/bvh_importer.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

namespace proc {
namespace {

enum class Channel { Xpos, Ypos, Zpos, Xrot, Yrot, Zrot };

struct ParsedJoint {
  std::string name;
  int parent = -1;
  glm::dvec3 offset{0.0};
  std::vector<Channel> channels;
  bool end_site = false;
};

struct Parser {
  std::string text;
  std::size_t cursor = 0;
  Status* status = nullptr;
  std::string label;

  void fail(const std::string& message) { status->error(label + ": " + message); }

  void skip() {
    while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) ++cursor;
  }

  bool next(std::string& token) {
    skip();
    if (cursor >= text.size()) return false;
    if (text[cursor] == '{' || text[cursor] == '}') {
      token = text.substr(cursor, 1);
      ++cursor;
      return true;
    }
    const std::size_t start = cursor;
    while (cursor < text.size() && !std::isspace(static_cast<unsigned char>(text[cursor])) && text[cursor] != '{' &&
           text[cursor] != '}') {
      ++cursor;
    }
    token = text.substr(start, cursor - start);
    return !token.empty();
  }

  bool expect(const char* word) {
    std::string token;
    if (!next(token) || token != word) {
      fail(std::string("expected ") + word);
      return false;
    }
    return true;
  }

  bool readDouble(double& value) {
    std::string token;
    if (!next(token)) {
      fail("unexpected end of file");
      return false;
    }
    try {
      size_t used = 0;
      value = std::stod(token, &used);
      if (used != token.size() || !std::isfinite(value)) {
        fail("invalid number " + token);
        return false;
      }
    } catch (const std::exception&) {
      fail("invalid number " + token);
      return false;
    }
    return true;
  }
};

bool parseChannel(const std::string& token, Channel& channel) {
  if (token == "Xposition") channel = Channel::Xpos;
  else if (token == "Yposition") channel = Channel::Ypos;
  else if (token == "Zposition") channel = Channel::Zpos;
  else if (token == "Xrotation") channel = Channel::Xrot;
  else if (token == "Yrotation") channel = Channel::Yrot;
  else if (token == "Zrotation") channel = Channel::Zrot;
  else return false;
  return true;
}

bool parseJoint(Parser& parser, std::vector<ParsedJoint>& joints, int parent, bool end_site) {
  ParsedJoint joint;
  joint.parent = parent;
  joint.end_site = end_site;
  if (!end_site) {
    if (!parser.next(joint.name)) {
      parser.fail("missing joint name");
      return false;
    }
  } else {
    const std::string parent_name = parent >= 0 ? joints[parent].name : "root";
    joint.name = parent_name + "_EndSite";
  }
  joints.push_back(joint);
  const int self = static_cast<int>(joints.size()) - 1;
  if (!parser.expect("{")) return false;
  std::string token;
  while (parser.next(token)) {
    if (token == "}") return true;
    if (token == "OFFSET") {
      if (!parser.readDouble(joints[self].offset.x) || !parser.readDouble(joints[self].offset.y) ||
          !parser.readDouble(joints[self].offset.z)) {
        return false;
      }
    } else if (token == "CHANNELS") {
      double count_value = 0.0;
      if (!parser.readDouble(count_value)) return false;
      const int count = static_cast<int>(count_value);
      for (int index = 0; index < count; ++index) {
        std::string channel_name;
        if (!parser.next(channel_name)) return false;
        Channel channel;
        if (!parseChannel(channel_name, channel)) {
          parser.fail("unknown channel " + channel_name);
          return false;
        }
        joints[self].channels.push_back(channel);
      }
    } else if (token == "JOINT") {
      if (!parseJoint(parser, joints, self, false)) return false;
    } else if (token == "End") {
      std::string site;
      if (!parser.next(site) || site != "Site") {
        parser.fail("expected End Site");
        return false;
      }
      if (!parseJoint(parser, joints, self, true)) return false;
    } else {
      parser.fail("unexpected token " + token);
      return false;
    }
  }
  parser.fail("unclosed joint " + joints[self].name);
  return false;
}

glm::dquat channelRotation(const std::vector<Channel>& channels, const std::vector<double>& values, int& cursor) {
  glm::dquat rotation = quatIdentity();
  for (Channel channel : channels) {
    const double value = values[cursor++];
    glm::dvec3 axis(0.0);
    if (channel == Channel::Xrot) axis = glm::dvec3(1.0, 0.0, 0.0);
    else if (channel == Channel::Yrot) axis = glm::dvec3(0.0, 1.0, 0.0);
    else if (channel == Channel::Zrot) axis = glm::dvec3(0.0, 0.0, 1.0);
    else continue;
    rotation = quatAxisAngle(axis, glm::radians(value)) * rotation;
  }
  return rotation;
}

glm::dvec3 channelTranslation(const glm::dvec3& offset, const std::vector<Channel>& channels, const std::vector<double>& values,
                             int& cursor) {
  glm::dvec3 translation = offset;
  bool any = false;
  glm::dvec3 absolute = offset;
  for (Channel channel : channels) {
    const double value = values[cursor++];
    if (channel == Channel::Xpos) {
      absolute.x = value;
      any = true;
    } else if (channel == Channel::Ypos) {
      absolute.y = value;
      any = true;
    } else if (channel == Channel::Zpos) {
      absolute.z = value;
      any = true;
    }
  }
  if (any) translation = absolute;
  return translation;
}

}  // namespace

Status loadBvhFromString(std::string_view text, const AxisConvention& convention, SourceClip& out, std::string_view label) {
  Status status;
  Parser parser;
  parser.text = std::string(text);
  parser.status = &status;
  parser.label = std::string(label);
  if (!parser.expect("HIERARCHY") || !parser.expect("ROOT")) return status;
  std::vector<ParsedJoint> parsed;
  if (!parseJoint(parser, parsed, -1, false)) return status;
  if (parsed.empty()) {
    status.error(parser.label + ": BVH has no joints");
    return status;
  }
  if (!parser.expect("MOTION") || !parser.expect("Frames:")) return status;
  double frame_count_value = 0.0;
  if (!parser.readDouble(frame_count_value)) return status;
  std::string frame_token;
  if (!parser.next(frame_token) || frame_token != "Frame") {
    status.error(parser.label + ": expected Frame Time");
    return status;
  }
  if (!parser.expect("Time:")) return status;
  double frame_time = 0.0;
  if (!parser.readDouble(frame_time) || !(frame_time > 0.0)) {
    status.error(parser.label + ": Frame Time must be positive");
    return status;
  }
  const int frame_count = static_cast<int>(frame_count_value);
  if (frame_count <= 0) {
    status.error(parser.label + ": frame count must be positive");
    return status;
  }
  int channel_count = 0;
  for (const ParsedJoint& joint : parsed) channel_count += static_cast<int>(joint.channels.size());
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(frame_count) * static_cast<std::size_t>(channel_count));
  double sample = 0.0;
  while (parser.readDouble(sample)) {
    if (!status.ok()) return status;
    samples.push_back(sample);
  }
  status.errors.clear();
  if (static_cast<int>(samples.size()) != frame_count * channel_count) {
    status.error(parser.label + ": motion sample count does not match Frames and CHANNELS");
    return status;
  }

  SourceClip clip;
  clip.name = std::string(label);
  clip.format = "BVH";
  clip.nominal_fps = 1.0 / frame_time;
  clip.duration = frame_count <= 1 ? frame_time : static_cast<double>(frame_count - 1) * frame_time;
  clip.joints.resize(parsed.size());
  for (int index = 0; index < static_cast<int>(parsed.size()); ++index) {
    clip.joints[index].name = parsed[index].name;
    clip.joints[index].parent = parsed[index].parent;
    clip.joints[index].file_rest_translation = parsed[index].offset;
    clip.joints[index].file_rest_rotation = quatIdentity();
    clip.joints[index].rest_local_translation = parsed[index].offset;
    clip.joints[index].rest_local_rotation = quatIdentity();
  }
  int sample_cursor = 0;
  clip.frames.resize(frame_count);
  for (int frame_index = 0; frame_index < frame_count; ++frame_index) {
    SourceFrame& frame = clip.frames[frame_index];
    frame.time = static_cast<double>(frame_index) * frame_time;
    frame.file_translation.resize(parsed.size());
    frame.file_rotation.resize(parsed.size());
    frame.local_translation.resize(parsed.size());
    frame.local_rotation.resize(parsed.size());
    for (int joint = 0; joint < static_cast<int>(parsed.size()); ++joint) {
      const std::vector<double> values(samples.begin() + sample_cursor, samples.begin() + sample_cursor + parsed[joint].channels.size());
      int cursor = 0;
      const glm::dvec3 translation = channelTranslation(parsed[joint].offset, parsed[joint].channels, values, cursor);
      cursor = 0;
      const glm::dquat rotation = channelRotation(parsed[joint].channels, values, cursor);
      sample_cursor += static_cast<int>(parsed[joint].channels.size());
      frame.file_translation[joint] = translation;
      frame.file_rotation[joint] = rotation;
      frame.local_translation[joint] = translation;
      frame.local_rotation[joint] = rotation;
    }
  }
  rebuildSourceTopology(clip);
  rebuildSourceGlobals(clip);
  applyAxisConvention(clip, convention);
  if (!status.ok()) return status;
  out = std::move(clip);
  return status;
}

Status loadBvhFromFile(const std::filesystem::path& path, const AxisConvention& convention, SourceClip& out) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    Status status;
    status.error("could not open " + path.string());
    return status;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  Status status = loadBvhFromString(buffer.str(), convention, out, path.filename().string());
  if (status.ok()) out.path = path.string();
  return status;
}

}  // namespace proc
