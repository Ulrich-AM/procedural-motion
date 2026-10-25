#include "render/renderer.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace proc {
namespace {

constexpr unsigned kArrayBuffer = 0x8892;
constexpr unsigned kElementBuffer = 0x8893;
constexpr unsigned kStaticDraw = 0x88E4;
constexpr unsigned kDynamicDraw = 0x88E8;
constexpr unsigned kFragmentShader = 0x8B30;
constexpr unsigned kVertexShader = 0x8B31;
constexpr unsigned kCompileStatus = 0x8B81;
constexpr unsigned kLinkStatus = 0x8B82;

using GlInt = int;
using GlSizei = int;
using GlSizeiptr = std::ptrdiff_t;

#define PROC_GL(name, ret, args) ret(APIENTRY* name) args = nullptr
PROC_GL(glGenVertexArrays, void, (GlSizei, unsigned*));
PROC_GL(glBindVertexArray, void, (unsigned));
PROC_GL(glGenBuffers, void, (GlSizei, unsigned*));
PROC_GL(glBindBuffer, void, (unsigned, unsigned));
PROC_GL(glBufferData, void, (unsigned, GlSizeiptr, const void*, unsigned));
PROC_GL(glEnableVertexAttribArray, void, (unsigned));
PROC_GL(glVertexAttribPointer, void, (unsigned, GlInt, unsigned, unsigned char, GlSizeiptr, const void*));
PROC_GL(glCreateShader, unsigned, (unsigned));
PROC_GL(glShaderSource, void, (unsigned, GlSizei, const char* const*, const GlInt*));
PROC_GL(glCompileShader, void, (unsigned));
PROC_GL(glGetShaderiv, void, (unsigned, unsigned, GlInt*));
PROC_GL(glGetShaderInfoLog, void, (unsigned, GlSizei, GlSizei*, char*));
PROC_GL(glDeleteShader, void, (unsigned));
PROC_GL(glCreateProgram, unsigned, ());
PROC_GL(glAttachShader, void, (unsigned, unsigned));
PROC_GL(glLinkProgram, void, (unsigned));
PROC_GL(glGetProgramiv, void, (unsigned, unsigned, GlInt*));
PROC_GL(glGetProgramInfoLog, void, (unsigned, GlSizei, GlSizei*, char*));
PROC_GL(glUseProgram, void, (unsigned));
PROC_GL(glDeleteProgram, void, (unsigned));
PROC_GL(glGetUniformLocation, GlInt, (unsigned, const char*));
PROC_GL(glUniformMatrix4fv, void, (GlInt, GlSizei, unsigned char, const float*));
PROC_GL(glUniform3f, void, (GlInt, float, float, float));
PROC_GL(glUniform1f, void, (GlInt, float));
#undef PROC_GL

bool loadFunction(const char* name, void** pointer) {
  *pointer = reinterpret_cast<void*>(glfwGetProcAddress(name));
  return *pointer != nullptr;
}

const char* kVertexShaderSource = R"(#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_mvp;
uniform mat4 u_model;
out vec3 v_normal;
void main() {
  gl_Position = u_mvp * vec4(a_position, 1.0);
  v_normal = mat3(u_model) * a_normal;
}
)";

const char* kFragmentShaderSource = R"(#version 330 core
in vec3 v_normal;
uniform vec3 u_color;
uniform float u_lit;
out vec4 frag_color;
void main() {
  vec3 color = u_color;
  if (u_lit > 0.5) {
    vec3 normal = normalize(v_normal);
    vec3 key = normalize(vec3(0.25, 0.45, 0.86));
    float shade = max(0.22, dot(normal, key));
    color *= shade;
  }
  frag_color = vec4(color, 1.0);
}
)";

unsigned compileShader(unsigned type, const char* source, std::string& error) {
  const unsigned shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  int success = 0;
  glGetShaderiv(shader, kCompileStatus, &success);
  if (!success) {
    char log[1024];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    error = log;
  }
  return shader;
}

void pushVertex(std::vector<float>& vertices, float x, float y, float z, float nx, float ny, float nz) {
  vertices.insert(vertices.end(), {x, y, z, nx, ny, nz});
}

}  // namespace

bool Renderer::init(std::string& error) {
#define LOAD(name)                                     \
  if (!loadFunction(#name, reinterpret_cast<void**>(&name))) { \
    error = "missing OpenGL entry " #name;            \
    return false;                                      \
  }
  LOAD(glGenVertexArrays)
  LOAD(glBindVertexArray)
  LOAD(glGenBuffers)
  LOAD(glBindBuffer)
  LOAD(glBufferData)
  LOAD(glEnableVertexAttribArray)
  LOAD(glVertexAttribPointer)
  LOAD(glCreateShader)
  LOAD(glShaderSource)
  LOAD(glCompileShader)
  LOAD(glGetShaderiv)
  LOAD(glGetShaderInfoLog)
  LOAD(glDeleteShader)
  LOAD(glCreateProgram)
  LOAD(glAttachShader)
  LOAD(glLinkProgram)
  LOAD(glGetProgramiv)
  LOAD(glGetProgramInfoLog)
  LOAD(glUseProgram)
  LOAD(glDeleteProgram)
  LOAD(glGetUniformLocation)
  LOAD(glUniformMatrix4fv)
  LOAD(glUniform3f)
  LOAD(glUniform1f)
#undef LOAD

  const unsigned vertex = compileShader(kVertexShader, kVertexShaderSource, error);
  if (!error.empty()) return false;
  const unsigned fragment = compileShader(kFragmentShader, kFragmentShaderSource, error);
  if (!error.empty()) return false;
  program_ = glCreateProgram();
  glAttachShader(program_, vertex);
  glAttachShader(program_, fragment);
  glLinkProgram(program_);
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  int linked = 0;
  glGetProgramiv(program_, kLinkStatus, &linked);
  if (!linked) {
    char log[1024];
    glGetProgramInfoLog(program_, sizeof(log), nullptr, log);
    error = log;
    return false;
  }
  u_mvp_ = glGetUniformLocation(program_, "u_mvp");
  u_model_ = glGetUniformLocation(program_, "u_model");
  u_color_ = glGetUniformLocation(program_, "u_color");
  u_lit_ = glGetUniformLocation(program_, "u_lit");

  std::vector<float> sphere_vertices;
  std::vector<unsigned> sphere_indices;
  constexpr int kStacks = 14;
  constexpr int kSlices = 20;
  for (int stack = 0; stack <= kStacks; ++stack) {
    const float v = static_cast<float>(stack) / static_cast<float>(kStacks);
    const float phi = v * 3.14159265f;
    for (int slice = 0; slice <= kSlices; ++slice) {
      const float u = static_cast<float>(slice) / static_cast<float>(kSlices);
      const float theta = u * 6.2831853f;
      const float x = std::sin(phi) * std::cos(theta);
      const float y = std::sin(phi) * std::sin(theta);
      const float z = std::cos(phi);
      pushVertex(sphere_vertices, x, y, z, x, y, z);
    }
  }
  for (int stack = 0; stack < kStacks; ++stack) {
    for (int slice = 0; slice < kSlices; ++slice) {
      const unsigned a = static_cast<unsigned>(stack * (kSlices + 1) + slice);
      const unsigned b = a + static_cast<unsigned>(kSlices + 1);
      sphere_indices.insert(sphere_indices.end(), {a, b, a + 1, b, b + 1, a + 1});
    }
  }
  sphere_ = upload(sphere_vertices, sphere_indices);

  std::vector<float> cylinder_vertices;
  std::vector<unsigned> cylinder_indices;
  constexpr int kSides = 18;
  for (int side = 0; side <= kSides; ++side) {
    const float theta = static_cast<float>(side) / static_cast<float>(kSides) * 6.2831853f;
    const float x = std::cos(theta);
    const float y = std::sin(theta);
    pushVertex(cylinder_vertices, x, y, 0.0f, x, y, 0.0f);
    pushVertex(cylinder_vertices, x, y, 1.0f, x, y, 0.0f);
  }
  for (int side = 0; side < kSides; ++side) {
    const unsigned a = static_cast<unsigned>(side * 2);
    cylinder_indices.insert(cylinder_indices.end(), {a, a + 2, a + 1, a + 1, a + 2, a + 3});
  }
  const unsigned bottom_center = static_cast<unsigned>(cylinder_vertices.size() / 6);
  pushVertex(cylinder_vertices, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f);
  const unsigned top_center = bottom_center + 1;
  pushVertex(cylinder_vertices, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f);
  for (int side = 0; side <= kSides; ++side) {
    const float theta = static_cast<float>(side) / static_cast<float>(kSides) * 6.2831853f;
    pushVertex(cylinder_vertices, std::cos(theta), std::sin(theta), 0.0f, 0.0f, 0.0f, -1.0f);
    pushVertex(cylinder_vertices, std::cos(theta), std::sin(theta), 1.0f, 0.0f, 0.0f, 1.0f);
  }
  for (int side = 0; side < kSides; ++side) {
    const unsigned bottom = top_center + 1 + static_cast<unsigned>(side * 2);
    const unsigned top = bottom + 1;
    cylinder_indices.insert(cylinder_indices.end(), {bottom_center, bottom + 2, bottom, top_center, top, top + 2});
  }
  cylinder_ = upload(cylinder_vertices, cylinder_indices);

  std::vector<float> disc_vertices;
  std::vector<unsigned> disc_indices;
  pushVertex(disc_vertices, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  for (int side = 0; side <= kSides; ++side) {
    const float theta = static_cast<float>(side) / static_cast<float>(kSides) * 6.2831853f;
    pushVertex(disc_vertices, std::cos(theta), std::sin(theta), 0.0f, 0.0f, 0.0f, 1.0f);
  }
  for (int side = 0; side < kSides; ++side) {
    disc_indices.insert(disc_indices.end(), {0u, static_cast<unsigned>(side + 1), static_cast<unsigned>(side + 2)});
  }
  disc_ = upload(disc_vertices, disc_indices);

  glGenVertexArrays(1, &line_vao_);
  glGenBuffers(1, &line_vbo_);
  glBindVertexArray(line_vao_);
  glBindBuffer(kArrayBuffer, line_vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
  glBindVertexArray(0);
  return true;
}

void Renderer::shutdown() {}

Mesh Renderer::upload(const std::vector<float>& vertices, const std::vector<unsigned>& indices) const {
  Mesh mesh;
  mesh.index_count = static_cast<int>(indices.size());
  glGenVertexArrays(1, &mesh.vao);
  glGenBuffers(1, &mesh.vbo);
  glGenBuffers(1, &mesh.ebo);
  glBindVertexArray(mesh.vao);
  glBindBuffer(kArrayBuffer, mesh.vbo);
  glBufferData(kArrayBuffer, static_cast<GlSizeiptr>(vertices.size() * sizeof(float)), vertices.data(), kStaticDraw);
  glBindBuffer(kElementBuffer, mesh.ebo);
  glBufferData(kElementBuffer, static_cast<GlSizeiptr>(indices.size() * sizeof(unsigned)), indices.data(), kStaticDraw);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, reinterpret_cast<void*>(sizeof(float) * 3));
  glBindVertexArray(0);
  return mesh;
}

void Renderer::setViewport(int x, int y, int width, int height) { glViewport(x, y, width, height); }

void Renderer::setCamera(const glm::mat4& view, const glm::mat4& projection) {
  view_ = view;
  projection_ = projection;
}

void Renderer::drawMesh(const Mesh& mesh, const glm::mat4& model, const glm::vec3& color, bool lit) const {
  const glm::mat4 mvp = projection_ * view_ * model;
  glUseProgram(program_);
  glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, &mvp[0][0]);
  glUniformMatrix4fv(u_model_, 1, GL_FALSE, &model[0][0]);
  glUniform3f(u_color_, color.x, color.y, color.z);
  glUniform1f(u_lit_, lit ? 1.0f : 0.0f);
  glBindVertexArray(mesh.vao);
  glDrawElements(GL_TRIANGLES, mesh.index_count, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Renderer::drawLines(const std::vector<glm::vec3>& points, const glm::vec3& color, bool strip) const {
  if (points.size() < 2) return;
  glBindVertexArray(line_vao_);
  glBindBuffer(kArrayBuffer, line_vbo_);
  glBufferData(kArrayBuffer, static_cast<GlSizeiptr>(points.size() * sizeof(glm::vec3)), points.data(), kDynamicDraw);
  const glm::mat4 model(1.0f);
  const glm::mat4 mvp = projection_ * view_;
  glUseProgram(program_);
  glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, &mvp[0][0]);
  glUniformMatrix4fv(u_model_, 1, GL_FALSE, &model[0][0]);
  glUniform3f(u_color_, color.x, color.y, color.z);
  glUniform1f(u_lit_, 0.0f);
  glDrawArrays(strip ? GL_LINE_STRIP : GL_LINES, 0, static_cast<int>(points.size()));
  glBindVertexArray(0);
}

}  // namespace proc
