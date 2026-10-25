#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include <string>
#include <vector>

namespace proc {

struct Mesh {
  unsigned vao = 0;
  unsigned vbo = 0;
  unsigned ebo = 0;
  int index_count = 0;
};

class Renderer {
 public:
  bool init(std::string& error);
  void shutdown();

  void setViewport(int x, int y, int width, int height);
  void setCamera(const glm::mat4& view, const glm::mat4& projection);
  void drawMesh(const Mesh& mesh, const glm::mat4& model, const glm::vec3& color, bool lit) const;
  void drawLines(const std::vector<glm::vec3>& points, const glm::vec3& color, bool strip) const;

  const Mesh& sphere() const { return sphere_; }
  const Mesh& cylinder() const { return cylinder_; }
  const Mesh& disc() const { return disc_; }

 private:
  unsigned program_ = 0;
  int u_mvp_ = -1;
  int u_model_ = -1;
  int u_color_ = -1;
  int u_lit_ = -1;
  glm::mat4 view_{1.0f};
  glm::mat4 projection_{1.0f};
  Mesh sphere_;
  Mesh cylinder_;
  Mesh disc_;
  unsigned line_vao_ = 0;
  unsigned line_vbo_ = 0;

  Mesh upload(const std::vector<float>& vertices, const std::vector<unsigned>& indices) const;
};

}  // namespace proc
