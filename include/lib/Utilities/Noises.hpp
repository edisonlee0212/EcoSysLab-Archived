#pragma once

#include <glm/glm.hpp>
#include <vector>

using namespace evo_engine;

namespace eco_sys_lab {
enum class NoiseType {
  Constant,
  Linear,
  Simplex,
  Perlin,
};

struct NoiseDescriptor {
  unsigned m_type = 0;
  float m_frequency = 0.1f;
  float m_intensity = 1.0f;
  float m_multiplier = 1.0f;
  float m_min = -10;
  float m_max = 10;
  float m_offset = 0.0f;
  glm::vec3 m_shift = glm::vec3(0.0f);
  bool m_ridgid = false;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
class Noise2D {
 public:
  glm::vec2 m_minMax = glm::vec2(-1000, 1000);

  std::vector<NoiseDescriptor> m_noiseDescriptors;
  Noise2D();
  bool OnInspect();
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
  void RandomOffset(float min, float max);
  [[nodiscard]] float GetValue(const glm::vec2& position) const;
};

class Noise3D {
 public:
  glm::vec2 m_minMax = glm::vec2(-1000, 1000);
  std::vector<NoiseDescriptor> m_noiseDescriptors;
  Noise3D();
  bool OnInspect();
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);

  [[nodiscard]] float GetValue(const glm::vec3& position) const;
};
}  // namespace eco_sys_lab