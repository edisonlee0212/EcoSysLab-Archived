#pragma once
#include "SorghumGrowthDescriptor.hpp"
#include <SorghumDescriptor.hpp>
#include <Vertex.hpp>

using namespace EvoEngine;
namespace EcoSysLab {
class PanicleData : public IPrivateComponent {
public:
  std::vector<Vertex> m_vertices;
  std::vector<glm::uvec3> m_triangles;
  void FormPanicle(const SorghumStatePair & sorghumStatePair);
  void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void OnDestroy() override;
  void Serialize(YAML::Emitter &out) override;
  void Deserialize(const YAML::Node &in) override;
};
} // namespace EcoSysLab