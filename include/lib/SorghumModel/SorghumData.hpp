#pragma once
#include "SorghumGrowthDescriptor.hpp"
#include <SorghumDescriptor.hpp>
using namespace EvoEngine;
namespace EcoSysLab {
enum class SorghumMode{
  SorghumGrowthDescriptor,
  SorghumDescriptor
};

class SorghumData : public IPrivateComponent {
  float m_currentTime = 1.0f;
  unsigned m_recordedVersion = 0;
  friend class SorghumLayer;
  bool m_segmentedMask = false;
public:
  int m_mode = (int)SorghumMode::SorghumGrowthDescriptor;
  glm::vec3 m_gravityDirection = glm::vec3(0, -1, 0);
  bool m_meshGenerated = false;
  AssetRef m_descriptor;
  int m_seed = 0;
  bool m_skeleton = false;
  

  void OnCreate() override;
  void OnDestroy() override;
  void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void SetTime(float time);
  void ExportModel(const std::string &filename,
                   const bool &includeFoliage = true) const;
  void Serialize(YAML::Emitter &out) override;
  void Deserialize(const YAML::Node &in) override;
  void CollectAssetRef(std::vector<AssetRef> &list) override;
  void FormPlant();
  void ApplyGeometry();

  void SetEnableSegmentedMask(bool value);
};
} // namespace PlantFactory
