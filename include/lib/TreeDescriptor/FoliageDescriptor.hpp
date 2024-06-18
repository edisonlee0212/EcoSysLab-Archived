#pragma once
#include "Skeleton.hpp"

using namespace evo_engine;

namespace eco_sys_lab {


class FoliageDescriptor : public IAsset {
 public:
  glm::vec2 m_leafSize = glm::vec2(0.02f, 0.04f);
  int m_leafCountPerInternode = 5;
  float m_positionVariance = 0.175f;
  float m_rotationVariance = 10.f;
  float m_branchingAngle = 30.f;
  float m_maxNodeThickness = 1.0f;
  float m_minRootDistance = 0.0f;
  float m_maxEndDistance = 0.2f;

  float m_horizontalTropism = 0.f;
  float m_gravitropism = 0.f;

  AssetRef m_leafMaterial;
  AssetRef m_twigMaterial;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internodeInfo,
                               const float treeSize) const;
};

}  // namespace eco_sys_lab