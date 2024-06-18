#pragma once
#include "ShootDescriptor.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreeVisualizer.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
enum class ShootGrowthParameterType {
  GrowthRate,
  BranchingAngleMean,
  BranchingAngleVariance,
  RollAngleMean,
  RollAngleVariance,
  ApicalAngleMean,
  ApicalAngleVariance,
  Gravitropism,
  Phototropism,
  Sagging,
  SaggingThicknessFactor,
  MaxSagging,
  InternodeLength,
  InternodeLengthThicknessFactor,
  EndNodeThickness,
  ThicknessAccumulationFactor,
  ThicknessAgeFactor,
  ShadowFactor,

  ApicalBudExtinctionRate,
  LateralBudExtinctionRate,
  ApicalBudLightingFactor,
  LateralBudLightingFactor,
  ApicalControl,
  ApicalDominance,
  ApicalDominanceLoss,

  LowBranchPruning,
  LowBranchPruningThicknessFactor,

};

struct ShootGrowthParameterOffset {
  unsigned m_type = static_cast<unsigned>(ShootGrowthParameterType::GrowthRate);
  glm::vec2 m_range = {0.0f, 0.0f};
  Curve2D m_offset;
};

class ShootDescriptorGenerator : public IAsset {
  std::vector<ShootGrowthParameterOffset> m_shootDescriptorOffsets;

 public:
  AssetRef m_baseShootDescriptor{};
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  [[nodiscard]] std::shared_ptr<ShootDescriptor> Generate();
  bool OnInspectShootGrowthParametersOffset(std::vector<ShootGrowthParameterOffset>& shoot_growth_parameter_offsets);
};
}  // namespace eco_sys_lab