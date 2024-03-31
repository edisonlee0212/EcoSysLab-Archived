#pragma once

#include "Plot2D.hpp"
#include "SorghumGrowthDescriptor.hpp"
#include "SorghumState.hpp"
using namespace EvoEngine;
namespace EcoSysLab {

class SorghumDescriptor : public IAsset {
public:
  //Panicle
  SingleDistribution<glm::vec2> m_panicleSize;
  SingleDistribution<float> m_panicleSeedAmount;
  SingleDistribution<float> m_panicleSeedRadius;
  //Stem
  SingleDistribution<float> m_stemTiltAngle;
  SingleDistribution<float> m_stemLength;
  SingleDistribution<float> m_stemWidth;
  //Leaf
  SingleDistribution<float> m_leafAmount;

  PlottedDistribution<float> m_leafStartingPoint;
  PlottedDistribution<float> m_leafCurling;
  PlottedDistribution<float> m_leafRollAngle;
  PlottedDistribution<float> m_leafBranchingAngle;
  PlottedDistribution<float> m_leafBending;
  PlottedDistribution<float> m_leafBendingAcceleration;
  PlottedDistribution<float> m_leafBendingSmoothness;
  PlottedDistribution<float> m_leafWaviness;
  PlottedDistribution<float> m_leafWavinessFrequency;
  PlottedDistribution<float> m_leafPeriodStart;
  PlottedDistribution<float> m_leafLength;
  PlottedDistribution<float> m_leafWidth;

  //Finer control
  Curve2D m_widthAlongStem;
  Curve2D m_curlingAlongLeaf;
  Curve2D m_widthAlongLeaf;
  Curve2D m_wavinessAlongLeaf;
  void OnCreate() override;
  void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void Serialize(YAML::Emitter &out) override;
  void Deserialize(const YAML::Node &in) override;

  [[nodiscard]] Entity CreateEntity(unsigned int seed = 0) const;
  void Apply(const std::shared_ptr<SorghumState>& targetState, unsigned int seed = 0) const;
};
} // namespace EcoSysLab