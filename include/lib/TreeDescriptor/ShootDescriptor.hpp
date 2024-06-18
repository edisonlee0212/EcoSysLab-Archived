#pragma once
#include "Noises.hpp"
#include "ProceduralNoise.hpp"
#include "TreeModel.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class ShootDescriptor : public IAsset {
 public:
  /**
   * \brief The expected height gain for the tree for one year (max root distance).
   */
  float m_growthRate = 0.25f;
  float m_straightTrunk = 0.0f;

#pragma region Internode
  int m_baseInternodeCount = 1;
  glm::vec2 m_baseNodeApicalAngleMeanVariance = glm::vec2(0.0f);

  /**
   * \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
   */
  glm::vec2 m_branchingAngleMeanVariance = glm::vec2(45, 2);
  /**
   * \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
   */
  glm::vec2 m_rollAngleMeanVariance = glm::vec2(30, 2);
  /**
   * \brief The procedural noise of an angular difference orientation of lateral buds between two internodes
   */
  AssetRef m_rollAngle{};
  Noise2D m_rollAngleNoise2D{};
  /**
   * \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
   */
  glm::vec2 m_apicalAngleMeanVariance = glm::vec2(0, 3);
  /**
   * \brief The procedural noise of an angular difference orientation of lateral buds between two internodes
   */
  AssetRef m_apicalAngle{};
  Noise2D m_apicalAngleNoise2D{};
  /**
   * \brief The gravitropism.
   */
  float m_gravitropism = 0.0;
  /**
   * \brief The phototropism
   */
  float m_phototropism = 0.045f;
  /**
   * \brief The horizontal tropism
   */
  float m_horizontalTropism = 0.0f;

  float m_gravityBendingStrength = 0.f;
  float m_gravityBendingThicknessFactor = 1.f;
  float m_gravityBendingMax = 1.f;

  /**
   * \brief The internode length
   */
  float m_internodeLength = 0.03f;
  /*
   * \brief How the thickness of branch effect the length of the actual node.
   */
  float m_internodeLengthThicknessFactor = 0.15f;
  /**
   * \brief Thickness of end internode
   */
  float m_endNodeThickness = 0.004f;
  /**
   * \brief The thickness accumulation factor
   */
  float m_thicknessAccumulationFactor = 0.45f;
  /**
   * \brief The extra thickness gained from node length.
   */
  float m_thicknessAgeFactor = 0.0f;
  /**
   * \brief The shadow volume factor of the internode.
   */
  float m_internodeShadowFactor = 0.03f;

#pragma endregion
#pragma region Bud fate
  /**
   * \brief The number of lateral buds an internode contains
   */
  int m_lateralBudCount = 1;
  int m_maxOrder = -1;
  /**
   * \brief The probability of death of apical bud each year.
   */
  float m_apicalBudExtinctionRate = 0.0f;
  /**
   * \brief The probability of death of lateral bud each year.
   */
  float m_lateralBudFlushingRate = 0.5f;
  /**
   * \brief Apical control base
   */
  float m_apicalControl = 1.25f;
  /**
   * \brief Apical control base
   */
  float m_rootDistanceControl = 0.f;
  /**
   * \brief Apical control base
   */
  float m_heightControl = 0.f;

  /**
   * \brief How much inhibitor will an internode generate.
   */
  float m_apicalDominance = 0.25f;
  /**
   * \brief How much inhibitor will shrink when going through the branch.
   */
  float m_apicalDominanceLoss = 0.08f;

#pragma endregion
#pragma region Pruning
  bool m_trunkProtection = false;

  int m_maxFlowLength = 0;

  /**
   * \brief The pruning factor for branch because of absence of light
   */
  float m_lightPruningFactor = 0.0f;

  float m_branchStrength = 1.f;
  float m_branchStrengthThicknessFactor = 3.f;
  float m_branchStrengthLightingThreshold = 0.f;
  float m_branchStrengthLightingLoss = 0.f;
  float m_branchBreakingMultiplier = 1.f;
  float m_branchBreakingFactor = 1.f;
#pragma endregion

  AssetRef m_barkMaterial;
#pragma region Leaf
  float m_leafFlushingLightingRequirement = 0.1f;
  float m_leafFallProbability;
  float m_leafDistanceToBranchEndLimit;
#pragma endregion
#pragma region Fruit
  float m_fruitFlushingLightingRequirement = 0.1f;
  float m_fruitFallProbability;
  float m_fruitDistanceToBranchEndLimit;
#pragma endregion
  void PrepareController(ShootGrowthController& shootGrowthController) const;

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

}  // namespace eco_sys_lab