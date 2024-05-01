#pragma once
#include "Noises.hpp"
#include "TreeModel.hpp"
#include "ProceduralNoise.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class ShootDescriptor : public IAsset{
	public:
		/**
		 * \brief The expected height gain for the tree for one year (max root distance).
		 */
		float m_growthRate = 0.25f;
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
		AssetRef m_rollAngle {};
		Noise2D m_rollAngleNoise2D{};
		/**
		* \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_apicalAngleMeanVariance = glm::vec2(0, 3);
		/**
		* \brief The procedural noise of an angular difference orientation of lateral buds between two internodes
		*/
		AssetRef m_apicalAngle {};
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
		/**
		 * \brief The strength of gravity bending.
		 */
		glm::vec3 m_saggingFactorThicknessReductionMax = glm::vec3(0.0006f, 2.0f, 0.1f);
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
		* \brief The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruning = 0.15f;
		/**
		* \brief The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruningThicknessFactor = 0.0f;
		/**
		 * \brief The pruning factor for branch because of absence of light
		 */
		float m_lightPruningProbability = 20.0f;

		/**
		 * \brief The pruning factor for branch because of absence of light
		 */
		float m_lightPruningFactor = 0.0f;
		/**
		 * \brief The pruning factor for branch because of being too long
		 */
		float m_thicknessPruningFactor = 0.005f;

		float m_thicknessPruningProbability = 20.f;
#pragma endregion

		AssetRef m_barkMaterial;
#pragma region Leaf
		/**
		 * \brief The number of leaf buds an internode contains
		 */
		int m_leafBudCount;
		float m_leafGrowthRate = 0.05f;
		/**
		 * \brief Flushing prob of leaf bud related to the temperature.
		 */
		glm::vec4 m_leafBudFlushingProbabilityTemperatureRange;
		/**
		* \brief Base resource requirement factor for leaf
		*/
		float m_leafVigorRequirement;
		
		glm::vec3 m_maxLeafSize;
		float m_leafPositionVariance;
		float m_leafRotationVariance;
		float m_leafChlorophyllLoss;
		float m_leafChlorophyllSynthesisFactorTemperature;
		float m_leafFallProbability;

		float m_leafDistanceToBranchEndLimit;
#pragma endregion
#pragma region Fruit
		/**
		 * \brief The number of fruit buds an internode contains
		 */
		int m_fruitBudCount;
		float m_fruitGrowthRate = 0.05f;
		/**
		 * \brief Flushing prob of fruit bud related to the temperature.
		 */
		glm::vec4 m_fruitBudFlushingProbabilityTemperatureRange;

		/**
		* \brief Base resource requirement factor for fruit
		*/
		float m_fruitVigorRequirement;

		glm::vec3 m_maxFruitSize;
		float m_fruitPositionVariance;
		float m_fruitRotationVariance;

		float m_fruitFallProbability;
#pragma endregion
		void PrepareController(ShootGrowthController& shootGrowthController);

		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void CollectAssetRef(std::vector<AssetRef>& list) override;
	};


	
}