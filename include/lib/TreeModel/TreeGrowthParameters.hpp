#pragma once
#include "TreeModel.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{
	struct RootGrowthParameters {
		float m_rootNodeGrowthRate;

		/**
		 * \brief How much the soil density affects the growth;
		 */
		float m_environmentalFriction;
		/**
		 * \brief How much will the increase of soil density affects the growth;
		 */
		float m_environmentalFrictionFactor;
		/**
		 * \brief The root node length
		 */
		float m_rootNodeLength;

		/**
		 * \brief Thickness of end internode
		 */
		float m_endNodeThickness;
		/**
		 * \brief The thickness accumulation factor
		 */
		float m_thicknessAccumulationFactor;
		/**
		 * \brief The extra thickness gained from node length.
		 */
		float m_thicknessAccumulateAgeFactor;
		/**
		* The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		glm::vec2 m_branchingAngleMeanVariance;
		/**
		* The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_rollAngleMeanVariance;
		/**
		* The mean and variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		glm::vec2 m_apicalAngleMeanVariance;
		/**
		 * \brief Apical control base
		 */
		float m_apicalControl;
		/**
		 * \brief Age influence on apical control
		 */
		float m_apicalControlAgeFactor;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
		/**
		* \brief How much inhibitor will shrink when the tree ages.
		*/
		float m_apicalDominanceAgeFactor;
		/**
		* \brief How much inhibitor will shrink when going through the branch.
		*/
		float m_apicalDominanceDistanceFactor;
		/**
		* The possibility of the lateral branch having different tropism as the parent branch
		*/
		float m_tropismSwitchingProbability;
		/**
		* The distance factor of the possibility of the lateral branch having different tropism as the parent branch
		*/
		float m_tropismSwitchingProbabilityDistanceFactor;
		/**
		* The overall intensity of the tropism.
		*/
		float m_tropismIntensity;
		float m_rootNodeVigorRequirement = 1.0f;
		float m_vigorRequirementAggregateLoss = 1.0f;
		float m_branchingProbability = 1.0f;

		
	};

	struct FineRootParameters
	{
		int m_segmentSize = 2;
		float m_segmentLength = 0.02f;
		float m_apicalAngleVariance = 2.5f;
		float m_branchingAngle = 60.f;
		float m_thickness = 0.002f;
		float m_nodeThicknessRequirement = 0.05f;
	};

	struct ShootGrowthParameters {
		float m_internodeGrowthRate;
		float m_leafGrowthRate = 0.05f;
		float m_fruitGrowthRate = 0.05f;

#pragma region Bud
		/**
		 * \brief The number of lateral buds an internode contains
		 */
		int m_lateralBudCount;
		/**
		 * \brief The number of fruit buds an internode contains
		 */
		int m_fruitBudCount;
		/**
		 * \brief The number of leaf buds an internode contains
		 */
		int m_leafBudCount;
		/**
		* \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		glm::vec2 m_branchingAngleMeanVariance{};
		/**
		* \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_rollAngleMeanVariance{};
		/**
		* \brief The mean and variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		glm::vec2 m_apicalAngleMeanVariance{};
		/**
		 * \brief The gravitropism.
		 */
		float m_gravitropism;
		/**
		 * \brief The phototropism
		 */
		float m_phototropism;

		/**
		 * \brief Flushing prob of lateral bud related to the temperature.
		 */
		glm::vec4 m_lateralBudFlushingProbabilityTemperatureRange;
		/**
		 * \brief Flushing prob of leaf bud related to the temperature.
		 */
		glm::vec4 m_leafBudFlushingProbabilityTemperatureRange;
		/**
		 * \brief Flushing prob of fruit bud related to the temperature.
		 */
		glm::vec4 m_fruitBudFlushingProbabilityTemperatureRange;
		/**
		 * \brief The lighting factor for apical bud elongation rate.
		 */
		float m_apicalBudLightingFactor;
		/**
		 * \brief The lighting factor for lateral bud flushing probability.
		 */
		float m_lateralBudLightingFactor;
		/**
		 * \brief The lighting factor for leaf bud flushing probability.
		 */
		float m_leafBudLightingFactor;
		/**
		 * \brief The lighting factor for fruit bud flushing probability.
		 */
		float m_fruitBudLightingFactor;
		/**
		 * \brief Apical control base
		 */
		float m_apicalControl;
		/**
		 * \brief Age influence on apical control
		 */
		float m_apicalControlAgeFactor;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
		/**
		* \brief How much inhibitor will shrink when the tree ages.
		*/
		float m_apicalDominanceAgeFactor;
		/**
		* \brief How much inhibitor will shrink when going through the branch.
		*/
		float m_apicalDominanceDistanceFactor;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_apicalBudExtinctionRate;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_lateralBudExtinctionRate;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_leafBudExtinctionRate;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_fruitBudExtinctionRate;

		/**
		* \brief Productive resource requirement factor for internode elongation
		*/
		float m_internodeVigorRequirement;
		/**
		* \brief Base resource requirement factor for leaf
		*/
		float m_leafVigorRequirement;
		/**
		* \brief Base resource requirement factor for fruit
		*/
		float m_fruitVigorRequirement;

		float m_vigorRequirementAggregateLoss = 1.0f;
#pragma endregion
#pragma region Internode
		/**
		 * \brief The internode length
		 */
		float m_internodeLength;

		/**
		 * \brief Thickness of end internode
		 */
		float m_endNodeThickness;
		/**
		 * \brief The thickness accumulation factor
		 */
		float m_thicknessAccumulationFactor;
		/**
		 * \brief The extra thickness gained from node length.
		 */
		float m_thicknessAccumulateAgeFactor;
		/**
		* \brief The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruning;
		/**
		 * \brief The The impact of the amount of incoming light on the shedding of end internodes.
		 */
		float m_endNodePruningLightFactor;
		/**
		 * \brief The strength of gravity bending.
		 */
		glm::vec3 m_saggingFactorThicknessReductionMax = glm::vec3(0.8f, 1.75f, 1.0f);


#pragma endregion

#pragma region Leaf

		glm::vec3 m_maxLeafSize;
		float m_leafPositionVariance;
		float m_leafRotationVariance;
		float m_leafChlorophyllLoss;
		float m_leafChlorophyllSynthesisFactorTemperature;
		float m_leafFallProbability;

		float m_leafDistanceToBranchEndLimit;
#pragma endregion
#pragma region Fruit

		glm::vec3 m_maxFruitSize;
		float m_fruitPositionVariance;
		float m_fruitRotationVariance;

		float m_fruitFallProbability;
#pragma endregion
	};


	struct TwigParameters
	{
		int m_segmentSize = 1;
		float m_segmentLength = 0.04f;
		float m_apicalAngleVariance = 0.0f;
		float m_branchingAngle = 30.f;
		float m_thickness = 0.002f;
		float m_nodeThicknessRequirement = 0.05f;
	};
}