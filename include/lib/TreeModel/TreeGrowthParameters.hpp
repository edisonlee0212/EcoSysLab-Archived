#pragma once
#include "TreeModel.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{
	struct ShootGrowthParameters {
		/**
		 * \brief The expected height gain for the tree for one year (max root distance).
		 */
		float m_growthRate = 0.3;
#pragma region Internode
		/**
		* \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		glm::vec2 m_branchingAngleMeanVariance{};
		/**
		* \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_rollAngleMeanVariance{};
		/**
		* \brief The variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		float m_apicalAngleVariance{};
		/**
		 * \brief The gravitropism.
		 */
		float m_gravitropism;
		/**
		 * \brief The phototropism
		 */
		float m_phototropism;
		/**
		 * \brief The strength of gravity bending.
		 */
		glm::vec3 m_saggingFactorThicknessReductionMax = glm::vec3(0.8f, 1.75f, 1.0f);
		/**
		 * \brief The internode length
		 */
		float m_internodeLength;
		/*
		 * \brief How the thickness of branch effect the length of the actual node.
		 */
		float m_internodeLengthThicknessFactor = 0.25f;
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
		float m_lateralBudFlushingRate = 1.0f;
		/**
		 * \brief Flushing prob of apical bud related to the light intensity.
		 */
		float m_apicalBudLightingFactor = 1.0f;
		/**
		 * \brief Flushing prob of lateral bud related to the light intensity.
		 */
		float m_lateralBudLightingFactor = 1.0f;
		/**
		 * \brief Flushing prob of apical bud related to the space availability.
		 */
		float m_apicalBudSpaceFactor = 1.0f;
		/**
		 * \brief Flushing prob of lateral bud related to the space availability.
		 */
		float m_lateralBudSpaceFactor = 1.0f;
		/**
		 * \brief Apical control base
		 */
		float m_apicalControl;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
		/**
		* \brief How much inhibitor will shrink when going through the branch.
		*/
		float m_apicalDominanceLoss;

#pragma endregion
#pragma region Pruning
		/**
		* \brief The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruning;
		/**
		* \brief The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruningThicknessFactor = 0.0f;
		/**
		 * \brief The pruning factor for branch because of absence of light
		 */
		float m_lightPruningFactor = 0.0f;
		/**
		 * \brief The pruning factor for branch because of being too long
		 */
		float m_thicknessPruningFactor = 0.005f;
#pragma endregion
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
	};


	
}