#pragma once
#include "TreeGrowthData.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{
	struct ShootGrowthController {
		bool m_branchPush = false;
		bool m_useLevelForApicalControl = true;
#pragma region Internode
		/**
		 * \brief The expected elongation length for an internode for one year.
		 */
		float m_internodeGrowthRate;
		/**
		* \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		std::function<float(const Node<InternodeGrowthData>& internode)> m_branchingAngle;
		/**
		* \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		std::function<float(const Node<InternodeGrowthData>& internode)> m_rollAngle;
		/**
		* \brief The mean and variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		std::function<float(const Node<InternodeGrowthData>& internode)> m_apicalAngle;
		/**
		 * \brief The gravitropism.
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_gravitropism;
		/**
		 * \brief The phototropism
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_phototropism;
		/**
		 * \brief The strength of gravity bending.
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_sagging;

		/**
		 * \brief The internode length
		 */
		float m_internodeLength;
		/*
		 * \brief How the thickness of branch effect the length of the actual node.
		 */
		float m_internodeLengthThicknessFactor = 0.0f;
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
		float m_thicknessAgeFactor;
		/**
		 * \brief The shadow volume factor of the internode.
		 */
		float m_internodeShadowFactor = 1.f;
#pragma endregion
#pragma region Bud
		/**
		 * \brief The number of lateral buds an internode contains
		 */
		int m_lateralBudCount;
		/**
		 * \brief Extinction rate of apical bud.
		 */
		std::function<void(const Node<InternodeGrowthData>& internode, Bud& targetBud)> m_budExtinctionRate;

		/**
		 * \brief Flushing rate of a bud.
		 */
		std::function<void(const Node<InternodeGrowthData>& internode, Bud& targetBud)> m_budFlushingRate;
		/**
		 * \brief Apical control base
		 */
		float m_apicalControl;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		std::function<float(const Node<InternodeGrowthData>& internode)> m_apicalDominance;
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
		float m_lowBranchPruningThicknessFactor;
		/**
		 * \brief The The impact of the amount of incoming light on the shedding of end internodes.
		 */
		std::function<float(float deltaTime, const Node<InternodeGrowthData>& internode)> m_pruningFactor;
#pragma endregion
#pragma region Leaf
		/**
		* \brief Base resource requirement factor for leaf
		*/
		float m_leafVigorRequirement;
		
		/**
		 * \brief Flushing prob of leaf bud.
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_leafBudFlushingProbability;
		
		/**
		 * \brief The number of leaf buds an internode contains
		 */
		int m_leafBudCount;

		float m_leafGrowthRate = 0.05f;
		
		/**
		 * \brief The size of the leaf when it reaches full maturity.
		 */
		glm::vec3 m_maxLeafSize;
		/**
		 * \brief The relative distance variance between leaf and bud.
		 */
		float m_leafPositionVariance;
		/**
		 * \brief The rotation variance between leaf and bud.
		 */
		float m_leafRotationVariance;
		/**
		 * \brief The damage to the leaf during this iteration caused by various factors
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_leafDamage;
		/**
		 * \brief The probability of leaf falling after health return to 0.0
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_leafFallProbability;
		float m_leafShadowVolume = 0.05f;
#pragma endregion
#pragma region Fruit
		/**
		* \brief Base resource requirement factor for fruit
		*/
		float m_fruitVigorRequirement;
		/**
		 * \brief Flushing prob of fruit bud.
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_fruitBudFlushingProbability;
		/**
		 * \brief The number of fruit buds an internode contains
		 */
		int m_fruitBudCount;

		float m_fruitGrowthRate = 0.05f;
		/**
		 * \brief The size of the fruit when it reaches full maturity.
		 */
		glm::vec3 m_maxFruitSize;
		/**
		 * \brief The position variance between fruit and bud.
		 */
		float m_fruitPositionVariance;
		/**
		 * \brief The rotation variance between fruit and bud.
		 */
		float m_fruitRotationVariance;
		/**
		 * \brief The damage to the fruit during this iteration caused by various factors
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_fruitDamage;
		/**
		 * \brief The probability of fruit falling after health return to 0.0
		 */
		std::function<float(const Node<InternodeGrowthData>& internode)> m_fruitFallProbability;
#pragma endregion
	};
}