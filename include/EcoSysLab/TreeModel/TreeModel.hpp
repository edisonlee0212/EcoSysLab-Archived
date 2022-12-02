#pragma once

#include "PlantStructure.hpp"

using namespace UniEngine;
namespace EcoSysLab {
	enum class BudType {
		Apical,
		Lateral,
		Leaf,
		Fruit
	};

	enum class BudStatus {
		Dormant,
		Flushed,
		Died
	};

	class Bud {
	public:
		BudType m_type = BudType::Apical;
		BudStatus m_status = BudStatus::Dormant;

		/*
		 * The desired water gain for maintaining current plant structure.
		 * Depending on the size of fruit/leaf.
		 */
		float m_baseResourceRequirement = 0.0f;
		/*
		 * The desired water gain for reproduction (forming shoot/leaf/fruit) of this bud.
		 * Depending on apical control.
		 */
		float m_productiveResourceRequirement = 0.0f;
		float m_adjustedProductiveResourceRequirement = 0.0f;
		glm::quat m_localRotation = glm::vec3(0.0f);
	};

	struct InternodeGrowthData {
		int m_age = 0;
		float m_inhibitorTarget = 0;
		float m_inhibitor = 0;
		glm::quat m_desiredLocalRotation = glm::vec3(0.0f);
		float m_sagging = 0;

		float m_maxDistanceToAnyBranchEnd = 0;
		float m_level = 0;
		int m_order = 0;
		float m_descendentTotalBiomass = 0;
		float m_biomass = 0;
		float m_extraMass = 0.0f;
		float m_rootDistance = 0;

		glm::vec3 m_lightDirection = glm::vec3(0, 1, 0);
		float m_lightIntensity = 1.0f;

		/**
		 * List of buds, first one will always be the apical bud which points forward.
		 */
		std::vector<Bud> m_buds;

		float m_productiveResourceRequirement = 0.0f;
		float m_descendentProductiveResourceRequirement = 0.0f;
		float m_adjustedTotalProductiveWaterRequirement = 0.0f;
		float m_adjustedProductiveResourceRequirement = 0.0f;
		float m_adjustedDescendentProductiveResourceRequirement = 0.0f;

		std::vector<glm::mat4> m_leaves;
		std::vector<glm::mat4> m_fruits;

		void Clear();
	};

	struct BranchGrowthData {
		int m_order = 0;

	};

	struct SkeletonGrowthData {

	};

	enum class RootType {
		Tap,
		Lateral,
		Heart
	};
	struct RootInternodeGrowthData {
		float m_rootDistance = 0;
		int m_order = 0;
		RootType m_rootType;
	};
	struct RootBranchGrowthData {
		int m_order = 0;
	};

	struct RootSkeletonGrowthData {

	};

	class RootGrowthParameters {
	public:
		float m_growthRate;
		float m_gravitropism;
		float m_rootNodeLength;
		glm::vec2 m_endNodeThicknessAndControl;
		/**
		* The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		glm::vec2 m_branchingAngleMeanVariance{};
		/**
		* The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_rollAngleMeanVariance{};
		/**
		* The mean and variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		glm::vec2 m_apicalAngleMeanVariance{};

		[[nodiscard]] float GetGrowthRate(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetRootNodeLength(const Node<RootInternodeGrowthData> &rootNode) const;

		[[nodiscard]] float GetRootApicalAngle(const Node<RootInternodeGrowthData> &rootNode) const;

		[[nodiscard]] float GetRootRollAngle(const Node<RootInternodeGrowthData> &rootNode) const;

		[[nodiscard]] float GetRootBranchingAngle(const Node<RootInternodeGrowthData> &rootNode) const;

		[[nodiscard]] float GetEndNodeThickness(const Node<RootInternodeGrowthData> &internode) const;

		[[nodiscard]] float GetThicknessControlFactor(const Node<RootInternodeGrowthData> &internode) const;
	};

	class TreeGrowthParameters {
	public:
		int m_lateralBudCount;
		/**
		* The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		glm::vec2 m_branchingAngleMeanVariance{};
		/**
		* The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_rollAngleMeanVariance{};
		/**
		* The mean and variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		glm::vec2 m_apicalAngleMeanVariance{};

		float m_gravitropism;
		float m_phototropism;
		float m_internodeLength;
		float m_growthRate;
		glm::vec2 m_endNodeThicknessAndControl{};
		float m_lateralBudFlushingProbability;
		/*
		 * To form significant trunk. Larger than 1 means forming big trunk.
		 */
		glm::vec2 m_apicalControlBaseDistFactor{};

		/**
		* How much inhibitor will an internode generate.
		*/
		glm::vec3 m_apicalDominanceBaseAgeDist{};

		glm::vec2 m_budKillProbabilityApicalLateral{};
		/**
		* The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruning;
		/**
		 * The strength of gravity bending.
		 */
		glm::vec3 m_saggingFactorThicknessReductionMax = glm::vec3(0.8f, 1.75f, 1.0f);

		glm::vec3 m_baseResourceRequirementFactor;
		glm::vec3 m_productiveResourceRequirementFactor;

		[[nodiscard]] int GetLateralBudCount(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetDesiredBranchingAngle(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetDesiredRollAngle(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetDesiredApicalAngle(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetGravitropism(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetPhototropism(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetInternodeLength(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetGrowthRate(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetEndNodeThickness(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetThicknessControlFactor(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float
		GetLateralBudFlushingProbability(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetApicalControl(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetApicalDominanceBase(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetApicalDominanceDecrease(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetApicalBudKillProbability(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float
		GetLateralBudKillProbability(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] bool GetPruning(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetLowBranchPruning(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float GetSagging(const Node<InternodeGrowthData> &internode) const;


		[[nodiscard]] float
		GetShootBaseResourceRequirementFactor(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float
		GetLeafBaseResourceRequirementFactor(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float
		GetFruitBaseResourceRequirementFactor(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float
		GetShootProductiveResourceRequirementFactor(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float
		GetLeafProductiveResourceRequirementFactor(const Node<InternodeGrowthData> &internode) const;

		[[nodiscard]] float
		GetFruitProductiveResourceRequirementFactor(const Node<InternodeGrowthData> &internode) const;

		TreeGrowthParameters();
	};

	struct GrowthNutrients {
		float m_water = 0.0f;
	};

	class TreeModel {
#pragma region Root Growth

		bool GrowShoots(float extendLength, NodeHandle rootNodeHandle,
						const RootGrowthParameters &parameters, float &collectedInhibitor);

#pragma endregion
#pragma region Tree Growth

		bool LowBranchPruning(float maxDistance, NodeHandle internodeHandle,
							  const TreeGrowthParameters &parameters);

		inline void CalculateSagging(NodeHandle internodeHandle,
									 const TreeGrowthParameters &parameters);

		inline void CalculateResourceRequirement(NodeHandle internodeHandle,
												 const TreeGrowthParameters &parameters);

		inline bool GrowInternode(NodeHandle internodeHandle, const TreeGrowthParameters &parameters,
								  const GrowthNutrients &growthNutrients);

		bool GrowShoots(float extendLength, NodeHandle internodeHandle,
						const TreeGrowthParameters &parameters, float &collectedInhibitor);

#pragma endregion

		void Initialize(const TreeGrowthParameters &treeGrowthParameters, const RootGrowthParameters &rootParameters);

		bool m_initialized = false;
		Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> m_branchSkeleton;
		Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData> m_rootSkeleton;
		std::deque<
				std::pair<Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData>,
						Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData>>> m_history;

	public:
		glm::vec3 m_gravityDirection = glm::vec3(0, -1, 0);

		/**
		 * Erase the entire tree.
		 */
		void Clear();

		/**
		 * Grow one iteration of the tree, given the nutrients and the procedural parameters.
		 * @param growthNutrients The nutrients from the root (water, etc.)
		 * @param treeGrowthParameters The procedural parameters that guides the growth.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool Grow(const GrowthNutrients &growthNutrients, const TreeGrowthParameters &treeGrowthParameters,
				  const RootGrowthParameters &rootGrowthParameters);


		int m_historyLimit = -1;

		[[nodiscard]] Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &RefBranchSkeleton();

		[[nodiscard]] const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &
		PeekBranchSkeleton(int iteration) const;

		[[nodiscard]] Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData> &
		RefRootSkeleton();

		[[nodiscard]] const Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData> &
		PeekRootSkeleton(int iteration) const;

		void ClearHistory();

		void Step();

		void Pop();

		[[nodiscard]] int CurrentIteration() const;

		void Reverse(int iteration);
	};
}