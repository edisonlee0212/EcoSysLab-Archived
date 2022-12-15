#pragma once

#include "PlantStructure.hpp"
#include "SoilModel.hpp"
#include "ClimateModel.hpp"
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
		float m_baseWaterRequirement = 0.0f;
		/*
		 * The desired water gain for reproduction (forming shoot/leaf/fruit) of this bud.
		 * Depending on apical control.
		 */
		float m_reproductionWaterRequirement = 0.0f;
		float m_adjustedReproductionWaterRequirement = 0.0f;

		float m_waterGain;
		glm::quat m_localRotation = glm::vec3(0.0f);
	};

	struct InternodeGrowthData {
		int m_age = 0;
		float m_inhibitorTarget = 0;
		float m_inhibitor = 0;
		glm::quat m_desiredLocalRotation = glm::vec3(0.0f);
		float m_sagging = 0;

		float m_maxDistanceToAnyBranchEnd = 0;
		int m_level = 0;
		int m_reverseLevel = 0;
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
		
		float m_reproductionWaterRequirement = 0.0f;

		float m_descendentReproductionWaterRequirement = 0.0f;
		float m_adjustedTotalReproductionWaterRequirement = 0.0f;
		float m_adjustedDescendentReproductionWaterRequirement = 0.0f;

		std::vector<glm::mat4> m_leaves;
		std::vector<glm::mat4> m_fruits;

		void Clear();
	};

	struct BranchGrowthData {
		int m_order = 0;

	};

	struct SkeletonGrowthData {

	};

	struct RootInternodeGrowthData {
		float m_maxDistanceToAnyBranchEnd = 0;
		float m_descendentTotalBiomass = 0;
		float m_biomass = 0;
		float m_extraMass = 0.0f;

		float m_rootDistance = 0;
		int m_rootUnitDistance = 0;
		int m_order = 0;
		
		float m_nitrateLevels;

		float m_auxinTarget = 0;
		float m_auxin = 0;

		float m_horizontalTropism;
		float m_verticalTropism;
	};
	struct RootBranchGrowthData {
		int m_order = 0;
	};

	struct RootSkeletonGrowthData {

	};

	class RootGrowthParameters {
	public:
		float m_growthRate = 0.5f;
		float m_rootNodeLength;
		glm::vec2 m_endNodeThicknessAndControl;
		float m_thicknessLengthAccumulate = 0.001f;
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

		float m_auxinTransportLoss = 1.0f;

		float m_tropismSwitchingProbability = 0.3f;
		float m_tropismSwitchingProbabilityDistanceFactor = 0.8f;
		float m_tropismIntensity = 0.3f;

		float m_baseBranchingProbability = 1.0f;
		float m_branchingProbabilityChildrenDecrease = 0.8f;
		float m_branchingProbabilityDistanceDecrease = 0.8f;
		[[nodiscard]] float GetGrowthRate() const;

		[[nodiscard]] float GetAuxinTransportLoss(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootNodeLength(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootApicalAngle(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootRollAngle(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootBranchingAngle(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetEndNodeThickness(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetThicknessControlFactor(const Node<RootInternodeGrowthData>& rootNode) const;
		[[nodiscard]] float GetThicknessAccumulateFactor(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetBranchingProbability(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetTropismIntensity(const Node<RootInternodeGrowthData>& rootNode) const;

		void SetTropisms(Node<RootInternodeGrowthData>& oldNode, Node<RootInternodeGrowthData>& newNode) const;

		RootGrowthParameters();
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

		glm::vec3 m_baseResourceRequirementFactor{};
		glm::vec3 m_productiveResourceRequirementFactor{};

		[[nodiscard]] int GetLateralBudCount(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetDesiredBranchingAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetDesiredRollAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetDesiredApicalAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetGravitropism(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetPhototropism(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetInternodeLength(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetGrowthRate(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetEndNodeThickness(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetThicknessControlFactor(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float
			GetLateralBudFlushingProbability(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetApicalControl(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetApicalDominanceBase(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetApicalDominanceDecrease(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetApicalBudKillProbability(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float
			GetLateralBudKillProbability(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] bool GetPruning(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetLowBranchPruning(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetSagging(const Node<InternodeGrowthData>& internode) const;


		[[nodiscard]] float
			GetShootBaseResourceRequirementFactor(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float
			GetLeafBaseResourceRequirementFactor(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float
			GetFruitBaseResourceRequirementFactor(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float
			GetShootProductiveResourceRequirementFactor(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float
			GetLeafProductiveResourceRequirementFactor(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float
			GetFruitProductiveResourceRequirementFactor(const Node<InternodeGrowthData>& internode) const;

		TreeGrowthParameters();
	};

	struct BranchGrowthNutrients {
		float m_waterRequirement = 0.0f;
		float m_water = 0.0f;
	};

	struct RootGrowthNutrients
	{
		float m_totalNitrate = 0.0f;
		float m_carbonRequirement = 0.0f;
		float m_carbon = 0.0f;
	};

	class TreeModel {
#pragma region Root Growth

		bool ElongateRoot(float extendLength, NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters, float& collectedAuxin);

		inline bool GrowRootNode(SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters);

		inline void CalculateResourceRequirement(NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters);
		inline void CalculateThickness(NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters);
#pragma endregion
#pragma region Tree Growth
		inline void CollectResourceRequirement(NodeHandle internodeHandle);

		inline void CalculateResourceRequirement(float &waterCollection, NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		inline void AdjustProductiveResourceRequirement(NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		bool LowBranchPruning(float maxDistance, NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		inline void CalculateThicknessAndSagging(NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		inline bool GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const TreeGrowthParameters& treeGrowthParameters);

		bool ElongateInternode(float extendLength, NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters, float& collectedInhibitor);

#pragma endregion

		void Initialize(const TreeGrowthParameters& treeGrowthParameters, const RootGrowthParameters& rootGrowthParameters);

		bool m_initialized = false;
		Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> m_branchSkeleton;
		Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData> m_rootSkeleton;
		std::deque<
			std::pair<Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData>,
			Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData>>> m_history;


		/**
		 * Grow one iteration of the branches, given the climate model and the procedural parameters.
		 * @param climateModel The climate model
		 * @param treeGrowthParameters The procedural parameters that guides the growth.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowBranches(ClimateModel& climateModel, const TreeGrowthParameters& treeGrowthParameters);

		/**
		 * Grow one iteration of the roots, given the soil model and the procedural parameters.
		 * @param soilModel The soil model
		 * @param rootGrowthParameters The procedural parameters that guides the growth.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowRoots(SoilModel& soilModel,
			const RootGrowthParameters& rootGrowthParameters);
	public:
		BranchGrowthNutrients m_branchGrowthNutrients;
		RootGrowthNutrients m_rootGrowthNutrients;
		glm::mat4 m_globalTransform = glm::translate(glm::vec3(0, 0, 0)) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(1.0f));
		glm::vec3 m_currentGravityDirection = glm::vec3(0, -1, 0);

		/**
		 * Erase the entire tree.
		 */
		void Clear();

		/**
		 * Grow one iteration of the tree, given the nutrients and the procedural parameters.
		 * @param soilModel The soil model
		 * @param climateModel The climate model
		 * @param rootGrowthParameters The procedural parameters that guides the growth of the roots.
		 * @param treeGrowthParameters The procedural parameters that guides the growth of the branches.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool Grow(SoilModel& soilModel, ClimateModel& climateModel,
			const RootGrowthParameters& rootGrowthParameters, const TreeGrowthParameters& treeGrowthParameters);

		int m_historyLimit = -1;


		[[nodiscard]] Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData>& RefBranchSkeleton();

		[[nodiscard]] const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData>&
			PeekBranchSkeleton(int iteration) const;

		[[nodiscard]] Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData>&
			RefRootSkeleton();


		[[nodiscard]] const Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData>&
			PeekRootSkeleton(int iteration) const;

		void ClearHistory();

		void Step();

		void Pop();

		[[nodiscard]] int CurrentIteration() const;

		void Reverse(int iteration);
	};
}