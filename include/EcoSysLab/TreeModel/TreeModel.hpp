#pragma once

#include "PlantStructure.hpp"
#include "SoilModel.hpp"
#include "ClimateModel.hpp"
#include "Octree.hpp"
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
		Died,
		Removed
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

		//-1.0 means the no fruit.
		float m_maturity = -1.0f;
		float m_drought = 0.0f;

		float m_chlorophyll = 0.0f;

		float m_luminousFlux = 0.0f;

		glm::mat4 m_reproductiveModuleTransform = glm::mat4(0.0f);
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

		float m_reproductiveWaterRequirement;

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
		[[nodiscard]] float GetExpectedGrowthRate() const;

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
		 * \brief The internode length
		 */
		float m_internodeLength;
		/**
		 * \brief The growth rate
		 */
		float m_growthRate;
		/**
		 * \brief Thickness of internode
		 */
		float m_endNodeThickness;
		float m_thicknessAccumulateFactor;
		/**
		 * \brief Flushing prob of lateral bud
		 */
		float m_lateralBudFlushingProbability;
		/**
		 * \brief Flushing prob of leaf bud
		 */
		glm::vec4 m_leafBudFlushingProbabilityTemperatureRange;
		/**
		 * \brief Flushing prob of fruit bud
		 */
		glm::vec4 m_fruitBudFlushingProbabilityTemperatureRange;

		/**
		 * \brief AC
		 */
		glm::vec2 m_apicalControlBaseDistFactor{};

		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
		float m_apicalDominanceDistanceFactor;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_budKillProbability;
		/**
		* \brief The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruning;
		/**
		 * \brief The strength of gravity bending.
		 */
		glm::vec3 m_saggingFactorThicknessReductionMax = glm::vec3(0.8f, 1.75f, 1.0f);
		/**
		* \brief Base resource requirement factor for internode
		*/
		float m_shootBaseWaterRequirement;
		/**
		* \brief Base resource requirement factor for leaf
		*/
		float m_leafBaseWaterRequirement;
		/**
		* \brief Base resource requirement factor for fruit
		*/
		float m_fruitBaseWaterRequirement;
		/**
		* \brief Productive resource requirement factor for internode elongation
		*/
		float m_shootProductiveWaterRequirement;
		/**
		* \brief Productive resource requirement factor for leaf to grow larger, later to generate carbohydrates
		*/
		float m_leafProductiveWaterRequirement;
		/**
		* \brief Productive resource requirement factor for fruit to grow larger
		*/
		float m_fruitProductiveWaterRequirement;

#pragma region Foliage

		glm::vec3 m_maxLeafSize;
		float m_leafPositionVariance;
		float m_leafRandomRotation;
		float m_leafChlorophyllLoss;
		float m_leafChlorophyllSynthesisFactorTemperature;
		float m_leafFallProbability;

		float m_leafDistanceToBranchEndLimit;
#pragma endregion
#pragma region Fruit

		glm::vec3 m_maxFruitSize;
		float m_fruitPositionVariance;
		float m_fruitRandomRotation;

#pragma endregion
		float m_trunkRadius;


		[[nodiscard]] float GetDesiredBranchingAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetDesiredRollAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetDesiredApicalAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetApicalControl(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetSagging(const Node<InternodeGrowthData>& internode) const;


		TreeGrowthParameters();
	};

	struct TreeGrowthNutrients {
		float m_water = 0.0f;
		float m_luminousFlux = 0.0f;
		float m_carbohydrate = 0.0f;
	};

	struct TreeVoxelData
	{
		NodeHandle m_nodeHandle = -1;
		NodeHandle m_flowHandle = -1;
		unsigned m_referenceCount = 0;
	};

	class TreeModel {
#pragma region Root Growth
		
		bool ElongateRoot(const glm::mat4& globalTransform, SoilModel& soilModel, float extendLength, NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters, float& collectedAuxin);

		inline bool GrowRootNode(const glm::mat4& globalTransform, SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters);

		inline void CalculateResourceRequirement(NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters);
		inline void CalculateThickness(NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters);

		void CollectWaterFromRoots(const glm::mat4& globalTransform, SoilModel& soilModel,
			const RootGrowthParameters& rootGrowthParameters);
#pragma endregion
#pragma region Tree Growth
		inline void CollectResourceRequirement(NodeHandle internodeHandle);

		inline void CalculateResourceRequirement(NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement);

		inline void AdjustProductiveResourceRequirement(NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		bool LowBranchPruning(float maxDistance, NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		inline void CalculateThicknessAndSagging(NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		inline bool GrowInternode(const glm::mat4& globalTransform, ClimateModel& climateModel, NodeHandle internodeHandle, const TreeGrowthParameters& treeGrowthParameters);

		bool ElongateInternode(const glm::mat4& globalTransform, float extendLength, NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters, float& collectedInhibitor);
		void CollectLuminousFluxFromLeaves(ClimateModel& climateModel,
			const TreeGrowthParameters& treeGrowthParameters);
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
		 * @param newTreeGrowthNutrientsRequirement
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowBranches(const glm::mat4& globalTransform, ClimateModel& climateModel, const TreeGrowthParameters& treeGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement);

		/**
		 * Grow one iteration of the roots, given the soil model and the procedural parameters.
		 * @param soilModel The soil model
		 * @param rootGrowthParameters The procedural parameters that guides the growth.
		 * @param newTreeGrowthNutrientsRequirement
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowRoots(const glm::mat4& globalTransform, SoilModel& soilModel,
			const RootGrowthParameters& rootGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement);


		int m_leafCount = 0;
		int m_fruitCount = 0;
	public:
		Octree<TreeVoxelData> m_rootOctree;
		Octree<TreeVoxelData> m_branchOctree;
		bool m_enableRootCollisionDetection = false;
		bool m_enableBranchCollisionDetection = false;

		TreeGrowthNutrients m_treeGrowthNutrientsRequirement;
		TreeGrowthNutrients m_treeGrowthNutrients;
		float m_globalGrowthRate = 0.0f;
		glm::vec3 m_currentGravityDirection = glm::vec3(0, -1, 0);

		/**
		 * Erase the entire tree.
		 */
		void Clear();

		[[nodiscard]] int GetLeafCount() const;
		[[nodiscard]] int GetFruitCount() const;

		/**
		 * Grow one iteration of the tree, given the nutrients and the procedural parameters.
		 * @param globalTransform The global transform of tree in world space.
		 * @param soilModel The soil model
		 * @param climateModel The climate model
		 * @param rootGrowthParameters The procedural parameters that guides the growth of the roots.
		 * @param treeGrowthParameters The procedural parameters that guides the growth of the branches.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool Grow(const glm::mat4& globalTransform, SoilModel& soilModel, ClimateModel& climateModel,
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