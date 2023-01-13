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
		 * The desired resource needed for maintaining current plant structure.
		 * Depending on the size of fruit/leaf.
		 */
		float m_maintenanceResourceRequirement = 0.0f;
		/*
		 * The desired resource needed for reproduction (forming shoot/leaf/fruit) of this bud.
		 * Depending on the size of fruit/leaf.
		 * Adjusted by apical control.
		 */
		float m_developmentResourceRequirement = 0.0f;
		/*
		 * The allocated total resource for maintenance and development of this module.
		 */
		float m_allocatedResource;

		glm::quat m_localRotation = glm::vec3(0.0f);

		//-1.0 means the no fruit.
		float m_maturity = -1.0f;
		float m_drought = 0.0f;

		float m_chlorophyll = 0.0f;

		float m_luminousFlux = 0.0f;
		glm::mat4 m_reproductiveModuleTransform = glm::mat4(0.0f);
	};

	struct InternodeGrowthData {
		bool m_lateral = false;
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

		float m_temperature;

		glm::vec3 m_lightDirection = glm::vec3(0, 1, 0);
		float m_lightIntensity = 1.0f;

		/**
		 * List of buds, first one will always be the apical bud which points forward.
		 */
		std::vector<Bud> m_buds;
		/*
		 * The desired resource needed for maintaining current plant structure.
		 * Depending on the volume of internode.
		 */
		float m_maintenanceResourceRequirement = 0.0f;
		/*
		 * Sum of buds' development resource requirement and internode's development resource requirement.
		 */
		float m_developmentResourceRequirement = 0.0f;
		/*
		 * Sum of all child node's development resource requirement;
		 */
		float m_subtreeDevelopmentResourceRequirement = 0.0f;
		/*
		 * The allocated total resource for maintenance and development of this module.
		 */
		float m_allocatedResource;
		std::vector<glm::mat4> m_leaves;
		std::vector<glm::mat4> m_fruits;

	};

	struct BranchGrowthData {
		int m_order = 0;

	};

	struct SkeletonGrowthData {

	};

	struct RootInternodeGrowthData {
		bool m_lateral = false;
		float m_soilDensity = 0.0f;
		int m_age = 0;
		float m_maxDistanceToAnyBranchEnd = 0;
		float m_descendentTotalBiomass = 0;
		float m_biomass = 0;
		float m_extraMass = 0.0f;

		float m_rootDistance = 0;
		int m_rootUnitDistance = 0;
		int m_order = 0;

		float m_nitrite = 1.0f;
		
		float m_inhibitorTarget = 0;
		float m_inhibitor = 0;

		float m_horizontalTropism;
		float m_verticalTropism;
		/*
		 * The desired resource needed for maintaining current plant structure.
		 * Depending on the volume of root node.
		 */
		float m_maintenanceResourceRequirement = 0.0f;
		/*
		 * The root node's development resource requirement.
		 */
		float m_developmentResourceRequirement = 0.0f;
		/*
		 * Sum of all child node's development resource requirement;
		 */
		float m_subtreeDevelopmentResourceRequirement = 0.0f;
		/*
		 * The allocated total resource for maintenance and development of this module.
		 */
		float m_allocatedResource;

	};
	struct RootBranchGrowthData {
		int m_order = 0;
	};

	struct RootSkeletonGrowthData {

	};

	class RootGrowthParameters {
	public:
		/**
		 * \brief The growth rate. The expected internode elongation per iteration
		 */
		float m_growthRate;
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
		 * \brief Apical control base and distance decrease from root.
		 */
		glm::vec2 m_apicalControlBaseDistFactor;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
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
		/**
		* The base branching probability
		*/
		float m_baseBranchingProbability;
		/**
		* The probability decrease for each children.
		*/
		float m_branchingProbabilityChildrenDecrease;
		/**
		* The probability decrease along the branch.
		*/
		float m_branchingProbabilityDistanceDecrease;

		[[nodiscard]] float GetRootApicalAngle(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootRollAngle(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootBranchingAngle(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetBranchingProbability(const Node<RootInternodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetApicalControlFactor(const Node<RootInternodeGrowthData>& rootNode) const;

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
		 * \brief The growth rate. The expected internode elongation per iteration
		 */
		float m_growthRate;
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
		 * \brief The lighting factor for lateral bud flushing probability.
		 */
		float m_lateralBudFlushingProbabilityLightingFactor;
		/**
		 * \brief The lighting factor for leaf bud flushing probability.
		 */
		float m_leafBudFlushingProbabilityLightingFactor;
		/**
		 * \brief The lighting factor for fruit bud flushing probability.
		 */
		float m_fruitBudFlushingProbabilityLightingFactor;
		/**
		 * \brief Apical control base and distance decrease from root.
		 */
		glm::vec2 m_apicalControlBaseDistFactor;

		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
		/**
		* \brief How much inhibitor will shrink when going through the branch.
		*/
		float m_apicalDominanceDistanceFactor;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_budKillProbability;
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

		[[nodiscard]] float GetDesiredBranchingAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetDesiredRollAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetDesiredApicalAngle(const Node<InternodeGrowthData>& internode) const;

		[[nodiscard]] float GetApicalControlFactor(const Node<InternodeGrowthData>& internode) const;

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

	struct IlluminationEstimationSettings {
		int m_probeLayerAmount = 8;
		int m_probeCountPerLayer = 8;
		float m_distanceLossMagnitude = 0.25f;
		float m_distanceLossFactor = 1.5f;
		float m_probeMinMaxRatio = 0.8f;
	};

	class TreeVolume
	{
	public:
		std::vector<std::vector<float>> m_layers;
		int m_layerAmount = 16;
		int m_sectorAmount = 16;
		float m_offset = 0;
		float m_maxHeight = 0.0f;
		bool m_hasData = false;
		[[nodiscard]] glm::ivec2 SelectSlice(const glm::vec3& position) const;
		void Clear();
		[[nodiscard]] glm::vec3 TipPosition(int layer, int slice) const;
		void Smooth();
		[[nodiscard]] float IlluminationEstimation(const glm::vec3& position, const IlluminationEstimationSettings& settings, glm::vec3& lightDirection) const;
	};

	class TreeModel {
#pragma region Root Growth

		bool ElongateRoot(SoilModel& soilModel, float extendLength, NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters, float& collectedAuxin);

		inline bool GrowRootNode(SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters);

		inline void CalculateResourceRequirement(NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters);
		inline void CalculateThickness(NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters);

		inline void CollectWaterAndNitriteFromRoots(const glm::mat4& globalTransform, SoilModel& soilModel,
			const RootGrowthParameters& rootGrowthParameters);

		inline void AccumulateRootNodeResourceRequirement();

		inline void ApicalControl(const RootGrowthParameters& rootGrowthParameters, float resourceLevel);

		inline void CalculateResourceRequirement(const RootGrowthParameters& rootGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement);

#pragma endregion
#pragma region Tree Growth
		inline void AccumulateInternodeResourceRequirement();

		inline void CalculateResourceRequirement(const TreeGrowthParameters& treeGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement);

		inline void ApicalControl(const TreeGrowthParameters& treeGrowthParameters, float resourceLevel);

		inline bool InternodePruning(float maxDistance, NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		inline void CalculateThicknessAndSagging(NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters);

		inline bool GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const TreeGrowthParameters& treeGrowthParameters);

		bool ElongateInternode(float extendLength, NodeHandle internodeHandle,
			const TreeGrowthParameters& treeGrowthParameters, float& collectedInhibitor);

		friend class Tree;
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
		int m_flowNodeLimit = 30;
		template <typename SkeletonData, typename FlowData, typename NodeData>
		void CollisionDetection(float minRadius, Octree<TreeVoxelData>& octree, Skeleton<SkeletonData, FlowData, NodeData>& skeleton);

		bool m_resourceFlow = false;

		TreeVolume m_treeVolume;
		IlluminationEstimationSettings m_illuminationEstimationSettings;
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

		void SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel);
		void SampleSoilDensity(const glm::mat4& globalTransform, SoilModel& soilModel);
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

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void TreeModel::CollisionDetection(float minRadius, Octree<TreeVoxelData>& octree,
		Skeleton<SkeletonData, FlowData, NodeData>& skeleton)
	{
		const auto boxSize = skeleton.m_max - skeleton.m_min;
		const float maxRadius = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z) + 2.0f * minRadius;
		int subdivisionLevel = 0;
		float testRadius = minRadius;
		while (testRadius <= maxRadius)
		{
			subdivisionLevel++;
			testRadius *= 2.f;
		}
		octree.Reset(maxRadius, subdivisionLevel, (skeleton.m_min + skeleton.m_max) * 0.5f);
		const auto& sortedRootNodeList = skeleton.RefSortedNodeList();
		const auto& sortedRootFlowList = skeleton.RefSortedFlowList();
		int collisionCount = 0;
		int flowCollisionCount = 0;
		std::unordered_map<int, int> nodeCollisionCollection;
		std::unordered_map<int, int> flowCollisionCollection;
		for (const auto& nodeHandle : sortedRootNodeList)
		{
			const auto& node = skeleton.RefNode(nodeHandle);
			const auto& info = node.m_info;
			octree.Occupy(info.m_globalPosition, info.m_globalRotation, info.m_length * 0.9f, info.m_thickness, [&](OctreeNode<TreeVoxelData>& octreeNode)
				{
					if (octreeNode.m_data.m_nodeHandle == nodeHandle) return;
			if (octreeNode.m_data.m_nodeHandle == node.GetParentHandle()) return;
			for (const auto& i : node.RefChildHandles()) if (octreeNode.m_data.m_nodeHandle == i) return;
			auto flowHandle = node.GetFlowHandle();
			if (octreeNode.m_data.m_referenceCount != 0)
			{
				if (octreeNode.m_data.m_nodeHandle > nodeHandle)
				{
					nodeCollisionCollection[octreeNode.m_data.m_nodeHandle] = nodeHandle;
				}
				else
				{
					nodeCollisionCollection[nodeHandle] = octreeNode.m_data.m_nodeHandle;
				}
				if (octreeNode.m_data.m_flowHandle != flowHandle) {
					if (octreeNode.m_data.m_flowHandle > flowHandle)
					{
						flowCollisionCollection[octreeNode.m_data.m_flowHandle] = flowHandle;
					}
					else
					{
						flowCollisionCollection[flowHandle] = octreeNode.m_data.m_flowHandle;
					}
				}
			}
			else {
				octreeNode.m_data.m_flowHandle = flowHandle;
				octreeNode.m_data.m_nodeHandle = nodeHandle;
			}
			octreeNode.m_data.m_referenceCount++;
				});

		}
		collisionCount = nodeCollisionCollection.size();
		flowCollisionCount = flowCollisionCollection.size();
		std::vector<int> collisionStat;
		collisionStat.resize(200);
		for (auto& i : collisionStat) i = 0;
		int totalVoxel = 0;
		octree.IterateLeaves([&](const OctreeNode<TreeVoxelData>& leaf)
			{
				collisionStat[leaf.m_data.m_referenceCount]++;
		totalVoxel++;
			});

		std::string report = "Collision: [" + std::to_string(collisionCount) + "/" + std::to_string(sortedRootNodeList.size()) + "], [" + std::to_string(flowCollisionCount) + "/" + std::to_string(sortedRootFlowList.size()) + "], ";
		report += "total occupied: " + std::to_string(totalVoxel) + ", collision stat: ";

		std::string appendStat;
		for (int i = 199; i > 0; i--)
		{
			if (collisionStat[i] != 0)
			{
				appendStat += "[" + std::to_string(i) + "]->" + std::to_string(collisionStat[i]) + "; ";
			}
		}
		if (appendStat.empty()) appendStat = "No collision";

		UNIENGINE_LOG(report + appendStat);
	}
}
