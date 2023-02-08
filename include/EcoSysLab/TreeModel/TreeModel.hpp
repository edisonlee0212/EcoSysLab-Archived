#pragma once

#include "PlantStructure.hpp"
#include "VigorSink.hpp"
#include "TreeVolume.hpp"
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

		VigorSink m_vigorSink;

		glm::quat m_localRotation = glm::vec3(0.0f);

		//-1.0 means the no fruit.
		float m_maturity = -1.0f;
		float m_drought = 0.0f;

		float m_chlorophyll = 0.0f;

		float m_shootFlux = 0.0f;

		glm::vec3 m_reproductiveModuleSize = glm::vec3(0.0f);
		glm::mat4 m_reproductiveModuleTransform = glm::mat4(0.0f);
	};
	
	struct InternodeGrowthData {
		bool m_isMaxChild = false;
		bool m_lateral = false;
		float m_age = 0;
		float m_inhibitor = 0;
		glm::quat m_desiredLocalRotation = glm::vec3(0.0f);
		float m_sagging = 0;

		float m_maxDistanceToAnyBranchEnd = 0;
		int m_order = 0;
		float m_descendentTotalBiomass = 0;
		float m_biomass = 0;
		float m_extraMass = 0.0f;
		float m_rootDistance = 0;

		float m_temperature = 0.0f;

		glm::vec3 m_lightDirection = glm::vec3(0, 1, 0);
		float m_lightIntensity = 1.0f;

		float m_lightEnergy = 0.0f;
		/**
		 * List of buds, first one will always be the apical bud which points forward.
		 */
		std::vector<Bud> m_buds;
		VigorFlow m_vigorFlow;
		std::vector<glm::mat4> m_leaves;
		std::vector<glm::mat4> m_fruits;

	};

	struct ShootStemGrowthData {
		int m_order = 0;

	};

	struct ShootGrowthData {

	};

	struct RootNodeGrowthData {
		bool m_isMaxChild = false;
		bool m_lateral = false;
		float m_soilDensity = 0.0f;
		float m_age = 0;
		float m_maxDistanceToAnyBranchEnd = 0;
		float m_descendentTotalBiomass = 0;
		float m_biomass = 0;
		float m_extraMass = 0.0f;

		float m_rootDistance = 0;
		int m_order = 0;

		float m_nitrite = 1.0f;
		float m_water = 1.0f;

		float m_inhibitor = 0;

		float m_horizontalTropism = 0.0f;
		float m_verticalTropism = 0.0f;
		VigorFlow m_vigorFlow;
		/*
		 * The allocated total resource for maintenance and development of this module.
		 */
		VigorSink m_vigorSink;

		std::vector<glm::vec4> m_fineRootAnchors;

	};
	struct RootStemGrowthData {
		int m_order = 0;
	};

	struct RootGrowthData {

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
		float m_rootNodeGrowthRate;
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
		float m_branchingProbability = 1.0f;

		float m_fineRootSegmentLength = 0.02f;
		float m_fineRootApicalAngleVariance = 2.5f;
		float m_fineRootBranchingAngle = 60.f;
		float m_fineRootThickness = 0.002f;
		float m_fineRootMinNodeThickness = 0.05f;
		int m_fineRootNodeCount = 2;

		[[nodiscard]] float GetRootApicalAngle(const Node<RootNodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootRollAngle(const Node<RootNodeGrowthData>& rootNode) const;

		[[nodiscard]] float GetRootBranchingAngle(const Node<RootNodeGrowthData>& rootNode) const;



		void SetTropisms(Node<RootNodeGrowthData>& oldNode, Node<RootNodeGrowthData>& newNode) const;

		RootGrowthParameters();
	};




	class ShootGrowthParameters {
	public:
		/**
		 * \brief The growth rate. The expected internode elongation per iteration
		 */
		float m_growthRate;

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


		[[nodiscard]] float GetSagging(const Node<InternodeGrowthData>& internode) const;


		ShootGrowthParameters();
	};

	struct RootFlux {
		float m_nitrite = 0.0f;
		float m_water = 0.0f;
	};

	struct ShootFlux {
		float m_lightEnergy = 0.0f;
	};

	struct PlantVigor
	{
		float m_rootVigor = 0.0f;
		float m_shootVigor = 0.0f;
	};

	struct PlantGrowthRequirement
	{
		float m_maintenanceVigor = 0.0f;
		float m_developmentalVigor = 0.0f;
	};

	struct TreeVoxelData
	{
		NodeHandle m_nodeHandle = -1;
		NodeHandle m_flowHandle = -1;
		unsigned m_referenceCount = 0;
	};

	

	struct ShootRootVigorRatio
	{
		float m_rootVigorWeight = 1.0f;
		float m_shootVigorWeight = 1.0f;
	};

	
	typedef Skeleton<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> ShootSkeleton;
	typedef Skeleton<RootGrowthData, RootStemGrowthData, RootNodeGrowthData> RootSkeleton;
	class TreeModel {
#pragma region Root Growth

		bool ElongateRoot(SoilModel& soilModel, float extendLength, NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters, float& collectedAuxin);

		inline bool GrowRootNode(SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters);

		inline void CalculateThickness(NodeHandle rootNodeHandle,
			const RootGrowthParameters& rootGrowthParameters);

		inline void CollectRootFlux(const glm::mat4& globalTransform, SoilModel& soilModel,
			const RootGrowthParameters& rootGrowthParameters);

		inline void AggregateRootVigorRequirement();

		inline void AllocateRootVigor(const RootGrowthParameters& rootGrowthParameters);

		inline void CalculateVigorRequirement(const RootGrowthParameters& rootGrowthParameters, PlantGrowthRequirement& newRootGrowthNutrientsRequirement);
		inline void SampleNitrite(const glm::mat4& globalTransform, SoilModel& soilModel);
#pragma endregion
#pragma region Tree Growth
		inline void AggregateInternodeVigorRequirement();

		inline void CalculateVigorRequirement(const ShootGrowthParameters& shootGrowthParameters, PlantGrowthRequirement& newTreeGrowthNutrientsRequirement);

		inline void AllocateShootVigor(const ShootGrowthParameters& shootGrowthParameters);

		inline bool PruneInternodes(float maxDistance, NodeHandle internodeHandle,
			const ShootGrowthParameters& shootGrowthParameters);

		inline void CalculateThicknessAndSagging(NodeHandle internodeHandle,
			const ShootGrowthParameters& shootGrowthParameters);

		inline bool GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const ShootGrowthParameters& shootGrowthParameters);

		bool ElongateInternode(float extendLength, NodeHandle internodeHandle,
			const ShootGrowthParameters& shootGrowthParameters, float& collectedInhibitor);

		friend class Tree;
		void CollectShootFlux(const glm::mat4& globalTransform, ClimateModel& climateModel,
			const ShootGrowthParameters& shootGrowthParameters);
#pragma endregion

		void Initialize(const ShootGrowthParameters& shootGrowthParameters, const RootGrowthParameters& rootGrowthParameters);

		bool m_initialized = false;
		ShootSkeleton m_shootSkeleton;
		RootSkeleton m_rootSkeleton;
		std::deque<std::pair<ShootSkeleton, RootSkeleton>> m_history;

		/**
		 * Grow one iteration of the branches, given the climate model and the procedural parameters.
		 * @param globalTransform The plant's world transform.
		 * @param climateModel The climate model.
		 * @param shootGrowthParameters The procedural parameters that guides the growth.
		 * @param newShootGrowthRequirement Growth requirements from shoots.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowShoots(const glm::mat4& globalTransform, ClimateModel& climateModel, 
			const ShootGrowthParameters& shootGrowthParameters, PlantGrowthRequirement& newShootGrowthRequirement);

		/**
		 * Grow one iteration of the roots, given the soil model and the procedural parameters.
		 * @param globalTransform The plant's world transform.
		 * @param soilModel The soil model
		 * @param rootGrowthParameters The procedural parameters that guides the growth.
		 * @param newRootGrowthRequirement Growth requirements from roots.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowRoots(const glm::mat4& globalTransform, SoilModel& soilModel,
			const RootGrowthParameters& rootGrowthParameters, PlantGrowthRequirement& newRootGrowthRequirement);

		inline void PlantVigorAllocation();

		int m_leafCount = 0;
		int m_fruitCount = 0;
		int m_fineRootCount = 0;

		float m_age = 0;
		float m_internodeDevelopmentRate = 1.0f;
		float m_rootNodeDevelopmentRate = 1.0f;
		float m_currentDeltaTime = 1.0f;
	public:
		static void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::vec3& front, glm::vec3& up);

		static void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::quat& rotation);

		std::vector<int> m_internodeOrderCounts;
		std::vector<int> m_rootNodeOrderCounts;

		int m_flowNodeLimit = 10;
		template <typename SkeletonData, typename FlowData, typename NodeData>
		void CollisionDetection(float minRadius, Octree<TreeVoxelData>& octree, Skeleton<SkeletonData, FlowData, NodeData>& skeleton);

		bool m_collectLight = false;
		bool m_collectWater = false;
		bool m_collectNitrite = true;
		TreeVolume m_treeVolume;
		IlluminationEstimationSettings m_illuminationEstimationSettings;
		Octree<TreeVoxelData> m_rootOctree;
		Octree<TreeVoxelData> m_branchOctree;
		bool m_enableRootCollisionDetection = false;
		bool m_enableBranchCollisionDetection = false;

		PlantGrowthRequirement m_shootGrowthRequirement;
		PlantGrowthRequirement m_rootGrowthRequirement;
		ShootFlux m_shootFlux;
		RootFlux m_rootFlux;
		PlantVigor m_plantVigor;
		ShootRootVigorRatio m_vigorRatio;
		glm::vec3 m_currentGravityDirection = glm::vec3(0, -1, 0);

		/**
		 * Erase the entire tree.
		 */
		void Clear();

		[[nodiscard]] int GetLeafCount() const;
		[[nodiscard]] int GetFruitCount() const;
		[[nodiscard]] int GetFineRootCount() const;
		/**
		 * Grow one iteration of the tree, given the nutrients and the procedural parameters.
		 * @param globalTransform The global transform of tree in world space.
		 * @param soilModel The soil model
		 * @param climateModel The climate model
		 * @param rootGrowthParameters The procedural parameters that guides the growth of the roots.
		 * @param shootGrowthParameters The procedural parameters that guides the growth of the branches.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool Grow(float deltaTime, const glm::mat4& globalTransform, SoilModel& soilModel, ClimateModel& climateModel,
			const RootGrowthParameters& rootGrowthParameters, const ShootGrowthParameters& shootGrowthParameters);

		int m_historyLimit = -1;

		void SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel);
		void SampleSoilDensity(const glm::mat4& globalTransform, SoilModel& soilModel);
		[[nodiscard]] ShootSkeleton& RefShootSkeleton();

		[[nodiscard]] const ShootSkeleton& PeekShootSkeleton(int iteration) const;

		[[nodiscard]] RootSkeleton&
			RefRootSkeleton();


		[[nodiscard]] const RootSkeleton& PeekRootSkeleton(int iteration) const;

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
