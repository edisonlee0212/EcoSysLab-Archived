#pragma once
#include "VoxelSoilModel.hpp"
#include "ClimateModel.hpp"
#include "Octree.hpp"
#include "TreeGrowthController.hpp"
#include "TreeIOTree.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	struct TreeGrowthSettings
	{
		bool m_enableRoot = false;
		bool m_enableShoot = true;

		bool m_autoBalance = true;
		bool m_collectRootFlux = true;
		
		bool m_collectNitrite = true;

		float m_nodeDevelopmentalVigorFillingRate = 1.0f;

		bool m_enableRootCollisionDetection = false;
		bool m_enableBranchCollisionDetection = false;

		bool m_useSpaceColonization = false;
		bool m_spaceColonizationAutoResize = false;
		float m_spaceColonizationRemovalDistanceFactor = 2;
		float m_spaceColonizationDetectionDistanceFactor = 4;
		float m_spaceColonizationTheta = 90.0f;

		
	};



	class TreeModel {
#pragma region Root Growth

		bool ElongateRoot(VoxelSoilModel& soilModel, float extendLength, NodeHandle rootNodeHandle,
			const RootGrowthController& rootGrowthParameters, float& collectedAuxin);

		bool GrowRootNode(VoxelSoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthController& rootGrowthParameters);

		void CalculateThickness(NodeHandle rootNodeHandle,
			const RootGrowthController& rootGrowthParameters);

		bool PruneRootNodes(const RootGrowthController& rootGrowthParameters);

		void AggregateRootVigorRequirement(const RootGrowthController& rootGrowthParameters, NodeHandle baseRootNodeHandle);

		void AllocateRootVigor(const RootGrowthController& rootGrowthParameters);

		void CalculateVigorRequirement(const RootGrowthController& rootGrowthParameters, RootGrowthRequirement& newRootGrowthNutrientsRequirement);
		void SampleNitrite(const glm::mat4& globalTransform, VoxelSoilModel& soilModel);

		void RootGrowthPostProcess(const glm::mat4& globalTransform, VoxelSoilModel& soilModel,
			const RootGrowthController& rootGrowthParameters);
#pragma endregion
#pragma region Tree Growth
		void AggregateInternodeVigorRequirement(const ShootGrowthController& shootGrowthParameters, NodeHandle baseInternodeHandle);

		void CalculateVigorRequirement(const ShootGrowthController& shootGrowthParameters, ShootGrowthRequirement& newTreeGrowthNutrientsRequirement);

		void AllocateShootVigor(float vigor, NodeHandle baseInternodeHandle, const std::vector<NodeHandle>& sortedInternodeList, const ShootGrowthRequirement& shootGrowthRequirement, const ShootGrowthController& shootGrowthParameters);

		bool PruneInternodes(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthParameters);

		void CalculateThicknessAndSagging(NodeHandle internodeHandle,
			const ShootGrowthController& shootGrowthParameters);

		bool GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const ShootGrowthController& shootGrowthParameters, bool seasonality);

		bool ElongateInternode(float extendLength, NodeHandle internodeHandle,
			const ShootGrowthController& shootGrowthController, float& collectedInhibitor);

		void ShootGrowthPostProcess(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthParameters);

		friend class Tree;
#pragma endregion

		void Initialize(const ShootGrowthController& shootGrowthParameters, const RootGrowthController& rootGrowthParameters);

		bool m_initialized = false;

		ShootSkeleton m_shootSkeleton;
		RootSkeleton m_rootSkeleton;


		std::deque<std::pair<ShootSkeleton, RootSkeleton>> m_history;

		/**
		 * Grow one iteration of the branches, given the climate model and the procedural parameters.
		 * @param globalTransform The plant's world transform.
		 * @param climateModel The climate model.
		 * @param shootGrowthParameters The procedural parameters that guides the growth.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowShoots(const glm::mat4& globalTransform, ClimateModel& climateModel,
			const ShootGrowthController& shootGrowthParameters);

		

		/**
		 * Grow one iteration of the roots, given the soil model and the procedural parameters.
		 * @param globalTransform The plant's world transform.
		 * @param soilModel The soil model
		 * @param rootGrowthParameters The procedural parameters that guides the growth.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowRoots(const glm::mat4& globalTransform, VoxelSoilModel& soilModel,
			const RootGrowthController& rootGrowthParameters);


		void PlantVigorAllocation();

		int m_leafCount = 0;
		int m_fruitCount = 0;
		int m_fineRootCount = 0;
		int m_twigCount = 0;

		float m_age = 0;
		int m_ageInYear = 0;
		float m_internodeDevelopmentRate = 1.0f;
		float m_rootNodeDevelopmentRate = 1.0f;
		float m_currentDeltaTime = 1.0f;

		bool m_enableRoot = true;
		bool m_enableShoot = true;

		void ResetReproductiveModule();


	public:
		float m_crownShynessDistance = 0.0f;
		unsigned m_index = 0;
		void RegisterVoxel(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthParameters);
		TreeOccupancyGrid m_treeOccupancyGrid{};

		void CalculateInternodeTransforms();
		void CalculateRootNodeTransforms();

		void PruneInternode(NodeHandle internodeHandle);
		void PruneRootNode(NodeHandle rootNodeHandle);

		void CollectRootFlux(const glm::mat4& globalTransform, VoxelSoilModel& soilModel, const std::vector<NodeHandle>& sortedSubTreeRootNodeList,
			const RootGrowthController& rootGrowthParameters);
		void CollectShootFlux(const glm::mat4& globalTransform, ClimateModel& climateModel, const std::vector<NodeHandle>& sortedSubTreeInternodeList,
			const ShootGrowthController& shootGrowthParameters);
		void HarvestFruits(const std::function<bool(const ReproductiveModule& fruit)>& harvestFunction);

		int m_iteration = 0;


		static void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::vec3& front, glm::vec3& up);

		static void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::quat& rotation);

		std::vector<int> m_internodeOrderCounts;
		std::vector<int> m_rootNodeOrderCounts;

		template <typename SkeletonData, typename FlowData, typename NodeData>
		void CollisionDetection(float minRadius, Octree<TreeVoxelData>& octree, Skeleton<SkeletonData, FlowData, NodeData>& skeleton);

		TreeGrowthSettings m_treeGrowthSettings;

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
		 * @param deltaTime The real world time for this iteration
		 * @param globalTransform The global transform of tree in world space.
		 * @param soilModel The soil model
		 * @param climateModel The climate model
		 * @param rootGrowthParameters The procedural parameters that guides the growth of the roots.
		 * @param shootGrowthParameters The procedural parameters that guides the growth of the branches.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool Grow(float deltaTime, const glm::mat4& globalTransform, VoxelSoilModel& soilModel, ClimateModel& climateModel,
			const RootGrowthController& rootGrowthParameters, const ShootGrowthController& shootGrowthParameters);

		bool GrowSubTree(float deltaTime, NodeHandle baseInternodeHandle, const glm::mat4& globalTransform, ClimateModel& climateModel,
			const ShootGrowthController& shootGrowthParameters);


		int m_historyLimit = -1;

		void SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel);
		void SampleSoilDensity(const glm::mat4& globalTransform, VoxelSoilModel& soilModel);
		[[nodiscard]] ShootSkeleton& RefShootSkeleton();

		[[nodiscard]] const ShootSkeleton& PeekShootSkeleton(int iteration = -1) const;

		[[nodiscard]] RootSkeleton&
			RefRootSkeleton();


		[[nodiscard]] const RootSkeleton& PeekRootSkeleton(int iteration = -1) const;

		void ClearHistory();

		void Step();

		void Pop();

		[[nodiscard]] int CurrentIteration() const;

		void Reverse(int iteration);

		void ExportTreeIOSkeleton(treeio::ArrayTree& arrayTree) const;
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
			octree.Occupy(info.m_globalPosition, info.m_globalRotation, info.m_length * 0.9f, info.m_thickness, [&](OctreeNode& octreeNode)
				{
					auto& data = octree.RefNodeData(octreeNode);
					if (data.m_nodeHandle == nodeHandle) return;
					if (data.m_nodeHandle == node.GetParentHandle()) return;
					for (const auto& i : node.RefChildHandles()) if (data.m_nodeHandle == i) return;
					auto flowHandle = node.GetFlowHandle();
					if (data.m_referenceCount != 0)
					{
						if (data.m_nodeHandle > nodeHandle)
						{
							nodeCollisionCollection[data.m_nodeHandle] = nodeHandle;
						}
						else
						{
							nodeCollisionCollection[nodeHandle] = data.m_nodeHandle;
						}
						if (data.m_flowHandle != flowHandle) {
							if (data.m_flowHandle > flowHandle)
							{
								flowCollisionCollection[data.m_flowHandle] = flowHandle;
							}
							else
							{
								flowCollisionCollection[flowHandle] = data.m_flowHandle;
							}
						}
					}
					else {
						data.m_flowHandle = flowHandle;
						data.m_nodeHandle = nodeHandle;
					}
					data.m_referenceCount++;
				});

		}
		collisionCount = nodeCollisionCollection.size();
		flowCollisionCount = flowCollisionCollection.size();
		std::vector<int> collisionStat;
		collisionStat.resize(200);
		for (auto& i : collisionStat) i = 0;
		int totalVoxel = 0;
		octree.IterateLeaves([&](const OctreeNode& leaf)
			{
				collisionStat[octree.PeekNodeData(leaf).m_referenceCount]++;
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

		EVOENGINE_LOG(report + appendStat);
	}
}
