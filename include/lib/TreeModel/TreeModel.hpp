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
		float m_nodeDevelopmentalVigorFillingRate = 1.0f;
		bool m_useSpaceColonization = false;
		bool m_spaceColonizationAutoResize = false;
		float m_spaceColonizationRemovalDistanceFactor = 2;
		float m_spaceColonizationDetectionDistanceFactor = 4;
		float m_spaceColonizationTheta = 90.0f;
	};

	class TreeModel {
#pragma region Tree Growth
		ShootFlux CollectShootFlux(const std::vector<NodeHandle>& sortedInternodeList);

		void AdjustGrowthRate(const std::vector<NodeHandle>& sortedInternodeList, float factor);

		float CalculateDesiredGrowthRate(const std::vector<NodeHandle>& sortedInternodeList, const ShootGrowthController& shootGrowthController);

		bool PruneInternodes(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthController);

		void CalculateThickness(const ShootGrowthController& shootGrowthController);

		void CalculateBiomass(const ShootGrowthController& shootGrowthController);

		void CalculateLevel();

		bool GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const ShootGrowthController& shootGrowthController);

		bool ElongateInternode(float extendLength, NodeHandle internodeHandle,
			const ShootGrowthController& shootGrowthController, float& collectedInhibitor);

		void ShootGrowthPostProcess(const ShootGrowthController& shootGrowthController);

		friend class Tree;
#pragma endregion

		void Initialize(const ShootGrowthController& shootGrowthController);

		bool m_initialized = false;

		ShootSkeleton m_shootSkeleton;

		std::deque<ShootSkeleton> m_history;

		int m_leafCount = 0;
		int m_fruitCount = 0;
		int m_twigCount = 0;

		float m_age = 0;
		int m_ageInYear = 0;
		float m_currentDeltaTime = 1.0f;

		bool m_enableShoot = true;

		void ResetReproductiveModule();

		int m_currentSeedValue = 0;

	public:
		float GetSubTreeMaxAge(NodeHandle baseInternodeHandle) const;
		bool Reduce(const ShootGrowthController& shootGrowthController, NodeHandle baseInternodeHandle, float targetAge);

		void CalculateTransform(const ShootGrowthController& shootGrowthController, bool sagging);

		int m_seed = 0;

		float m_crownShynessDistance = 0.0f;
		unsigned m_index = 0;
		void RegisterVoxel(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthController);
		TreeOccupancyGrid m_treeOccupancyGrid{};

		void PruneInternode(NodeHandle internodeHandle);

		void CalculateShootFlux(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthController);
		

		void HarvestFruits(const std::function<bool(const ReproductiveModule& fruit)>& harvestFunction);

		int m_iteration = 0;


		static void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::vec3& front, glm::vec3& up);

		static void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::quat& rotation);

		std::vector<int> m_internodeOrderCounts;

		TreeGrowthSettings m_treeGrowthSettings;

		glm::vec3 m_currentGravityDirection = glm::vec3(0, -1, 0);

		/**
		 * Erase the entire tree.
		 */
		void Clear();

		[[nodiscard]] int GetLeafCount() const;
		[[nodiscard]] int GetFruitCount() const;
		/**
		 * Grow one iteration of the tree, given the nutrients and the procedural parameters.
		 * @param deltaTime The real world time for this iteration.
		 * @param baseInternodeHandle The base internode of the subtree to grow. If set to 0 or -1, this equals with growing full tree.
		 * @param globalTransform The global transform of tree in world space.
		 * @param climateModel The climate model.
		 * @param shootGrowthController The procedural parameters that guides the growth of the branches.
		 * @param pruning If we want auto pruning to be enabled.
		 * @param overrideGrowthRate If positive (clamped to below 1), the growth rate will be overwritten instead of calculating by available resources.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool Grow(float deltaTime, NodeHandle baseInternodeHandle, const glm::mat4& globalTransform, ClimateModel& climateModel,
			const ShootGrowthController& shootGrowthController, bool pruning = true, float overrideGrowthRate = -1);

		int m_historyLimit = -1;

		void SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel);
		[[nodiscard]] ShootSkeleton& RefShootSkeleton();

		[[nodiscard]] const ShootSkeleton& PeekShootSkeleton(int iteration = -1) const;

		void ClearHistory();

		void Step();

		void Pop();

		[[nodiscard]] int CurrentIteration() const;

		void Reverse(int iteration);

		void ExportTreeIOSkeleton(treeio::ArrayTree& arrayTree) const;


#pragma region Pipe model
		void InitializeProfiles(const PipeModelParameters& pipeModelParameters);
		void CalculateProfiles(const PipeModelParameters& pipeModelParameters);
		void CalculateProfile(float maxRootDistance, NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters, bool scheduling);
		void Wait(NodeHandle nodeHandle);

		void PackTask(NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters, bool parallel);
		void MergeTask(float maxRootDistance, NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters);
		void CopyFrontToBackTask(NodeHandle nodeHandle);
		void CalculateShiftTask(NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters);
		void ApplyProfile(const PipeModelParameters& pipeModelParameters,
			NodeHandle nodeHandle);
		void ApplyProfiles(const PipeModelParameters& pipeModelParameters);
		void CalculatePipeProfileAdjustedTransforms(const PipeModelParameters& pipeModelParameters);
		glm::vec3 InterpolatePipeSegmentPosition(PipeSegmentHandle pipeSegmentHandle, float a) const;
		glm::vec3 InterpolatePipeSegmentAxis(PipeSegmentHandle pipeSegmentHandle, float a) const;
		float InterpolatePipeSegmentRadius(PipeSegmentHandle pipeSegmentHandle, float a) const;
		
#pragma endregion
	};
}
