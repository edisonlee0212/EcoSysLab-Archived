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

		void Initialize(const ShootGrowthController& shootGrowthParameters);

		bool m_initialized = false;

		ShootSkeleton m_shootSkeleton;

		std::deque<ShootSkeleton> m_history;

		/**
		 * Grow one iteration of the branches, given the climate model and the procedural parameters.
		 * @param globalTransform The plant's world transform.
		 * @param climateModel The climate model.
		 * @param shootGrowthParameters The procedural parameters that guides the growth.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool GrowShoots(const glm::mat4& globalTransform, ClimateModel& climateModel,
			const ShootGrowthController& shootGrowthParameters);

		void PlantVigorAllocation();

		int m_leafCount = 0;
		int m_fruitCount = 0;
		int m_twigCount = 0;

		float m_age = 0;
		int m_ageInYear = 0;
		float m_internodeDevelopmentRate = 1.0f;
		float m_currentDeltaTime = 1.0f;

		bool m_enableShoot = true;

		void ResetReproductiveModule();

		int m_currentSeedValue = 0;

	public:

		int m_seed = 0;

		float m_crownShynessDistance = 0.0f;
		unsigned m_index = 0;
		void RegisterVoxel(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthParameters);
		TreeOccupancyGrid m_treeOccupancyGrid{};

		void CalculateInternodeTransforms();

		void PruneInternode(NodeHandle internodeHandle);

		void CollectShootFlux(const glm::mat4& globalTransform, ClimateModel& climateModel, const std::vector<NodeHandle>& sortedSubTreeInternodeList,
			const ShootGrowthController& shootGrowthParameters);
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
		 * @param deltaTime The real world time for this iteration
		 * @param globalTransform The global transform of tree in world space.
		 * @param climateModel The climate model
		 * @param shootGrowthParameters The procedural parameters that guides the growth of the branches.
		 * @return Whether the growth caused a structural change during the growth.
		 */
		bool Grow(float deltaTime, const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthParameters);

		bool GrowSubTree(float deltaTime, NodeHandle baseInternodeHandle, const glm::mat4& globalTransform, ClimateModel& climateModel,
			const ShootGrowthController& shootGrowthParameters);

		int m_historyLimit = -1;

		void SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel);
		void SampleSoilDensity(const glm::mat4& globalTransform, VoxelSoilModel& soilModel);
		[[nodiscard]] ShootSkeleton& RefShootSkeleton();

		[[nodiscard]] const ShootSkeleton& PeekShootSkeleton(int iteration = -1) const;

		void ClearHistory();

		void Step();

		void Pop();

		[[nodiscard]] int CurrentIteration() const;

		void Reverse(int iteration);

		void ExportTreeIOSkeleton(treeio::ArrayTree& arrayTree) const;
	};
}
