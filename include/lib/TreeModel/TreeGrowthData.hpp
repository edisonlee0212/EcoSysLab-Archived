#pragma once
#include "Skeleton.hpp"
#include "EnvironmentGrid.hpp"
#include "PipeModelParameters.hpp"
#include "ProfileConstraints.hpp"
#include "TreeOccupancyGrid.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
#pragma region Utilities
	enum class BudType {
		Apical,
		Lateral,
		Leaf,
		Fruit
	};

	enum class BudStatus {
		Dormant,
		Died,
		Removed
	};

	struct ReproductiveModule
	{
		float m_maturity = 0.0f;
		float m_health = 1.0f;
		glm::mat4 m_transform = glm::mat4(0.0f);
		void Reset();
	};

	class Bud {
	public:
		float m_flushingRate;
		float m_extinctionRate;

		BudType m_type = BudType::Apical;
		BudStatus m_status = BudStatus::Dormant;

		glm::quat m_localRotation = glm::vec3(0.0f);

		//-1.0 means the no fruit.
		ReproductiveModule m_reproductiveModule;
		glm::vec3 m_markerDirection = glm::vec3(0.0f);
		size_t m_markerCount = 0;
		float m_shootFlux = 0.0f;
	};

	struct ShootFlux {
		float m_value = 0.0f;
	};

	struct TreeVoxelData
	{
		NodeHandle m_nodeHandle = -1;
		NodeHandle m_flowHandle = -1;
		unsigned m_referenceCount = 0;
	};

#pragma endregion

	struct InternodeGrowthData {
		float m_internodeLength = 0.0f;
		int m_indexOfParentBud = 0;
		bool m_maxChild = false;
		bool m_lateral = false;
		float m_startAge = 0;
		float m_finishAge = 0.0f;
		float m_inhibitorSink = 0;
		glm::quat m_desiredLocalRotation = glm::vec3(0.0f);
		glm::quat m_desiredGlobalRotation = glm::vec3(0.0f);
		glm::vec3 m_desiredGlobalPosition = glm::vec3(0.0f);

		float m_sagging = 0;

		int m_order = 0;
		int m_level = 0;
		float m_descendentTotalBiomass = 0;
		float m_biomass = 0;
		float m_extraMass = 0.0f;
		float m_temperature = 0.0f;

		float m_lightIntensity = 1.0f;
		glm::vec3 m_lightDirection = glm::vec3(0, 1, 0);
		float m_pipeResistance = 0.0f;

		float m_growthPotential = 0.0f;
		float m_apicalControl = 0.0f;
		float m_desiredGrowthRate = 0.0f;
		float m_growthRate = 0.0f;

		float m_spaceOccupancy = 0.0f;
		
		
		/**
		 * List of buds, first one will always be the apical bud which points forward.
		 */
		std::vector<Bud> m_buds;
		std::vector<glm::mat4> m_leaves;
		std::vector<glm::mat4> m_fruits;

#pragma region Pipe Model
		PipeProfile<CellParticlePhysicsData> m_frontProfile{};
		std::unordered_map<PipeHandle, ParticleHandle> m_frontParticleMap{};
		bool m_boundariesUpdated = false;
		PipeProfile<CellParticlePhysicsData> m_backProfile{};
		std::unordered_map<PipeHandle, ParticleHandle> m_backParticleMap{};
		ProfileConstraints m_profileConstraints {};

		float m_frontControlPointDistance = 0.0f;
		float m_backControlPointDistance = 0.0f;

		float m_centerDirectionRadius = 0.0f;

		glm::vec2 m_offset = glm::vec2(0.0f);

		glm::vec2 m_shift = glm::vec2(0.0f);
		bool m_needPacking = false;
		bool m_apical = false;
		bool m_split = false;
		std::vector<std::shared_future<void>> m_tasks{};

		glm::vec3 m_adjustedGlobalPosition{};
		glm::quat m_adjustedGlobalRotation{};
		float m_pipeCellRadius = 0.002f;
		int m_pipeSize = 0;
#pragma endregion
	};

	struct ShootStemGrowthData {
		int m_order = 0;
	};

	struct ShootGrowthData {
		Octree<TreeVoxelData> m_octree = {};

		size_t m_maxMarkerCount = 0;

		//ShootFlux m_shootFlux = {};

		std::vector<ReproductiveModule> m_droppedLeaves;
		std::vector<ReproductiveModule> m_droppedFruits;

		glm::vec3 m_desiredMin = glm::vec3(FLT_MAX);
		glm::vec3 m_desiredMax = glm::vec3(FLT_MIN);

		int m_maxLevel = 0;

#pragma region Pipe Model
		bool m_parallelScheduling = true;
		
		PipeModelPipeGroup m_pipeGroup {};
		
		float m_profileCalculationTime = 0.0f;
		int m_numOfParticles = 0;
#pragma endregion
	};


	typedef Skeleton<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> ShootSkeleton;
}