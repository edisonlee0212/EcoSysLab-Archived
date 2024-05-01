#pragma once
#include "Skeleton.hpp"
#include "EnvironmentGrid.hpp"
#include "StrandModelParameters.hpp"
#include "ProfileConstraints.hpp"
#include "TreeOccupancyGrid.hpp"
#include "Octree.hpp" 
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
		float m_flushingRate;// No Serialize
		float m_extinctionRate;// No Serialize

		BudType m_type = BudType::Apical;
		BudStatus m_status = BudStatus::Dormant;

		glm::quat m_localRotation = glm::vec3(0.0f);

		//-1.0 means the no fruit.
		ReproductiveModule m_reproductiveModule;
		glm::vec3 m_markerDirection = glm::vec3(0.0f);// No Serialize
		size_t m_markerCount = 0;// No Serialize
		float m_shootFlux = 0.0f;// No Serialize
	};

	struct ShootFlux {
		float m_value = 0.0f;
	};

	struct TreeVoxelData
	{
		SkeletonNodeHandle m_nodeHandle = -1;
		SkeletonNodeHandle m_flowHandle = -1;
		unsigned m_referenceCount = 0;
	};

#pragma endregion
	

	struct InternodeGrowthData {
		float m_internodeLength = 0.0f;
		int m_indexOfParentBud = 0;
		float m_startAge = 0;
		float m_finishAge = 0.0f;
		
		glm::quat m_desiredLocalRotation = glm::vec3(0.0f);
		glm::quat m_desiredGlobalRotation = glm::vec3(0.0f);
		glm::vec3 m_desiredGlobalPosition = glm::vec3(0.0f);

		float m_sagging = 0;
		int m_order = 0;
		float m_extraMass = 0.0f;
		float m_density = 1.f;
		float m_strength = 1.f;
		

		/**
		 * List of buds, first one will always be the apical bud which points forward.
		 */
		std::vector<Bud> m_buds;
		std::vector<glm::mat4> m_leaves;
		std::vector<glm::mat4> m_fruits;
		
		int m_level = 0;// No Serialize
		bool m_maxChild = false;// No Serialize
		float m_descendantTotalBiomass = 0;// No Serialize
		float m_biomass = 0;// No Serialize
		glm::vec3 m_desiredDescendantWeightCenter = glm::vec3(0.f);
		glm::vec3 m_descendantWeightCenter = glm::vec3(0.f);
		float m_temperature = 0.0f;// No Serialize
		float m_inhibitorSink = 0;// No Serialize
		float m_lightIntensity = 1.0f;// No Serialize
		float m_maxDescendantLightIntensity = 0.f;//No Serialize
		glm::vec3 m_lightDirection = glm::vec3(0, 1, 0);// No Serialize
		float m_growthPotential = 0.0f;// No Serialize
		float m_desiredGrowthRate = 0.0f;// No Serialize
		float m_growthRate = 0.0f;// No Serialize
		float m_spaceOccupancy = 0.0f;
	};

	struct ShootStemGrowthData {
		int m_order = 0;
	};

	struct ShootGrowthData {
		Octree<TreeVoxelData> m_octree = {};

		size_t m_maxMarkerCount = 0;

		std::vector<ReproductiveModule> m_droppedLeaves;
		std::vector<ReproductiveModule> m_droppedFruits;

		glm::vec3 m_desiredMin = glm::vec3(FLT_MAX);
		glm::vec3 m_desiredMax = glm::vec3(FLT_MIN);

		int m_maxLevel = 0;
		int m_maxOrder = 0;
	};


	typedef Skeleton<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> ShootSkeleton;

	struct StrandModelNodeData
	{
		StrandModelProfile<CellParticlePhysicsData> m_profile{};
		std::unordered_map<StrandHandle, ParticleHandle> m_particleMap{};
		bool m_boundariesUpdated = false;
		ProfileConstraints m_profileConstraints{};

		float m_frontControlPointDistance = 0.0f;
		float m_backControlPointDistance = 0.0f;

		float m_centerDirectionRadius = 0.0f;

		glm::vec2 m_offset = glm::vec2(0.0f);
		float m_twistAngle = 0.0f;
		int m_packingIteration = 0;
		bool m_split = false;
		
		float m_strandRadius = 0.002f;
		int m_strandCount = 0;

		JobHandle m_job = {};
	};

	struct StrandModelFlowData
	{

	};

	struct StrandModelSkeletonData
	{
		StrandModelStrandGroup m_strandGroup{};
		int m_numOfParticles = 0;
	};

	typedef Skeleton<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData> StrandModelSkeleton;
}