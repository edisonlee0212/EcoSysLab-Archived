#pragma once
using namespace EvoEngine;
namespace EcoSysLab
{
	typedef int SpatialPlantParameterHandle;
	typedef int SpatialPlantHandle;
	struct SpatialPlantParameter
	{
		/**
		 * \brief Final size of plant
		 */
		float m_w = 10.f;
		/**
		 * \brief Growth Rate
		 */
		float m_k = 0.01f;

		float m_seedingSizeFactor = .2f;
		float m_seedInitialRadius = 1.f;
		float m_seedingPossibility = 0.01f;
		/**
		 * \brief The represented color of the plant.
		 */
		glm::vec4 m_color = glm::vec4(1.0f);
	};

	struct SpatialPlant {
		bool m_recycled = false;
		SpatialPlantParameterHandle m_parameterHandle = 0;
		SpatialPlantHandle m_handle;
		glm::vec2 m_position = glm::vec2(0.0f);
		float m_radius = 0.0f;
		[[nodiscard]] float Overlap(const SpatialPlant& otherPlant) const;
		[[nodiscard]] float SymmetricInfluence(const SpatialPlant& otherPlant) const;
		[[nodiscard]] float AsymmetricInfluence(const SpatialPlant& otherPlant) const;
		/**
		 * \brief If the radii of self is larger, self gets more resources.
		 * \param otherPlant 
		 * \param weightingFactor 
		 * \return 
		 */
		[[nodiscard]] float AsymmetricalCompetition(const SpatialPlant& otherPlant, float weightingFactor) const;
		[[nodiscard]] float GetArea() const;
		void Grow(float size);
	};

	struct SpatialPlantGlobalParameters
	{
		/**
		 * \brief Weighting factor of asymmetrical competition.
		 */
		float m_p = 0.5f;
		/**
		 * \brief Delta of Richard growth model.
		 */
		float m_delta = 2;
		/**
		 * \brief Plant size factor.
		 */
		float m_a = 1.f;

		/**
		 * \brief Plant variety regulator
		 */
		float m_viabilityStrength = 1.0f;

		float m_spawnProtectionFactor = 0.5f;

		float m_maxRadius = 1000.0f;

		float m_seedingRadiusMin = 2.0f;

		float m_seedingRadiusMax = 3.0f;

		
	};

	class SpatialPlantDistribution {
	public:
		int m_simulationTime = 0;
		std::vector<SpatialPlantParameter> m_spatialPlantParameters {};
		std::vector<SpatialPlant> m_plants {};
		std::queue<SpatialPlantHandle> m_recycledPlants{};
		SpatialPlantGlobalParameters m_spatialPlantGlobalParameters{};
		[[nodiscard]] float CalculateGrowth(const SpatialPlantGlobalParameters &richardGrowthModelParameters, SpatialPlantHandle plantHandle, const std::vector<SpatialPlantHandle> &neighborPlantHandles) const;
		void Simulate();

		SpatialPlantHandle AddPlant(SpatialPlantParameterHandle spatialPlantParameterHandle, float radius, const glm::vec2 &position);
		void RecyclePlant(SpatialPlantHandle plantHandle);
	};
}