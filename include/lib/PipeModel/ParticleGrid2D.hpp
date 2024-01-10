#pragma once
#include "ProfileBoundaries.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	typedef int ParticleHandle;
	class ParticleCell
	{
		template<typename PD>
		friend class ParticlePhysics2D;
		friend class ParticleGrid2D;
		static constexpr size_t CELL_CAPACITY = 6;
		static constexpr size_t MAX_CELL_INDEX = CELL_CAPACITY - 1;
		size_t m_atomCount = 0;
		ParticleHandle m_atomHandles[CELL_CAPACITY] = {};
	public:
		int m_boundaryIndex = -1;
		glm::vec2 m_closestPoint = glm::vec2(0.0f);
		void RegisterParticle(ParticleHandle handle);
		void Clear();
		void UnregisterParticle(ParticleHandle handle);
	};


	class ParticleGrid2D
	{
		glm::vec2 m_minBound = glm::vec2(0.0f);
		glm::vec2 m_maxBound = glm::vec2(0.0f);
		float m_cellSize = 1.0f;
		glm::ivec2 m_resolution = { 0, 0 };
		std::vector<ParticleCell> m_cells{};
		template<typename PD>
		friend class ParticlePhysics2D;
	public:
		void ApplyBoundaries(const ProfileBoundaries& profileBoundaries);
		ParticleGrid2D() = default;
		void Reset(float cellSize, const glm::vec2& minBound, const glm::ivec2& resolution);
		void Reset(float cellSize, const glm::vec2& minBound, const glm::vec2& maxBound);
		void RegisterParticle(const glm::vec2& position, ParticleHandle handle);
		[[nodiscard]] glm::ivec2 GetCoordinate(const glm::vec2& position) const;
		[[nodiscard]] glm::ivec2 GetCoordinate(unsigned index) const;
		[[nodiscard]] ParticleCell& RefCell(const glm::vec2& position);
		[[nodiscard]] ParticleCell& RefCell(const glm::ivec2& coordinate);
		[[nodiscard]] const std::vector<ParticleCell>& PeekCells() const;
		[[nodiscard]] glm::vec2 GetPosition(const glm::ivec2& coordinate) const;
		[[nodiscard]] glm::vec2 GetPosition(unsigned index) const;
		void Clear();
	};

	
}
