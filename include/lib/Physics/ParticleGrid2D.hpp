#pragma once
using namespace EvoEngine;
namespace EcoSysLab {
	typedef int ParticleHandle;

	class ParticleCell
	{
		template<typename PD>
		friend class ParticlePhysics2D;
		static constexpr size_t CELL_CAPACITY = 6;
		static constexpr size_t MAX_CELL_INDEX = CELL_CAPACITY - 1;
		size_t m_atomCount = 0;
		ParticleHandle m_atomHandles[CELL_CAPACITY] = {};
	public:
		void RegisterParticle(ParticleHandle handle);
		void Clear();
		void UnregisterParticle(ParticleHandle handle);
	};

	class ParticleGrid2D
	{
		glm::vec2 m_minBound = glm::vec2(0.0f);
		float m_cellSize = 1.0f;
		glm::ivec2 m_resolution = { 0, 0 };
		std::vector<ParticleCell> m_cells{};
		template<typename PD>
		friend class ParticlePhysics2D;
	public:
		ParticleGrid2D() = default;
		void Reset(float cellSize, const glm::vec2& minBound, const glm::ivec2& resolution);
		void Reset(float cellSize, const glm::vec2& minBound, const glm::vec2& maxBound);
		void RegisterParticle(const glm::vec2& position, ParticleHandle handle);
		[[nodiscard]] glm::ivec2 GetCoordinate(const glm::vec2& position) const;
		[[nodiscard]] ParticleCell& RefCell(const glm::vec2& position);
		[[nodiscard]] ParticleCell& RefCell(const glm::ivec2& coordinate);
		void Clear();
	};
}