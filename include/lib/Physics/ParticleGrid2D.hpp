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
		size_t m_width = 0;
		size_t m_height = 0;
		std::vector<ParticleCell> m_cells{};
		template<typename PD>
		friend class ParticlePhysics2D;
	public:
		ParticleGrid2D() = default;
		void Reset(size_t width, size_t height);
		void RegisterParticle(size_t x, size_t y, ParticleHandle handle);
		ParticleCell& RefCell(size_t x, size_t y);
		void Clear();
	};
}