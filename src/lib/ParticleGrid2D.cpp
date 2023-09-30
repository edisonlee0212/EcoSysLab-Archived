#include "ParticleGrid2D.hpp"

using namespace EcoSysLab;

void ParticleCell::RegisterParticle(const ParticleHandle handle)
{
	m_atomHandles[m_atomCount] = handle;
	m_atomCount += m_atomCount < MAX_CELL_INDEX;
}

void ParticleCell::Clear()
{
	m_atomCount = 0;
}

void ParticleCell::UnregisterParticle(const ParticleHandle handle)
{
	for(int i = 0; i < m_atomCount; i++)
	{
		if(m_atomHandles[i] == handle)
		{
			m_atomHandles[i] = m_atomHandles[m_atomCount - 1];
			m_atomCount--;
			return;
		}
	}
}

void ParticleGrid2D::Reset(const size_t width, const  size_t height)
{
	m_width = width;
	m_height = height;
	m_cells.resize(m_width * m_height);
	std::memset(m_cells.data(), 0, m_cells.size() * sizeof(ParticleCell));
}

void ParticleGrid2D::RegisterParticle(const size_t x, const size_t y, const ParticleHandle handle)
{
	assert(x < m_width && y < m_height);
	const auto cellIndex = x * m_height + y;
	m_cells[cellIndex].RegisterParticle(handle);
}

void ParticleGrid2D::Clear()
{
	std::memset(m_cells.data(), 0, m_cells.size() * sizeof(ParticleCell));
}


