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
	for (int i = 0; i < m_atomCount; i++)
	{
		if (m_atomHandles[i] == handle)
		{
			m_atomHandles[i] = m_atomHandles[m_atomCount - 1];
			m_atomCount--;
			return;
		}
	}
}

void ParticleGrid2D::Reset(float cellSize, const glm::vec2& minBound, const glm::vec2& maxBound)
{
	Reset(cellSize, minBound,
		glm::ivec2(glm::ceil((maxBound.x - minBound.x) / cellSize) + 1,
			glm::ceil((maxBound.y - minBound.y) / cellSize) + 1));
}

void ParticleGrid2D::RegisterParticle(const glm::vec2& position, const ParticleHandle handle)
{
	const auto coordinate = glm::ivec2(
		floor((position.x - m_minBound.x) / m_cellSize),
		floor((position.y - m_minBound.y) / m_cellSize));
	assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
	const auto cellIndex = coordinate.x + coordinate.y * m_resolution.x;
	m_cells[cellIndex].RegisterParticle(handle);
}

glm::ivec2 ParticleGrid2D::GetCoordinate(const glm::vec2& position) const
{
	const auto coordinate = glm::ivec2(
		floor((position.x - m_minBound.x) / m_cellSize),
		floor((position.y - m_minBound.y) / m_cellSize));
	assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
	return coordinate;
}

ParticleCell& ParticleGrid2D::RefCell(const glm::vec2& position)
{
	const auto coordinate = glm::ivec2(
		floor((position.x - m_minBound.x) / m_cellSize),
		floor((position.y - m_minBound.y) / m_cellSize));
	assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
	const auto cellIndex = coordinate.x + coordinate.y * m_resolution.x;
	return m_cells[cellIndex];
}

ParticleCell& ParticleGrid2D::RefCell(const glm::ivec2& coordinate)
{
	assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
	const auto cellIndex = coordinate.x + coordinate.y * m_resolution.x;
	return m_cells[cellIndex];
}

void ParticleGrid2D::Reset(const float cellSize, const glm::vec2& minBound, const glm::ivec2& resolution)
{
	m_resolution = resolution;
	m_cellSize = cellSize;
	m_minBound = minBound;
	m_cells.resize(resolution.x * resolution.y);
	Clear();
}

void ParticleGrid2D::Clear()
{
	std::memset(m_cells.data(), 0, m_cells.size() * sizeof(ParticleCell));
}


