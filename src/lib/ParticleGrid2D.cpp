#include "ParticleGrid2D.hpp"

#include "TreeVisualizer.hpp"

using namespace EcoSysLab;

void ParticleCell::RegisterParticle(ParticleHandle handle)
{
	m_atomHandles[m_atomCount] = handle;
	m_atomCount += m_atomCount < MAX_CELL_INDEX;
}

void ParticleCell::Clear()
{
	m_atomCount = 0;
}

void ParticleCell::UnregisterParticle(ParticleHandle handle)
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

void ParticleGrid2D::ApplyBoundaries(const ProfileBoundaries& profileBoundaries)
{
	auto& cells = m_cells;
	if (profileBoundaries.m_boundaries.empty())
	{
		for (int cellIndex = 0; cellIndex < m_cellSize; cellIndex++) {
			auto& cell = cells[cellIndex];
			cell.m_boundaryIndex = -1;
		}
	}
	else {
		for (int cellIndex = 0; cellIndex < m_cellSize; cellIndex++) {
			auto& cell = cells[cellIndex];
			const auto cellPosition = GetPosition(cellIndex);
			cell.m_boundaryIndex = profileBoundaries.InBoundaries(cellPosition, cell.m_closestPoint);
		}
	}
}


void ParticleGrid2D::Reset(float cellSize, const glm::vec2& minBound, const glm::ivec2& resolution)
{
	m_resolution = resolution;
	m_cellSize = cellSize;
	m_minBound = minBound;
	m_maxBound = minBound + cellSize * glm::vec2(resolution);
	m_cells.resize(resolution.x * resolution.y);
	Clear();
}

void ParticleGrid2D::Reset(float cellSize, const glm::vec2& minBound, const glm::vec2& maxBound)
{
	Reset(cellSize, minBound,
		glm::ivec2(glm::ceil((maxBound.x - minBound.x) / cellSize) + 1,
			glm::ceil((maxBound.y - minBound.y) / cellSize) + 1));
}

void ParticleGrid2D::RegisterParticle(const glm::vec2& position, ParticleHandle handle)
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

glm::ivec2 ParticleGrid2D::GetCoordinate(const unsigned index) const
{
	return { index % m_resolution.x, index / m_resolution.x };
}

ParticleCell& ParticleGrid2D::RefCell(const glm::vec2& position)
{
	const auto coordinate = glm::ivec2(
		glm::clamp(static_cast<int>((position.x - m_minBound.x) / m_cellSize), 0, m_resolution.x - 1),
		glm::clamp(static_cast<int>((position.y - m_minBound.y) / m_cellSize), 0, m_resolution.y - 1));
	const auto cellIndex = coordinate.x + coordinate.y * m_resolution.x;
	return m_cells[cellIndex];
}

ParticleCell& ParticleGrid2D::RefCell(const glm::ivec2& coordinate)
{
	const auto cellIndex = coordinate.x + coordinate.y * m_resolution.x;
	return m_cells[cellIndex];
}

const std::vector<ParticleCell>& ParticleGrid2D::PeekCells() const
{
	return m_cells;
}

glm::vec2 ParticleGrid2D::GetPosition(const glm::ivec2& coordinate) const
{
	return m_minBound + m_cellSize * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

glm::vec2 ParticleGrid2D::GetPosition(const unsigned index) const
{
	const auto coordinate = GetCoordinate(index);
	return m_minBound + m_cellSize * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

void ParticleGrid2D::Clear()
{
	for (auto& cell : m_cells)
	{
		cell.m_atomCount = 0;
	}
}
