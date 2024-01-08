#include "ParticleGrid2D.hpp"

#include "TreeVisualizer.hpp"

using namespace EcoSysLab;

void ParticleCell::RegisterParticle(ParticleHandle handle)
{
	m_atomHandles[m_atomCount] = handle;
	m_atomCount += m_atomCount < ParticleCell::MAX_CELL_INDEX;
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

void ParticleGrid2D::TestBoundaries(const std::vector<std::vector<glm::vec2>>& boundaries)
{
	auto& cells = m_cells;
	if (boundaries.empty())
	{
		Jobs::ParallelFor(cells.size(), [&](unsigned cellIndex)
			{
				auto& cell = cells[cellIndex];
				cell.m_inBoundary = true;
			});
	}
	else {
		Jobs::ParallelFor(cells.size(), [&](unsigned cellIndex)
			{
				auto& cell = cells[cellIndex];
				cell.m_inBoundary = false;
				const auto cellPosition = GetPosition(cellIndex);
				auto closestPoint = glm::vec2(0.0f);
				auto distance = FLT_MAX;
				for (const auto& boundary : boundaries) {
					int intersectCount1 = 0;
					int intersectCount2 = 0;
					int intersectCount3 = 0;
					int intersectCount4 = 0;
					for (int lineIndex = 0; lineIndex < boundary.size(); lineIndex++) {
						const auto& p1 = boundary[lineIndex];
						const auto& p2 = boundary[(lineIndex + 1) % boundary.size()];
						const auto p3 = cellPosition;
						const auto p41 = cellPosition + glm::vec2(1000.0f, 0.0f);
						const auto p42 = cellPosition + glm::vec2(-1000.0f, 0.0f);
						const auto p43 = cellPosition + glm::vec2(0.0f, 1000.0f);
						const auto p44 = cellPosition + glm::vec2(0.0f, -1000.0f);
						if (TreeVisualizer::intersect(p1, p2, p3, p41)) {
							intersectCount1++;
						}
						if (TreeVisualizer::intersect(p1, p2, p3, p42)) {
							intersectCount2++;
						}
						if (TreeVisualizer::intersect(p1, p2, p3, p43)) {
							intersectCount3++;
						}
						if (TreeVisualizer::intersect(p1, p2, p3, p44)) {
							intersectCount4++;
						}
						const auto testPoint = glm::closestPointOnLine(cellPosition, p1, p2);
						const auto newDistance = glm::distance(testPoint, cellPosition);
						if (distance > newDistance)
						{
							closestPoint = testPoint;
							distance = newDistance;
						}
					}
					cell.m_inBoundary = cell.m_inBoundary ||
						(intersectCount1 % 2 != 0
							&& intersectCount2 % 2 != 0
							&& intersectCount3 % 2 != 0
							&& intersectCount4 % 2 != 0);
				}
				cell.m_closestPoint = closestPoint;
			}
		);
	}
}


void ParticleGrid2D::Reset(float cellSize, const glm::vec2& minBound, const glm::ivec2& resolution)
{
	m_resolution = resolution;
	m_cellSize = cellSize;
	m_minBound = minBound;
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
	assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
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
	std::memset(m_cells.data(), 0, m_cells.size() * sizeof(ParticleCell));
}
