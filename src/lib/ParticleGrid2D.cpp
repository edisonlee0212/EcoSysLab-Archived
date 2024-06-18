#include "ParticleGrid2D.hpp"

#include "TreeVisualizer.hpp"

using namespace eco_sys_lab;

void ParticleCell::RegisterParticle(ParticleHandle handle) {
  m_atomHandles[m_atomCount] = handle;
  m_atomCount += m_atomCount < MAX_CELL_INDEX;
}

void ParticleCell::Clear() {
  m_atomCount = 0;
}

void ParticleCell::UnregisterParticle(ParticleHandle handle) {
  for (int i = 0; i < m_atomCount; i++) {
    if (m_atomHandles[i] == handle) {
      m_atomHandles[i] = m_atomHandles[m_atomCount - 1];
      m_atomCount--;
      return;
    }
  }
}

void ParticleGrid2D::ApplyBoundaries(const ProfileConstraints& profile_boundaries) {
  auto& cells = m_cells;
  if (profile_boundaries.boundaries.empty() && profile_boundaries.attractors.empty()) {
    for (int cell_index = 0; cell_index < m_cells.size(); cell_index++) {
      auto& cell = cells[cell_index];
      cell.m_target = -GetPosition(cell_index);
    }
  } else {
    for (int cell_index = 0; cell_index < m_cells.size(); cell_index++) {
      auto& cell = cells[cell_index];
      const auto cell_position = GetPosition(cell_index);
      cell.m_target = profile_boundaries.GetTarget(cell_position);
    }
  }
}

void ParticleGrid2D::Reset(const float cell_size, const glm::vec2& min_bound, const glm::ivec2& resolution) {
  m_resolution = resolution;
  m_cellSize = cell_size;
  m_minBound = min_bound;
  m_maxBound = min_bound + cell_size * glm::vec2(resolution);
  m_cells.resize(resolution.x * resolution.y);
  Clear();
}

void ParticleGrid2D::Reset(const float cell_size, const glm::vec2& min_bound, const glm::vec2& max_bound) {
  Reset(cell_size, min_bound,
        glm::ivec2(glm::ceil((max_bound.x - min_bound.x) / cell_size) + 1,
                   glm::ceil((max_bound.y - min_bound.y) / cell_size) + 1));
}

void ParticleGrid2D::RegisterParticle(const glm::vec2& position, ParticleHandle handle) {
  const auto coordinate =
      glm::ivec2(floor((position.x - m_minBound.x) / m_cellSize), floor((position.y - m_minBound.y) / m_cellSize));
  assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
  const auto cell_index = coordinate.x + coordinate.y * m_resolution.x;
  m_cells[cell_index].RegisterParticle(handle);
}

glm::ivec2 ParticleGrid2D::GetCoordinate(const glm::vec2& position) const {
  const auto coordinate =
      glm::ivec2(floor((position.x - m_minBound.x) / m_cellSize), floor((position.y - m_minBound.y) / m_cellSize));
  assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
  return coordinate;
}

glm::ivec2 ParticleGrid2D::GetCoordinate(const unsigned index) const {
  return {index % m_resolution.x, index / m_resolution.x};
}

ParticleCell& ParticleGrid2D::RefCell(const glm::vec2& position) {
  const auto coordinate =
      glm::ivec2(glm::clamp(static_cast<int>((position.x - m_minBound.x) / m_cellSize), 0, m_resolution.x - 1),
                 glm::clamp(static_cast<int>((position.y - m_minBound.y) / m_cellSize), 0, m_resolution.y - 1));
  const auto cell_index = coordinate.x + coordinate.y * m_resolution.x;
  return m_cells[cell_index];
}

ParticleCell& ParticleGrid2D::RefCell(const glm::ivec2& coordinate) {
  const auto cell_index = coordinate.x + coordinate.y * m_resolution.x;
  return m_cells[cell_index];
}

const std::vector<ParticleCell>& ParticleGrid2D::PeekCells() const {
  return m_cells;
}

glm::vec2 ParticleGrid2D::GetPosition(const glm::ivec2& coordinate) const {
  return m_minBound + m_cellSize * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

glm::vec2 ParticleGrid2D::GetPosition(const unsigned index) const {
  const auto coordinate = GetCoordinate(index);
  return m_minBound + m_cellSize * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

void ParticleGrid2D::Clear() {
  for (auto& cell : m_cells) {
    cell.m_atomCount = 0;
  }
}
