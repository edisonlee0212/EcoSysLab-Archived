#pragma once
#include "CellGrid.hpp"
#include "ProfileConstraints.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
typedef int ParticleHandle;
class ParticleCell {
  template <typename PD>
  friend class StrandModelProfile;
  friend class ParticleGrid2D;
  static constexpr size_t CELL_CAPACITY = 4;
  static constexpr size_t MAX_CELL_INDEX = CELL_CAPACITY - 1;
  size_t m_atomCount = 0;
  ParticleHandle m_atomHandles[CELL_CAPACITY] = {};

 public:
  glm::vec2 m_target = glm::vec2(0.0f);
  void RegisterParticle(ParticleHandle handle);
  void Clear();
  void UnregisterParticle(ParticleHandle handle);
};

class ParticleGrid2D {
  glm::vec2 m_minBound = glm::vec2(0.0f);
  glm::vec2 m_maxBound = glm::vec2(0.0f);
  float m_cellSize = 1.0f;
  glm::ivec2 m_resolution = {0, 0};
  std::vector<ParticleCell> m_cells{};
  template <typename PD>
  friend class StrandModelProfile;

 public:
  void ApplyBoundaries(const ProfileConstraints& profile_boundaries);
  ParticleGrid2D() = default;
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::ivec2& resolution);
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::vec2& max_bound);
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

}  // namespace eco_sys_lab
