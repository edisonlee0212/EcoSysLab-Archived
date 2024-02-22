#pragma once
#include "Jobs.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	/* Coordinate system

	The cell position is its center.
	Each cell is dx wide.

					<-dx ->
					-------------------------
					|     |     |     |     |
					|  x  |  x  |  x  |  x  |
					|     |     |     |     |
					-------------------------
					   |     |     |     |
					   |     |     |     |
	X-Coordinate:   -- 0 --- 1 --- 2 --- 3 -----

	The "m_minBound" stores the lower left corner of the lower left cell.
	I.e. for m_minBound = (0, 0) and m_resolution = (2, 2), and m_size = 1,
	the cell centers are at 0.5 and 1.5.

	*/

	template <typename CellData>
	class CellGrid
	{
		glm::vec2 m_minBound = glm::vec2(0.0f);
		glm::vec2 m_maxBound = glm::vec2(0.0f);
		float m_cellSize = 1.0f;
		glm::ivec2 m_resolution = { 0, 0 };
		std::vector<CellData> m_cells{};
	public:
		virtual ~CellGrid() = default;
		[[nodiscard]] glm::vec2 GetMinBound() const;
		[[nodiscard]] glm::vec2 GetMaxBound() const;
		[[nodiscard]] float GetCellSize() const;
		[[nodiscard]] glm::ivec2 GetResolution() const;
		CellGrid() = default;
		void Reset(float cellSize, const glm::vec2& minBound, const glm::ivec2& resolution);
		void Reset(float cellSize, const glm::vec2& minBound, const glm::vec2& maxBound);
		[[nodiscard]] glm::ivec2 GetCoordinate(const glm::vec2& position) const;
		[[nodiscard]] glm::ivec2 GetCoordinate(unsigned index) const;
		[[nodiscard]] CellData& RefCell(const glm::vec2& position);
		[[nodiscard]] CellData& RefCell(const glm::ivec2& coordinate);
		[[nodiscard]] CellData& RefCell(unsigned index);
		[[nodiscard]] const std::vector<CellData>& PeekCells() const;
		[[nodiscard]] std::vector<CellData>& RefCells();
		[[nodiscard]] glm::vec2 GetPosition(const glm::ivec2& coordinate) const;
		[[nodiscard]] glm::vec2 GetPosition(unsigned index) const;

		void ForEach(const glm::vec2& position, float radius, const std::function<void(CellData& data)>& func);
		virtual void Clear() = 0;
	};

	template <typename CellData>
	glm::vec2 CellGrid<CellData>::GetMinBound() const
	{
		return m_minBound;
	}

	template <typename CellData>
	glm::vec2 CellGrid<CellData>::GetMaxBound() const
	{
		return m_maxBound;
	}

	template <typename CellData>
	float CellGrid<CellData>::GetCellSize() const
	{
		return m_cellSize;
	}

	template <typename CellData>
	glm::ivec2 CellGrid<CellData>::GetResolution() const
	{
		return m_resolution;
	}

	template <typename CellData>
	void CellGrid<CellData>::Reset(const float cellSize, const glm::vec2& minBound, const glm::ivec2& resolution)
	{
		m_resolution = resolution;
		m_cellSize = cellSize;
		m_minBound = minBound;
		m_maxBound = minBound + cellSize * glm::vec2(resolution);
		m_cells.resize(resolution.x * resolution.y);
	}

	template <typename CellData>
	void CellGrid<CellData>::Reset(const float cellSize, const glm::vec2& minBound, const glm::vec2& maxBound)
	{
		Reset(cellSize, minBound,
			glm::ivec2(glm::ceil((maxBound.x - minBound.x) / cellSize) + 1,
				glm::ceil((maxBound.y - minBound.y) / cellSize) + 1));
	}

	template <typename CellData>
	glm::ivec2 CellGrid<CellData>::GetCoordinate(const glm::vec2& position) const
	{
		const auto coordinate = glm::ivec2(
			floor((position.x - m_minBound.x) / m_cellSize),
			floor((position.y - m_minBound.y) / m_cellSize));
		assert(coordinate.x < m_resolution.x && coordinate.y < m_resolution.y);
		return coordinate;
	}

	template <typename CellData>
	glm::ivec2 CellGrid<CellData>::GetCoordinate(const unsigned index) const
	{
		return { index % m_resolution.x, index / m_resolution.x };
	}

	template <typename CellData>
	CellData& CellGrid<CellData>::RefCell(const glm::vec2& position)
	{
		const auto coordinate = glm::ivec2(
			glm::clamp(static_cast<int>((position.x - m_minBound.x) / m_cellSize), 0, m_resolution.x - 1),
			glm::clamp(static_cast<int>((position.y - m_minBound.y) / m_cellSize), 0, m_resolution.y - 1));
		const auto cellIndex = coordinate.x + coordinate.y * m_resolution.x;
		return m_cells[cellIndex];
	}

	template <typename CellData>
	CellData& CellGrid<CellData>::RefCell(const glm::ivec2& coordinate)
	{
		const auto cellIndex = coordinate.x + coordinate.y * m_resolution.x;
		return m_cells[cellIndex];
	}

	template <typename CellData>
	CellData& CellGrid<CellData>::RefCell(const unsigned index)
	{
		return m_cells[index];
	}

	template <typename CellData>
	const std::vector<CellData>& CellGrid<CellData>::PeekCells() const
	{
		return m_cells;
	}

	template <typename CellData>
	std::vector<CellData>& CellGrid<CellData>::RefCells()
	{
		return m_cells;
	}

	template <typename CellData>
	glm::vec2 CellGrid<CellData>::GetPosition(const glm::ivec2& coordinate) const
	{
		return m_minBound + m_cellSize * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
	}

	template <typename CellData>
	glm::vec2 CellGrid<CellData>::GetPosition(const unsigned index) const
	{
		const auto coordinate = GetCoordinate(index);
		return m_minBound + m_cellSize * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
	}

	template <typename CellData>
	void CellGrid<CellData>::ForEach(const glm::vec2& position, const float radius,
		const std::function<void(CellData& data)>& func)
	{
		const auto actualCenter = position - m_minBound;
		const auto actualMinBound = actualCenter - glm::vec2(radius);
		const auto actualMaxBound = actualCenter + glm::vec2(radius);
		const auto start = glm::ivec2(glm::floor(actualMinBound / glm::vec2(m_cellSize)));
		const auto end = glm::ivec2(glm::ceil(actualMaxBound / glm::vec2(m_cellSize)));
		for (int i = start.x; i <= end.x; i++) {
			for (int j = start.y; j <= end.y; j++) {
				if (i < 0 || i >= m_resolution.x || j < 0 || j >= m_resolution.y) continue;
				func(RefCell(glm::ivec2(i, j)));
			}
		}
	}
}
