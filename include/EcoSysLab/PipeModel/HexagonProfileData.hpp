#pragma once

#include "ecosyslab_export.h"
#include "PipeGroup.hpp"
#include "PipeProfile.hpp"
#include "Skeleton.hpp"

namespace EcoSysLab
{
	enum class HexagonProfileDirection
	{
		UpLeft,
		UpRight,
		Right,
		DownRight,
		DownLeft,
		Left
	};

	struct HexagonCellData
	{
		CellHandle m_upLeft = -1;
		CellHandle m_upRight = -1;
		CellHandle m_right = -1;
		CellHandle m_downRight = -1;
		CellHandle m_downLeft = -1;
		CellHandle m_left = -1;

		glm::ivec2 m_coordinate = glm::ivec2(0);

		PipeHandle m_pipeHandle = -1;

		[[nodiscard]] glm::ivec2 GetCoordinate() const;

		[[nodiscard]] glm::ivec2 GetUpLeftCoordinate() const;

		[[nodiscard]] glm::ivec2 GetUpRightCoordinate() const;

		[[nodiscard]] glm::ivec2 GetRightCoordinate() const;

		[[nodiscard]] glm::ivec2 GetDownRightCoordinate() const;

		[[nodiscard]] glm::ivec2 GetDownLeftCoordinate() const;

		[[nodiscard]] glm::ivec2 GetLeftCoordinate() const;

		[[nodiscard]] CellHandle GetUpLeftHandle() const;

		[[nodiscard]] CellHandle GetUpRightHandle() const;

		[[nodiscard]] CellHandle GetRightHandle() const;

		[[nodiscard]] CellHandle GetDownRightHandle() const;

		[[nodiscard]] CellHandle GetDownLeftHandle() const;

		[[nodiscard]] CellHandle GetLeftHandle() const;

		[[nodiscard]] bool IsBoundary() const;
	};

	struct HexagonProfileData
	{
		std::map<std::pair<int, int>, CellHandle> m_cellMap;
		std::set<CellHandle> m_boundary;
		NodeHandle m_nodeHandle = -1;
		friend class HexagonProfileGroup;
		
		[[nodiscard]] CellHandle GetCellHandle(const glm::ivec2& coordinate) const;

		[[nodiscard]] static bool CheckBoundary(const std::vector<glm::vec2>& points);
		
		static bool RayLineIntersect(const glm::vec2 &rayOrigin, const glm::vec2& rayDirection, const glm::vec2& point1, const glm::vec2& point2);
		static bool InBoundary(const std::vector<glm::vec2>& boundary, const glm::vec2& point);
		static bool LineLineIntersect(const glm::vec2& pa, const glm::vec2& pb, const glm::vec2& pc, const glm::vec2& pd);
	};

	/*
	class HexagonProfile : PipeProfile<HexagonProfileData, HexagonCellData>
	{
	public:
		void Construct(const std::vector<glm::vec2>& points);

		void Copy(const HexagonProfile& src);
		void ShiftCoordinate(const glm::ivec2& offset);

		[[nodiscard]] glm::vec2 GetPosition(ProfileHandle handle) const;

		[[nodiscard]] glm::vec2 GetPosition(const glm::ivec2& coordinate) const;

		[[nodiscard]] glm::ivec2 GetCoordinate(const glm::vec2& position) const;

		[[nodiscard]] CellHandle Allocate(const glm::vec2& position);

		[[nodiscard]] CellHandle Allocate(const glm::ivec2& coordinate);

		void RemoveCell(CellHandle handle);

		[[nodiscard]] const std::set<CellHandle>& GetBoundary() const;

		[[nodiscard]] glm::ivec2 FindAvailableCoordinate(CellHandle targetHandle, const glm::vec2& direction);

	};
	*/
}
