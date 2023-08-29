#include "HexagonProfileData.hpp"

using namespace EcoSysLab;

glm::ivec2 HexagonCellData::GetCoordinate() const
{
	return m_coordinate;
}


glm::ivec2 HexagonCellData::GetUpLeftCoordinate() const
{
	return { m_coordinate.x - 1, m_coordinate.y + 1 };
}

glm::ivec2 HexagonCellData::GetUpRightCoordinate() const
{
	return { m_coordinate.x, m_coordinate.y + 1 };
}

glm::ivec2 HexagonCellData::GetRightCoordinate() const
{
	return { m_coordinate.x + 1, m_coordinate.y };
}

glm::ivec2 HexagonCellData::GetDownRightCoordinate() const
{
	return { m_coordinate.x + 1, m_coordinate.y - 1 };
}

glm::ivec2 HexagonCellData::GetDownLeftCoordinate() const
{
	return { m_coordinate.x, m_coordinate.y - 1 };
}

glm::ivec2 HexagonCellData::GetLeftCoordinate() const
{
	return { m_coordinate.x - 1, m_coordinate.y };
}


CellHandle HexagonCellData::GetUpLeftHandle() const
{
	return m_upLeft;
}

CellHandle HexagonCellData::GetUpRightHandle() const
{
	return m_upRight;
}

CellHandle HexagonCellData::GetRightHandle() const
{
	return m_right;
}


CellHandle HexagonCellData::GetDownRightHandle() const
{
	return m_downRight;
}


CellHandle HexagonCellData::GetDownLeftHandle() const
{
	return m_downLeft;
}


CellHandle HexagonCellData::GetLeftHandle() const
{
	return m_left;
}


bool HexagonCellData::IsBoundary() const
{
	return m_downLeft == -1 || m_downRight == -1 || m_left == -1 || m_right == -1 || m_upLeft == -1 || m_upRight == -1;
}



CellHandle HexagonProfileData::GetCellHandle(const glm::ivec2& coordinate) const
{
	const auto search = m_cellMap.find({ coordinate.x, coordinate.y });
	if (search != m_cellMap.end()) return search->second;
	return -1;
}
/*
void HexagonProfile::ShiftCoordinate(const glm::ivec2& offset)
{
	for (auto& cell : RefCells())
	{
		cell.m_data.m_coordinate += offset;
	}
	std::map<std::pair<int, int>, CellHandle> newMap;
	for (const auto& i : m_data.m_cellMap)
	{
		newMap[std::make_pair(i.first.first + offset.x, i.first.second + offset.y)] = i.second;
	}
	m_data.m_cellMap = newMap;
}


glm::vec2 HexagonProfile::GetPosition(ProfileHandle handle) const
{
	assert(PeekCells().size() > handle && !PeekCells()[handle].IsRecycled());
	return GetPosition(PeekCells()[handle].m_data.GetCoordinate());
}

glm::vec2 HexagonProfile::GetPosition(const glm::ivec2& coordinate) const
{
	return { coordinate.x + coordinate.y / 2.0f,
		coordinate.y * glm::cos(glm::radians(30.0f)) };
}


glm::ivec2 HexagonProfile::GetCoordinate(const glm::vec2& position) const
{
	int y = glm::round(position.y / glm::cos(glm::radians(30.0f)));
	return { glm::round((position.x - y / 2.0f)), y };
}


CellHandle HexagonProfile::Allocate(const glm::vec2& position)
{
	return Allocate(GetCoordinate(position));
}


CellHandle HexagonProfile::Allocate(const glm::ivec2& coordinate)
{
	if (m_data.m_cellMap.find({ coordinate.x, coordinate.y }) != m_data.m_cellMap.end())
	{
		EVOENGINE_ERROR("Cell exists!");
		return -1;
	}
	auto newCellHandle = AllocateCell();
	auto& newCell = RefCell(newCellHandle);

	newCell.m_data.m_coordinate = coordinate;
	m_data.m_cellMap[{ coordinate.x, coordinate.y }] = newCellHandle;
	const auto coordinate1 = newCell.m_data.GetLeftCoordinate();
	auto search1 = m_data.m_cellMap.find({ coordinate1.x, coordinate1.y });
	if (search1 != m_data.m_cellMap.end())
	{
		newCell.m_data.m_left = search1->second;
		auto& targetCell = RefCells()[search1->second];
		targetCell.m_data.m_right = newCellHandle;
		if (!targetCell.m_data.IsBoundary())
		{
			m_data.m_boundary.erase(search1->second);
		}
	}
	const auto coordinate2 = newCell.m_data.GetUpLeftCoordinate();
	auto search2 = m_data.m_cellMap.find({ coordinate2.x, coordinate2.y });
	if (search2 != m_data.m_cellMap.end())
	{
		newCell.m_data.m_upLeft = search2->second;
		auto& targetCell = RefCells()[search2->second];
		targetCell.m_data.m_downRight = newCellHandle;
		if (!targetCell.m_data.IsBoundary())
		{
			m_data.m_boundary.erase(search2->second);
		}
	}
	const auto coordinate3 = newCell.m_data.GetUpRightCoordinate();
	auto search3 = m_data.m_cellMap.find({ coordinate3.x, coordinate3.y });
	if (search3 != m_data.m_cellMap.end())
	{
		newCell.m_data.m_upRight = search3->second;
		auto& targetCell = RefCells()[search3->second];
		targetCell.m_data.m_downLeft = newCellHandle;
		if (!targetCell.m_data.IsBoundary())
		{
			m_data.m_boundary.erase(search3->second);
		}
	}

	const auto coordinate4 = newCell.m_data.GetRightCoordinate();
	auto search4 = m_data.m_cellMap.find({ coordinate4.x, coordinate4.y });
	if (search4 != m_data.m_cellMap.end())
	{
		newCell.m_data.m_right = search4->second;
		auto& targetCell = RefCells()[search4->second];
		targetCell.m_data.m_left = newCellHandle;
		if (!targetCell.m_data.IsBoundary())
		{
			m_data.m_boundary.erase(search4->second);
		}
	}

	const auto coordinate5 = newCell.m_data.GetDownRightCoordinate();
	auto search5 = m_data.m_cellMap.find({ coordinate5.x, coordinate5.y });
	if (search5 != m_data.m_cellMap.end())
	{
		newCell.m_data.m_downRight = search5->second;
		auto& targetCell = RefCells()[search5->second];
		targetCell.m_data.m_upLeft = newCellHandle;
		if (!targetCell.m_data.IsBoundary())
		{
			m_data.m_boundary.erase(search5->second);
		}
	}

	const auto coordinate6 = newCell.m_data.GetDownLeftCoordinate();
	auto search6 = m_data.m_cellMap.find({ coordinate6.x, coordinate6.y });
	if (search6 != m_data.m_cellMap.end())
	{
		newCell.m_data.m_downLeft = search6->second;
		auto& targetCell = RefCells()[search6->second];
		targetCell.m_data.m_upRight = newCellHandle;
		if (!targetCell.m_data.IsBoundary())
		{
			m_data.m_boundary.erase(search6->second);
		}
	}
	if (newCell.m_data.IsBoundary()) m_data.m_boundary.insert(newCellHandle);
	return newCellHandle;
}


void HexagonProfile::RemoveCell(CellHandle handle)
{
	auto& cell = RefCells()[handle];
	assert(!cell.IsRecycled());
	auto coordinate = cell.m_data.m_coordinate;
	m_data.m_cellMap.erase({ coordinate.x, coordinate.y });

	if (cell.m_data.m_upLeft != -1) {
		auto& targetCell = RefCells()[cell.m_data.m_upLeft];
		targetCell.m_data.m_downRight = -1;
		if (targetCell.m_data.IsBoundary()) m_data.m_boundary.insert(cell.m_data.m_upLeft);
	}
	if (cell.m_data.m_upRight != -1) {
		auto& targetCell = RefCells()[cell.m_data.m_upRight];
		targetCell.m_data.m_downLeft = -1;
		if (targetCell.m_data.IsBoundary()) m_data.m_boundary.insert(cell.m_data.m_upRight);
	}
	if (cell.m_data.m_right != -1) {
		auto& targetCell = RefCells()[cell.m_data.m_right];
		targetCell.m_data.m_left = -1;
		if (targetCell.m_data.IsBoundary()) m_data.m_boundary.insert(cell.m_data.m_right);
	}
	if (cell.m_data.m_downRight != -1) {
		auto& targetCell = RefCells()[cell.m_data.m_downRight];
		targetCell.m_data.m_upLeft = -1;
		if (targetCell.m_data.IsBoundary()) m_data.m_boundary.insert(cell.m_data.m_downRight);
	}
	if (cell.m_data.m_downLeft != -1) {
		auto& targetCell = RefCells()[cell.m_data.m_downLeft];
		targetCell.m_data.m_upRight = -1;
		if (targetCell.m_data.IsBoundary()) m_data.m_boundary.insert(cell.m_data.m_downLeft);
	}
	if (cell.m_data.m_left != -1) {
		auto& targetCell = RefCells()[cell.m_data.m_left];
		targetCell.m_data.m_right = -1;
		if (targetCell.m_data.IsBoundary()) m_data.m_boundary.insert(cell.m_data.m_left);
	}

	cell.m_data.m_upLeft = cell.m_data.m_upRight = cell.m_data.m_right = cell.m_data.m_downRight = cell.m_data.m_downLeft = cell.m_data.m_left = -1;

	cell.m_data.m_coordinate = glm::ivec2(0);

	m_data.m_boundary.erase(handle);

	RecycleCell(handle);
}
const std::set<CellHandle>& HexagonProfile::GetBoundary() const
{
	return m_data.m_boundary;
}


glm::ivec2 HexagonProfile::FindAvailableCoordinate(CellHandle targetHandle,
	const glm::vec2& direction)
{
	const auto& cell = PeekCells()[targetHandle];
	const auto angle = glm::degrees(glm::abs(glm::atan(direction.y / direction.x)));

	HexagonProfileDirection hexDirection;
	if (angle > 30.0f)
	{
		if (direction.y >= 0) {
			if (direction.x >= 0) {
				hexDirection = HexagonProfileDirection::UpRight;
			}
			else
			{
				hexDirection = HexagonProfileDirection::UpLeft;
			}
		}
		else
		{
			if (direction.x >= 0) {
				hexDirection = HexagonProfileDirection::DownRight;
			}
			else
			{
				hexDirection = HexagonProfileDirection::DownLeft;
			}
		}
	}
	else
	{
		if (direction.x >= 0)
		{
			hexDirection = HexagonProfileDirection::Right;
		}
		else
		{
			hexDirection = HexagonProfileDirection::Left;
		}
	}
	bool found = false;
	auto currentHandle = targetHandle;
	glm::ivec2 coordinate = cell.m_data.m_coordinate;
	while (!found) {
		auto& currentCell = RefCells()[currentHandle];
		switch (hexDirection)
		{
		case HexagonProfileDirection::UpLeft:
		{
			if (currentCell.m_data.GetLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetLeftCoordinate();
			}
			else if (currentCell.m_data.GetUpLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetUpLeftCoordinate();
			}
			else if (currentCell.m_data.GetUpRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetUpRightCoordinate();
			}
			else
			{
				const auto pos1 = GetPosition(currentCell.m_data.GetLeftCoordinate());
				const auto pos2 = GetPosition(currentCell.m_data.GetUpLeftCoordinate());
				const auto pos3 = GetPosition(currentCell.m_data.GetUpRightCoordinate());
				const auto distance1 = glm::distance(pos1, glm::closestPointOnLine(pos1, glm::vec2(0), direction * 10000.0f));
				const auto distance2 = glm::distance(pos2, glm::closestPointOnLine(pos2, glm::vec2(0), direction * 10000.0f));
				const auto distance3 = glm::distance(pos3, glm::closestPointOnLine(pos3, glm::vec2(0), direction * 10000.0f));

				if (distance1 <= distance2 && distance1 <= distance3)
				{
					currentHandle = currentCell.m_data.GetLeftHandle();
				}
				else if (distance2 <= distance1 && distance2 <= distance3)
				{
					currentHandle = currentCell.m_data.GetUpLeftHandle();
				}
				else if (distance3 <= distance1 && distance3 <= distance2)
				{
					currentHandle = currentCell.m_data.GetUpRightHandle();
				}
			}
		}
		break;
		case HexagonProfileDirection::UpRight:
		{
			if (currentCell.m_data.GetUpLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetUpLeftCoordinate();
			}
			else if (currentCell.m_data.GetUpRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetUpRightCoordinate();
			}
			else if (currentCell.m_data.GetRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetRightCoordinate();
			}
			else
			{
				const auto pos1 = GetPosition(currentCell.m_data.GetUpLeftCoordinate());
				const auto pos2 = GetPosition(currentCell.m_data.GetUpRightCoordinate());
				const auto pos3 = GetPosition(currentCell.m_data.GetRightCoordinate());
				const auto distance1 = glm::distance(pos1, glm::closestPointOnLine(pos1, glm::vec2(0), direction * 10000.0f));
				const auto distance2 = glm::distance(pos2, glm::closestPointOnLine(pos2, glm::vec2(0), direction * 10000.0f));
				const auto distance3 = glm::distance(pos3, glm::closestPointOnLine(pos3, glm::vec2(0), direction * 10000.0f));

				if (distance1 <= distance2 && distance1 <= distance3)
				{
					currentHandle = currentCell.m_data.GetUpLeftHandle();
				}
				else if (distance2 <= distance1 && distance2 <= distance3)
				{
					currentHandle = currentCell.m_data.GetUpRightHandle();
				}
				else if (distance3 <= distance1 && distance3 <= distance2)
				{
					currentHandle = currentCell.m_data.GetRightHandle();
				}
			}
		}
		break;
		case HexagonProfileDirection::Right:
		{
			if (currentCell.m_data.GetUpRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetUpRightCoordinate();
			}
			else if (currentCell.m_data.GetRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetRightCoordinate();
			}
			else if (currentCell.m_data.GetDownRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetDownRightCoordinate();
			}
			else
			{
				const auto pos1 = GetPosition(currentCell.m_data.GetUpRightCoordinate());
				const auto pos2 = GetPosition(currentCell.m_data.GetRightCoordinate());
				const auto pos3 = GetPosition(currentCell.m_data.GetDownRightCoordinate());
				const auto distance1 = glm::distance(pos1, glm::closestPointOnLine(pos1, glm::vec2(0), direction * 10000.0f));
				const auto distance2 = glm::distance(pos2, glm::closestPointOnLine(pos2, glm::vec2(0), direction * 10000.0f));
				const auto distance3 = glm::distance(pos3, glm::closestPointOnLine(pos3, glm::vec2(0), direction * 10000.0f));
				if (distance1 <= distance2 && distance1 <= distance3)
				{
					currentHandle = currentCell.m_data.GetUpRightHandle();
				}
				else if (distance2 <= distance1 && distance2 <= distance3)
				{
					currentHandle = currentCell.m_data.GetRightHandle();
				}
				else if (distance3 <= distance1 && distance3 <= distance2)
				{
					currentHandle = currentCell.m_data.GetDownRightHandle();
				}
			}
		}
		break;
		case HexagonProfileDirection::DownRight:
		{
			if (currentCell.m_data.GetRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetRightCoordinate();
			}
			else if (currentCell.m_data.GetDownRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetDownRightCoordinate();
			}
			else if (currentCell.m_data.GetDownLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetDownLeftCoordinate();
			}
			else
			{
				const auto pos1 = GetPosition(currentCell.m_data.GetRightCoordinate());
				const auto pos2 = GetPosition(currentCell.m_data.GetDownRightCoordinate());
				const auto pos3 = GetPosition(currentCell.m_data.GetDownLeftCoordinate());
				const auto distance1 = glm::distance(pos1, glm::closestPointOnLine(pos1, glm::vec2(0), direction * 10000.0f));
				const auto distance2 = glm::distance(pos2, glm::closestPointOnLine(pos2, glm::vec2(0), direction * 10000.0f));
				const auto distance3 = glm::distance(pos3, glm::closestPointOnLine(pos3, glm::vec2(0), direction * 10000.0f));
				if (distance1 <= distance2 && distance1 <= distance3)
				{
					currentHandle = currentCell.m_data.GetRightHandle();
				}
				else if (distance2 <= distance1 && distance2 <= distance3)
				{
					currentHandle = currentCell.m_data.GetDownRightHandle();
				}
				else if (distance3 <= distance1 && distance3 <= distance2)
				{
					currentHandle = currentCell.m_data.GetDownLeftHandle();
				}
			}
		}
		break;
		case HexagonProfileDirection::DownLeft:
		{
			if (currentCell.m_data.GetDownRightHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetDownRightCoordinate();
			}
			else if (currentCell.m_data.GetDownLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetDownLeftCoordinate();
			}
			else if (currentCell.m_data.GetLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetLeftCoordinate();
			}
			else
			{
				const auto pos1 = GetPosition(currentCell.m_data.GetDownRightCoordinate());
				const auto pos2 = GetPosition(currentCell.m_data.GetDownLeftCoordinate());
				const auto pos3 = GetPosition(currentCell.m_data.GetLeftCoordinate());
				const auto distance1 = glm::distance(pos1, glm::closestPointOnLine(pos1, glm::vec2(0), direction * 10000.0f));
				const auto distance2 = glm::distance(pos2, glm::closestPointOnLine(pos2, glm::vec2(0), direction * 10000.0f));
				const auto distance3 = glm::distance(pos3, glm::closestPointOnLine(pos3, glm::vec2(0), direction * 10000.0f));
				if (distance1 <= distance2 && distance1 <= distance3)
				{
					currentHandle = currentCell.m_data.GetDownRightHandle();
				}
				else if (distance2 <= distance1 && distance2 <= distance3)
				{
					currentHandle = currentCell.m_data.GetDownLeftHandle();
				}
				else if (distance3 <= distance1 && distance3 <= distance2)
				{
					currentHandle = currentCell.m_data.GetLeftHandle();
				}
			}
		}
		break;
		case HexagonProfileDirection::Left:
		{
			if (currentCell.m_data.GetDownLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetDownLeftCoordinate();
			}
			else if (currentCell.m_data.GetLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetLeftCoordinate();
			}
			else if (currentCell.m_data.GetUpLeftHandle() == -1)
			{
				found = true;
				coordinate = currentCell.m_data.GetUpLeftCoordinate();
			}
			else
			{
				const auto pos1 = GetPosition(currentCell.m_data.GetDownLeftCoordinate());
				const auto pos2 = GetPosition(currentCell.m_data.GetLeftCoordinate());
				const auto pos3 = GetPosition(currentCell.m_data.GetUpLeftCoordinate());
				const auto distance1 = glm::distance(pos1, glm::closestPointOnLine(pos1, glm::vec2(0), direction * 10000.0f));
				const auto distance2 = glm::distance(pos2, glm::closestPointOnLine(pos2, glm::vec2(0), direction * 10000.0f));
				const auto distance3 = glm::distance(pos3, glm::closestPointOnLine(pos3, glm::vec2(0), direction * 10000.0f));
				if (distance1 <= distance2 && distance1 <= distance3)
				{
					currentHandle = currentCell.m_data.GetDownLeftHandle();
				}
				else if (distance2 <= distance1 && distance2 <= distance3)
				{
					currentHandle = currentCell.m_data.GetLeftHandle();
				}
				else if (distance3 <= distance1 && distance3 <= distance2)
				{
					currentHandle = currentCell.m_data.GetUpLeftHandle();
				}
			}
		}
		break;
		}
	}
	return coordinate;
}

void HexagonProfile::Copy(const HexagonProfile& src)
{
	RefCells() = src.PeekCells();
	m_data = src.m_data;
	RefCellPool() = src.PeekCellPool();
}

bool HexagonProfileData::CheckBoundary(const std::vector<glm::vec2>& points)
{
	for (int i = 0; i < points.size(); i++) {
		auto& pa = points[(i == 0 ? points.size() - 1 : i - 1)];
		auto& pb = points[i];
		for (int j = 0; j < points.size(); j++) {
			auto& pc = points[(j == 0 ? points.size() - 1 : j - 1)];
			auto& pd = points[j];
			if (LineLineIntersect(pa, pb, pc, pd)) return true;
		}
	}
	return false;
}


void HexagonProfile::Construct(const std::vector<glm::vec2>& points)
{
	RefCells().clear();
	m_data.m_cellMap.clear();
	RefCellPool()= {};
	m_data.m_boundary.clear();
	auto copiedPoints = points;
	//1. Calculate min/max bound
	auto max = glm::vec2(FLT_MIN);
	auto min = glm::vec2(FLT_MAX);
	for (const auto& point : copiedPoints) {
		if (max.x < point.x) max.x = point.x;
		if (max.y < point.y) max.y = point.y;
		if (min.x > point.x) min.x = point.x;
		if (min.y > point.y) min.y = point.y;
	}
	const auto center = (max + min) / 2.0f;
	const auto boundaryRadius = (max - min) / 2.0f;
	max -= center;
	min -= center;
	for (auto& point : copiedPoints) {
		point -= center;
	}
	auto sum = glm::ivec2(0);
	const int yRange = glm::ceil(boundaryRadius.y / glm::cos(glm::radians(30.0f)));
	const int xRange = glm::ceil(boundaryRadius.x);
	for (int i = -xRange; i <= xRange; i++) {
		for (int j = -yRange; j <= yRange; j++) {
			glm::ivec2 coordinate;
			coordinate.y = j;
			coordinate.x = i - j / 2;
			if (m_data.InBoundary(copiedPoints, GetPosition(coordinate))) {
				const auto cellHandle = Allocate(coordinate);
				sum += coordinate;
			}
		}
	}
	if (RefCells().empty()) return;
	sum /= RefCells().size();
	for (auto& cell : RefCells()) {
		cell.m_data.m_coordinate -= sum;
	}
}
*/

bool HexagonProfileData::RayLineIntersect(const glm::vec2& rayOrigin, const glm::vec2& rayDirection,
	const glm::vec2& point1, const glm::vec2& point2)
{
	const auto v1 = rayOrigin - point1;
	const auto v2 = point2 - point1;
	const auto v3 = glm::vec2(-rayDirection.y, rayDirection.x);

	const float dot = glm::dot(v2, v3);
	if (dot == 0.0f)
		return false;

	const float t1 = (v2.x * v1.y - v2.y * v1.x) / dot;
	const float t2 = glm::dot(v1, v3) / dot;

	//!!!!Check t2 >= 0 if we allow intersect on point 1
	if (t1 >= 0.0f && t2 > 0.0f && 1.0f - t2 >= 0.0f)
		return true;

	return false;
}


bool HexagonProfileData::InBoundary(const std::vector<glm::vec2>& boundary, const glm::vec2& point)
{
	constexpr auto point2 = glm::vec2(1.0f, 0.0f);
	constexpr auto point3 = glm::vec2(1.0f, 0.0f);
	int windingNumber = 0;
	const auto size = boundary.size();
	if (size < 3) return false;
	for (int i = 0; i < size - 1; i++) {
		if (RayLineIntersect(point, point2, boundary[i], boundary[i + 1]) &&
			RayLineIntersect(point, point3, boundary[i], boundary[i + 1])) {
			windingNumber++;
		}
	}
	if (RayLineIntersect(point, point2, boundary[size - 1], boundary[0]) &&
		RayLineIntersect(point, point3, boundary[size - 1], boundary[0]))
		windingNumber++;
	if (windingNumber % 2 == 1) {
		return true;
	}
	return false;
}


bool HexagonProfileData::LineLineIntersect(const glm::vec2& pa, const glm::vec2& pb,
	const glm::vec2& pc, const glm::vec2& pd)
{
	const auto v1 = pa - pc;
	const auto v2 = pd - pc;
	const auto v3 = glm::vec2(-(pb.y - pa.y), (pb.x - pa.x));

	const float dot = glm::dot(v2, v3);
	if (dot == 0.0f)
		return false;

	const float t1 = (v2.x * v1.y - v2.y * v1.x) / dot;
	const float t2 = glm::dot(v1, v3) / dot;

	if (t1 > 0.0f && t1 < 1.0f && t2 > 0.0f && t2 < 1.0f)
		return true;

	return false;
}