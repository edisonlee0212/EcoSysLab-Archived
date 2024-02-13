#include "LogWood.hpp"

using namespace EcoSysLab;

float LogWoodIntersection::GetCenterDistance(const float angle) const
{
	assert(m_boundary.size() == 360);
	const int startIndex = static_cast<int>(angle) % 360;
	const float a = angle - static_cast<int>(angle);
	const int endIndex = (startIndex + 1) % 360;
	return glm::mix(m_boundary[startIndex].m_centerDistance, m_boundary[endIndex].m_centerDistance, a);
}

glm::vec2 LogWoodIntersection::GetBoundaryPoint(const float angle) const
{
	return m_center + glm::vec2(glm::cos(glm::radians(angle)), glm::sin(glm::radians(angle))) * GetCenterDistance(angle);
}

float LogWoodIntersection::GetDefectStatus(float angle) const
{
	assert(m_boundary.size() == 360);
	const int startIndex = static_cast<int>(angle) % 360;
	const float a = angle - static_cast<int>(angle);
	const int endIndex = (startIndex + 1) % 360;
	return glm::mix(m_boundary[startIndex].m_defectStatus, m_boundary[endIndex].m_defectStatus, a);
}

glm::vec2 LogWood::GetSurfacePoint(const float height, const float angle) const
{
	const int index = height / m_heightStep;
	const float a = (height - m_heightStep * index) / m_heightStep;
	if(index < m_intersections.size() - 1)
	{
		return glm::mix(m_intersections.at(index).GetBoundaryPoint(angle),
			m_intersections.at(index + 1).GetBoundaryPoint(angle), a);
	}
	return m_intersections.back().GetBoundaryPoint(angle);
}

float LogWood::GetDefectStatus(const float height, const float angle) const
{
	const int index = height / m_heightStep;
	const float a = (height - m_heightStep * index) / m_heightStep;
	if (index < m_intersections.size() - 1)
	{
		return glm::mix(m_intersections.at(index).GetDefectStatus(angle),
			m_intersections.at(index + 1).GetDefectStatus(angle), a);
	}
	return m_intersections.back().GetDefectStatus(angle);
}
