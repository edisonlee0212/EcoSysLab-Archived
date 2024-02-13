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

float LogWoodIntersection::GetDefectStatus(const float angle) const
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
	if (index < m_intersections.size() - 1)
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

LogWoodIntersectionBoundaryPoint& LogWood::GetBoundaryPoint(const float height, const float angle)
{
	const int index = height / m_heightStep;
	const float a = (height - m_heightStep * index) / m_heightStep;
	if (index < m_intersections.size() - 1)
	{
		if (a < 0.5f)
		{
			return m_intersections.at(index).m_boundary.at(angle);
		}
		return m_intersections.at(index + 1).m_boundary.at(angle);
	}
	return m_intersections.back().m_boundary.at(angle);
}

void LogWood::MarkDefectRegion(const float height, const float angle, const float heightRange, const float angleRange)
{
	const int heightStepSize = heightRange / m_heightStep;
	for(int yIndex = -heightStepSize; yIndex <= heightStepSize; yIndex++)
	{
		const auto sinVal = glm::abs(static_cast<float>(yIndex)) / heightStepSize;
		const auto sinAngle = glm::asin(sinVal);
		const float maxAngleRange = glm::cos(sinAngle);
		for (int xIndex = -angleRange; xIndex <= angleRange; xIndex++)
		{
			const auto actualYIndex = yIndex + height / m_heightStep;
			if (actualYIndex < 0 || actualYIndex >= m_intersections.size()) continue;
			if(glm::abs(static_cast<float>(xIndex)) / angleRange > maxAngleRange) continue;
			const auto actualXIndex = static_cast<int>(xIndex + angle) % 360;
			m_intersections.at(actualYIndex).m_boundary.at(actualXIndex).m_defectStatus = 1.0f;
		}
	}
}

void LogWood::EraseDefectRegion(float height, float angle, float heightRange, float angleRange)
{
	const int heightStepSize = heightRange / m_heightStep;
	for (int yIndex = -heightStepSize; yIndex <= heightStepSize; yIndex++)
	{
		const auto sinVal = glm::abs(static_cast<float>(yIndex)) / heightStepSize;
		const auto sinAngle = glm::asin(sinVal);
		const float maxAngleRange = glm::cos(sinAngle);
		for (int xIndex = -angleRange; xIndex <= angleRange; xIndex++)
		{
			const auto actualYIndex = yIndex + height / m_heightStep;
			if (actualYIndex < 0 || actualYIndex >= m_intersections.size()) continue;
			if (glm::abs(static_cast<float>(xIndex)) / angleRange > maxAngleRange) continue;
			const auto actualXIndex = static_cast<int>(xIndex + angle) % 360;
			m_intersections.at(actualYIndex).m_boundary.at(actualXIndex).m_defectStatus = 0.0f;
		}
	}
}

void LogWood::ClearDefects()
{
	for(auto& intersection : m_intersections)
	{
		for (auto& boundaryPoint : intersection.m_boundary) boundaryPoint.m_defectStatus = 0.0f;
	}
}


bool LogWood::RayCastSelection(const glm::mat4& transform, const float pointDistanceThreshold, const Ray& ray, float& height, float& angle) const
{
	float minDistance = FLT_MAX;
	bool found = false;
	for (int yIndex = 0; yIndex < m_intersections.size(); yIndex++)
	{
		const auto testHeight = yIndex * m_heightStep;
		for (int xIndex = 0; xIndex < 360; xIndex++)
		{
			const auto surfacePoint = GetSurfacePoint(testHeight, xIndex);
			const glm::vec3 position = (transform * glm::translate(glm::vec3(surfacePoint.x, testHeight, surfacePoint.y)))[3];
			glm::vec3 closestPoint = Ray::ClosestPointOnLine(position, ray.m_start, ray.m_start + ray.m_direction * 10000.0f);
			const float pointDistance = glm::distance(closestPoint, position);
			const float distanceToStart = glm::distance(ray.m_start, position);
			if (distanceToStart < minDistance && pointDistance < pointDistanceThreshold)
			{
				minDistance = distanceToStart;
				height = testHeight;
				angle = xIndex;
				found = true;
			}
		}
	}
	return found;
}
