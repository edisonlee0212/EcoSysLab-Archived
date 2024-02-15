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

float LogWoodIntersection::GetAverageDistance() const
{
	float retVal = 0.0f;
	for (const auto& i : m_boundary) retVal += i.m_centerDistance;
	return retVal / 360.f;
}

float LogWoodIntersection::GetMaxDistance() const
{
	float retVal = FLT_MIN;
	for (const auto& i : m_boundary)
	{
		retVal = glm::max(retVal, i.m_centerDistance);
	}
	return retVal;
}

float LogWoodIntersection::GetMinDistance() const
{
	float retVal = FLT_MAX;
	for (const auto& i : m_boundary)
	{
		retVal = glm::min(retVal, i.m_centerDistance);
	}
	return retVal;
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

float LogWood::GetCenterDistance(float height, float angle) const
{
	const int index = height / m_heightStep;
	const float a = (height - m_heightStep * index) / m_heightStep;
	if (index < m_intersections.size() - 1)
	{
		return glm::mix(m_intersections.at(index).GetCenterDistance(angle),
			m_intersections.at(index + 1).GetCenterDistance(angle), a);
	}
	return m_intersections.back().GetCenterDistance(angle);
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

void LogWood::Rotate(int degrees)
{
	while (degrees < 0.0f) degrees += 360.0f;
	degrees = degrees % 360;
	if (degrees == 0) return;
	for (auto& intersection : m_intersections)
	{
		intersection.m_center = glm::rotate(intersection.m_center, glm::radians(static_cast<float>(degrees)));
		intersection.m_boundary.insert(intersection.m_boundary.begin(), intersection.m_boundary.end() - degrees, intersection.m_boundary.end());
		intersection.m_boundary.resize(360);
	}
}

float LogWood::GetAverageDistance(const float height) const
{
	const int index = height / m_heightStep;
	const float a = (height - m_heightStep * index) / m_heightStep;
	if (index < m_intersections.size() - 1)
	{
		if (a < 0.5f)
		{
			return m_intersections.at(index).GetAverageDistance();
		}
		return m_intersections.at(index + 1).GetAverageDistance();
	}
	return m_intersections.back().GetAverageDistance();
}

float LogWood::GetAverageDistance() const
{
	float sumDistance = 0.0f;
	for (const auto& intersection : m_intersections)
	{
		for (const auto& point : intersection.m_boundary) sumDistance += point.m_centerDistance;
	}
	return sumDistance / m_intersections.size() / 360.0f;
}

float LogWood::GetMaxAverageDistance() const
{
	float retVal = FLT_MIN;
	for (const auto& i : m_intersections) {
		retVal = glm::max(i.GetAverageDistance(), retVal);
	}
	return retVal;
}

float LogWood::GetMinAverageDistance() const
{
	float retVal = FLT_MAX;
	for (const auto& i : m_intersections) {
		retVal = glm::min(i.GetAverageDistance(), retVal);
	}
	return retVal;
}

float LogWood::GetMaxDistance() const
{
	float retVal = FLT_MIN;
	for (const auto& i : m_intersections) {
		retVal = glm::max(i.GetMaxDistance(), retVal);
	}
	return retVal;
}

float LogWood::GetMinDistance() const
{
	float retVal = FLT_MAX;
	for (const auto& i : m_intersections) {
		retVal = glm::min(i.GetMinDistance(), retVal);
	}
	return retVal;
}

void LogWood::MarkDefectRegion(const float height, const float angle, const float heightRange, const float angleRange)
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
	for (auto& intersection : m_intersections)
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
