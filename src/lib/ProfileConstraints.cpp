#include "ProfileConstraints.hpp"

#include "TreeVisualizer.hpp"
using namespace EcoSysLab;

bool onSegment(const glm::vec2& p, const glm::vec2& q, const glm::vec2& r)
{
	if (q.x <= glm::max(p.x, r.x) && q.x >= glm::min(p.x, r.x) &&
		q.y <= glm::max(p.y, r.y) && q.y >= glm::min(p.y, r.y))
		return true;

	return false;
}
int orientation(const glm::vec2& p, const glm::vec2& q, const glm::vec2& r)
{
	// See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
	// for details of below formula. 
	const float val = (q.y - p.y) * (r.x - q.x) -
		(q.x - p.x) * (r.y - q.y);

	if (val == 0.0f) return 0;  // collinear 

	return (val > 0) ? 1 : 2; // clock or counterclock wise 
}

void ProfileBoundary::CalculateCenter()
{
	m_center = glm::vec2(0.0f);
	float sum = 0.0f;
	for (int lineIndex = 0; lineIndex < m_points.size(); lineIndex++)
	{
		const auto& p1 = m_points[lineIndex];
		const auto& p2 = m_points[(lineIndex + 1) % m_points.size()];
		const auto lineLength = glm::distance(p1, p2);
		sum += lineLength;
		m_center += (p1 + p2) * 0.5f * lineLength;
	}
	m_center /= sum;
}

void ProfileBoundary::RenderBoundary(const ImVec2 origin, const float zoomFactor, ImDrawList* drawList, ImU32 color, float thickness) const
{
	for (int pointIndex = 0; pointIndex < m_points.size() - 1; pointIndex++)
	{
		const auto& p1 = m_points[pointIndex];
		const auto& p2 = m_points[pointIndex + 1];
		drawList->AddLine(ImVec2(origin.x + p1.x * zoomFactor,
			origin.y + p1.y * zoomFactor), ImVec2(origin.x + p2.x * zoomFactor,
				origin.y + p2.y * zoomFactor), color, thickness);
	}

	const auto& p1 = m_points.back();
	const auto& p2 = m_points[0];
	drawList->AddLine(ImVec2(origin.x + p1.x * zoomFactor,
		origin.y + p1.y * zoomFactor), ImVec2(origin.x + p2.x * zoomFactor,
			origin.y + p2.y * zoomFactor), color, thickness);

}

void ProfileAttractor::RenderAttractor(ImVec2 origin, float zoomFactor, ImDrawList* drawList, ImU32 color,
	float thickness) const
{
	if(m_attractorPoints.empty()) return;
	for (const auto& attractorPoint : m_attractorPoints)
	{
		const auto& p1 = attractorPoint.first;
		const auto& p2 = attractorPoint.second;
		drawList->AddLine(ImVec2(origin.x + p1.x * zoomFactor,
			origin.y + p1.y * zoomFactor), ImVec2(origin.x + p2.x * zoomFactor,
				origin.y + p2.y * zoomFactor), color, thickness);
	}

}

glm::vec2 ProfileAttractor::FindClosestPoint(const glm::vec2& position) const
{
	auto distance = FLT_MAX;
	auto retVal = glm::vec2(0.0f);
	for (const auto& attractorPoint : m_attractorPoints)
	{
		const auto& p1 = attractorPoint.first;
		const auto& p2 = attractorPoint.second;
		const auto testPoint = glm::closestPointOnLine(position, p1, p2);
		const auto newDistance = glm::distance(testPoint, position);
		if (distance > newDistance)
		{
			distance = newDistance;
			retVal = testPoint;
		}
	}
	return retVal;
}

bool ProfileBoundary::BoundaryValid() const
{
	if (m_points.size() <= 3) return false;
	for (int lineIndex = 0; lineIndex < m_points.size(); lineIndex++)
	{
		const auto& p1 = m_points[lineIndex];
		const auto& p2 = m_points[(lineIndex + 1) % m_points.size()];
		for (int lineIndex2 = 0; lineIndex2 < m_points.size(); lineIndex2++)
		{
			if (lineIndex == lineIndex2) continue;
			if ((lineIndex + 1) % m_points.size() == lineIndex2
				|| (lineIndex2 + 1) % m_points.size() == lineIndex) continue;
			const auto& p3 = m_points[lineIndex2];
			const auto& p4 = m_points[(lineIndex2 + 1) % m_points.size()];
			if (Intersect(p1, p2, p3, p4))
			{
				return false;
			}
		}
	}
	return true;
}

bool ProfileBoundary::InBoundary(const glm::vec2& position) const
{
	auto distance = FLT_MAX;
	int intersectCount1 = 0;
	int intersectCount2 = 0;
	int intersectCount3 = 0;
	int intersectCount4 = 0;
	for (int lineIndex = 0; lineIndex < m_points.size(); lineIndex++) {
		const auto& p1 = m_points[lineIndex];
		const auto& p2 = m_points[(lineIndex + 1) % m_points.size()];
		const auto p3 = position;
		const auto p41 = position + glm::vec2(1000.0f, 0.0f);
		const auto p42 = position + glm::vec2(-1000.0f, 0.0f);
		const auto p43 = position + glm::vec2(0.0f, 1000.0f);
		const auto p44 = position + glm::vec2(0.0f, -1000.0f);
		if (Intersect(p1, p2, p3, p41)) {
			intersectCount1++;
		}
		if (Intersect(p1, p2, p3, p42)) {
			intersectCount2++;
		}
		if (Intersect(p1, p2, p3, p43)) {
			intersectCount3++;
		}
		if (Intersect(p1, p2, p3, p44)) {
			intersectCount4++;
		}
		const auto testPoint = glm::closestPointOnLine(position, p1, p2);
		const auto newDistance = glm::distance(testPoint, position);
		if (distance > newDistance)
		{
			distance = newDistance;
		}
	}
	return intersectCount1 % 2 != 0
		&& intersectCount2 % 2 != 0
		&& intersectCount3 % 2 != 0
		&& intersectCount4 % 2 != 0;
}

bool ProfileBoundary::InBoundary(const glm::vec2& position, glm::vec2& closestPoint) const
{
	closestPoint = glm::vec2(0.0f);
	auto distance = FLT_MAX;
	int intersectCount1 = 0;
	int intersectCount2 = 0;
	int intersectCount3 = 0;
	int intersectCount4 = 0;
	for (int lineIndex = 0; lineIndex < m_points.size(); lineIndex++) {
		const auto& p1 = m_points[lineIndex];
		const auto& p2 = m_points[(lineIndex + 1) % m_points.size()];
		const auto p3 = position;
		const auto p41 = position + glm::vec2(1000.0f, 0.0f);
		const auto p42 = position + glm::vec2(-1000.0f, 0.0f);
		const auto p43 = position + glm::vec2(0.0f, 1000.0f);
		const auto p44 = position + glm::vec2(0.0f, -1000.0f);
		if (Intersect(p1, p2, p3, p41)) {
			intersectCount1++;
		}
		if (Intersect(p1, p2, p3, p42)) {
			intersectCount2++;
		}
		if (Intersect(p1, p2, p3, p43)) {
			intersectCount3++;
		}
		if (Intersect(p1, p2, p3, p44)) {
			intersectCount4++;
		}
		const auto testPoint = glm::closestPointOnLine(position, p1, p2);
		const auto newDistance = glm::distance(testPoint, position);
		if (distance > newDistance)
		{
			closestPoint = testPoint;
			distance = newDistance;
		}
	}
	const bool retVal = intersectCount1 % 2 != 0
		&& intersectCount2 % 2 != 0
		&& intersectCount3 % 2 != 0
		&& intersectCount4 % 2 != 0;
	return retVal;
}

bool ProfileBoundary::Intersect(const glm::vec2& p1, const glm::vec2& q1, const glm::vec2& p2, const glm::vec2& q2)
{
	// Find the four orientations needed for general and 
	// special cases 
	const int o1 = orientation(p1, q1, p2);
	const int o2 = orientation(p1, q1, q2);
	const int o3 = orientation(p2, q2, p1);
	const int o4 = orientation(p2, q2, q1);
	// General case 
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases 
	// p1, q1 and p2 are collinear and p2 lies on segment p1q1 
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and q2 are collinear and q2 lies on segment p1q1 
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are collinear and p1 lies on segment p2q2 
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are collinear and q1 lies on segment p2q2 
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases 
}

int ProfileConstraints::FindBoundary(const glm::vec2& position) const
{
	int retVal = -1;
	auto distanceToClosestBoundaryCenter = FLT_MAX;
	for (int boundaryIndex = 0; boundaryIndex < m_boundaries.size(); boundaryIndex++)
	{
		const auto& boundary = m_boundaries.at(boundaryIndex);
		glm::vec2 test;
		if (boundary.InBoundary(position, test))
		{
			const auto currentDistanceToBoundaryCenter = glm::distance(position, boundary.m_center);
			if (currentDistanceToBoundaryCenter < distanceToClosestBoundaryCenter)
			{
				distanceToClosestBoundaryCenter = currentDistanceToBoundaryCenter;
				retVal = boundaryIndex;
			}
		}
	}
	return retVal;
}

bool ProfileConstraints::Valid(const size_t boundaryIndex) const
{
	const auto& targetBoundary = m_boundaries[boundaryIndex];
	if (!targetBoundary.BoundaryValid()) return false;

	for (int testBoundaryIndex = 0; testBoundaryIndex < m_boundaries.size() - 1; testBoundaryIndex++) {
		const auto& testBoundary = m_boundaries.at(testBoundaryIndex);
		for (int newPointIndex = 0; newPointIndex < targetBoundary.m_points.size(); newPointIndex++)
		{
			const auto& p1 = targetBoundary.m_points[newPointIndex];
			const auto& p2 = targetBoundary.m_points[(newPointIndex + 1) % targetBoundary.m_points.size()];
			for (int testPointIndex = 0; testPointIndex < testBoundary.m_points.size(); testPointIndex++)
			{
				const auto& p3 = testBoundary.m_points[testPointIndex];
				const auto& p4 = testBoundary.m_points[(testPointIndex + 1) % testBoundary.m_points.size()];
				if (ProfileBoundary::Intersect(p1, p2, p3, p4)) return false;
			}
		}
	}

	for (int testBoundaryIndex = 0; testBoundaryIndex < m_boundaries.size() - 1; testBoundaryIndex++) {
		const auto& testBoundary = m_boundaries.at(testBoundaryIndex);
		for (const auto& newPoint : targetBoundary.m_points)
		{
			if (testBoundary.InBoundary(newPoint)) return false;
		}
	}
	return true;
}

glm::vec2 ProfileConstraints::GetTarget(const glm::vec2& position) const
{
	glm::vec2 closestPoint;
	//1. If in any of the boundary
	int boundaryIndex = -1;
	auto distanceToClosestPointOnBoundary = FLT_MAX;
	auto distanceToClosestBoundaryCenter = FLT_MAX;
	for (int currentBoundaryIndex = 0; currentBoundaryIndex < m_boundaries.size(); currentBoundaryIndex++)
	{
		const auto& boundary = m_boundaries.at(currentBoundaryIndex);
		glm::vec2 currentClosestPoint;
		const auto currentInBoundary = boundary.InBoundary(position, currentClosestPoint);
		const auto currentDistance = glm::distance(currentClosestPoint, position);
		if (currentInBoundary)
		{
			const auto currentDistanceToBoundaryCenter = glm::distance(position, boundary.m_center);
			if (currentDistanceToBoundaryCenter < distanceToClosestBoundaryCenter)
			{
				distanceToClosestBoundaryCenter = currentDistanceToBoundaryCenter;
				boundaryIndex = currentBoundaryIndex;
			}
		}else if(currentDistance < distanceToClosestPointOnBoundary)
		{
			distanceToClosestPointOnBoundary = currentDistance;
			closestPoint = currentClosestPoint;
		}
		
	}
	if(boundaryIndex != -1)
	{
		if(m_attractors.empty())
		{
			closestPoint = m_boundaries.at(boundaryIndex).m_center;
		}else
		{
			auto distanceToAttractor = FLT_MAX;
			for(const auto& attractor : m_attractors)
			{
				const auto currentClosestPoint = attractor.FindClosestPoint(position);
				const float currentDistanceToAttractor = glm::distance(currentClosestPoint, position);
				if(currentDistanceToAttractor < distanceToAttractor)
				{
					distanceToAttractor = currentDistanceToAttractor;
					closestPoint = currentClosestPoint;
				}
			}
		}
	}
	return closestPoint - position;
}