#include "PipeProfile.hpp"

using namespace EcoSysLab;

bool LineLineIntersect(const glm::vec2& pa, const glm::vec2& pb, const glm::vec2& pc, const glm::vec2& pd)
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

bool RayLineIntersect(const glm::vec2& rayOrigin, const glm::vec2& rayDirection,
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
	if (t1 >= 0.0f && t2 >= 0.0f && 1.0f - t2 >= 0.0f)
		return true;

	return false;
}

void ProfileInfo::CheckBoundary()
{
	if (m_boundary.size() <= 2)
	{
		m_boundaryValid = false;
		return;
	}
	for (int i = 0; i < m_boundary.size(); i++) {
		auto& pa = m_boundary[(i == 0 ? m_boundary.size() - 1 : i - 1)];
		auto& pb = m_boundary[i];
		for (int j = 0; j < m_boundary.size(); j++) {
			auto& pc = m_boundary[(j == 0 ? m_boundary.size() - 1 : j - 1)];
			auto& pd = m_boundary[j];
			if (LineLineIntersect(pa, pb, pc, pd)) {
				m_boundaryValid = false;
				return;
			}
		}
	}
	m_boundaryValid = true;
}

bool ProfileInfo::IsBoundaryValid(const std::vector<glm::vec2>& points)
{
	if (points.size() <= 2) return false;
	for (int i = 0; i < points.size(); i++) {
		auto& pa = points[(i == 0 ? points.size() - 1 : i - 1)];
		auto& pb = points[i];
		for (int j = 0; j < points.size(); j++) {
			auto& pc = points[(j == 0 ? points.size() - 1 : j - 1)];
			auto& pd = points[j];
			if (LineLineIntersect(pa, pb, pc, pd)) return false;
		}
	}
	return true;
}

bool ProfileInfo::InBoundary(const glm::vec2& point) const
{
	constexpr auto point2 = glm::vec2(1.0f, 0.0f);
	constexpr auto point3 = glm::vec2(1.0f, 0.0f);
	int windingNumber = 0;
	const auto size = m_boundary.size();
	if (size < 3) return false;
	for (int i = 0; i < size - 1; i++) {
		if (RayLineIntersect(point, point2, m_boundary[i], m_boundary[i + 1]) &&
			RayLineIntersect(point, point3, m_boundary[i], m_boundary[i + 1])) {
			windingNumber++;
		}
	}
	if (RayLineIntersect(point, point2, m_boundary[size - 1], m_boundary[0]) &&
		RayLineIntersect(point, point3, m_boundary[size - 1], m_boundary[0]))
		windingNumber++;
	if (windingNumber % 2 == 1) {
		return true;
	}
	return false;
}
