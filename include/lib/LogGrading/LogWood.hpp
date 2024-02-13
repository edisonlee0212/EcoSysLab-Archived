#pragma once
using namespace EvoEngine;
namespace EcoSysLab
{
	struct LogWoodIntersectionBoundaryPoint
	{
		float m_centerDistance = 0.0f;
		float m_defectStatus = 0.0f;
	};
	class LogWoodIntersection
	{
	public:
		glm::vec2 m_center = glm::vec2(0.0f);
		std::vector<LogWoodIntersectionBoundaryPoint> m_boundary {};
		[[nodiscard]] float GetCenterDistance(float angle) const;
		[[nodiscard]] glm::vec2 GetBoundaryPoint(float angle) const;
		[[nodiscard]] float GetDefectStatus(float angle) const;
	};
	class LogWood
	{
	public:
		float m_heightStep = 0.01f;
		std::vector<LogWoodIntersection> m_intersections;
		[[nodiscard]] glm::vec2 GetSurfacePoint(float height, float angle) const;
		[[nodiscard]] float GetDefectStatus(float height, float angle) const;
	};
}