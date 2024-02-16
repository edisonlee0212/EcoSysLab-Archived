#pragma once
using namespace EvoEngine;
namespace EcoSysLab
{
	struct LogWoodIntersectionBoundaryPoint
	{
		float m_centerDistance = 0.0f;
		float m_defectStatus = 0.0f;
		glm::vec4 m_color = glm::vec4(1.0f);
	};
	class LogWoodIntersection
	{
	public:
		glm::vec2 m_center = glm::vec2(0.0f);
		std::vector<LogWoodIntersectionBoundaryPoint> m_boundary {};
		[[nodiscard]] float GetCenterDistance(float angle) const;
		[[nodiscard]] glm::vec2 GetBoundaryPoint(float angle) const;
		[[nodiscard]] float GetDefectStatus(float angle) const;
		[[nodiscard]] glm::vec4 GetColor(float angle) const;
		[[nodiscard]] float GetAverageDistance() const;
		[[nodiscard]] float GetMaxDistance() const;
		[[nodiscard]] float GetMinDistance() const;
	};

	struct LogGradeFaceCutting
	{
		float m_start = 0;
		float m_end = 0;
	};

	struct LogGradingFace
	{
		int m_faceIndex = 0;
		int m_startAngle = 0;
		int m_endAngle = 0;
		int m_faceGrade = 0;
		std::vector<LogGradeFaceCutting> m_cuttings{};
		float m_clearCuttingMinLength = 0;
		float m_clearCuttingMinProportion = 0;
	};

	struct LogGrading
	{
		int m_angleOffset = 0;
		float m_scalingDiameter = 0;
		LogGradingFace m_faces[4] {};
	};

	class LogWood
	{
	public:
		bool m_butt = true;
		float m_lengthWithoutTrim = 0.0f;
		std::vector<LogWoodIntersection> m_intersections;
		[[nodiscard]] glm::vec2 GetSurfacePoint(float height, float angle) const;
		[[nodiscard]] float GetCenterDistance(float height, float angle) const;
		[[nodiscard]] float GetDefectStatus(float height, float angle) const;
		[[nodiscard]] glm::vec4 GetColor(float height, float angle) const;

		[[nodiscard]] LogWoodIntersectionBoundaryPoint& GetBoundaryPoint(float height, float angle);
		void Rotate(int degrees);
		[[nodiscard]] float GetAverageDistance(float height) const;
		[[nodiscard]] float GetAverageDistance() const;
		[[nodiscard]] float GetMaxAverageDistance() const;
		[[nodiscard]] float GetMinAverageDistance() const;
		[[nodiscard]] float GetMaxDistance() const;
		[[nodiscard]] float GetMinDistance() const;
		void MarkDefectRegion(float height, float angle, float heightRange, float angleRange);
		void EraseDefectRegion(float height, float angle, float heightRange, float angleRange);
		void ClearDefects();
		[[nodiscard]] bool RayCastSelection(
			const glm::mat4 &transform,
			float pointDistanceThreshold, const Ray& ray, float &height, float &angle) const;

		[[nodiscard]] LogGrading CalculateGradingData(int angleOffset) const;
		void ColorBasedOnGrading(const LogGrading& logGradingData);
	};
}