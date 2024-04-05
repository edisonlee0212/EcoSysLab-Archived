#pragma once
#include <Curve.hpp>
#include "SorghumGrowthStages.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class SorghumSplineSegment {
	public:
		glm::vec3 m_position;
		glm::vec3 m_front;
		glm::vec3 m_up;
		float m_radius;
		float m_theta;
		float m_leftHeightOffset = 1.0f;
		float m_rightHeightOffset = 1.0f;
		SorghumSplineSegment() = default;
		SorghumSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front,
			float radius, float theta,
			float leftHeightOffset = 0.0f, float rightHeightOffset = 0.0f);

		[[nodiscard]] glm::vec3 GetLeafPoint(float angle) const;
		[[nodiscard]] glm::vec3 GetStemPoint(float angle) const;
		[[nodiscard]] glm::vec3 GetNormal(float angle) const;
	};

	class SorghumSpline
	{
		
	public:
		std::vector<SorghumSplineSegment> m_segments;
		SorghumSplineSegment Interpolate(int leftIndex, float a) const;

		void Subdivide(float subdivisionDistance, std::vector<SorghumSplineSegment>& subdividedSegments) const;
	};
} // namespace EcoSysLab