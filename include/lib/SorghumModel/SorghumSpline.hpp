#pragma once
#include <Curve.hpp>
#include "SorghumGrowthDescriptor.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	struct SorghumSplineNode {
		glm::vec3 m_position;
		float m_theta;
		float m_width;
		float m_waviness;
		glm::vec3 m_front;
		glm::vec3 m_left;
		float m_range;

		SorghumSplineNode(const glm::vec3 &position, float angle, float width, float waviness, const glm::vec3& front, const glm::vec3& left, float range);
		SorghumSplineNode() = default;
	};

	class SorghumSplineSegment {
	public:
		glm::vec3 m_position;
		glm::vec3 m_front;
		glm::vec3 m_up;
		glm::quat m_rotation;
		float m_radius;
		float m_theta;
		float m_leftHeightOffset = 1.0f;
		float m_rightHeightOffset = 1.0f;
		SorghumSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front,
			float radius, float theta,
			float leftHeightOffset = 0.0f, float rightHeightOffset = 0.0f);

		glm::vec3 GetLeafPoint(float angle) const;
		glm::vec3 GetStemPoint(float angle) const;
		glm::vec3 GetNormal(float angle) const;
	};
} // namespace EcoSysLab