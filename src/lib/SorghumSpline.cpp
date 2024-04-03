#include "SorghumSpline.hpp"
#include "SorghumLayer.hpp"

using namespace EcoSysLab;

SorghumSplineNode::SorghumSplineNode(const glm::vec3 &position, const float angle, const float width, const float waviness, const glm::vec3& front, const glm::vec3& left,
    float range) {
  m_position = position;
  m_theta = angle;
  m_width = width;
  m_waviness = waviness;
  m_front = front;
  m_left = left;
  m_range = range;
}


SorghumSplineSegment::SorghumSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front,
	const float radius, const float theta, const float leftHeightOffset, const float rightHeightOffset) {
	m_position = position;
	m_up = up;
	m_front = front;
	m_radius = radius;
	m_theta = theta;
	m_leftHeightOffset = leftHeightOffset;
	m_rightHeightOffset = rightHeightOffset;
}

glm::vec3 SorghumSplineSegment::GetLeafPoint(const float angle) const
{
	if (glm::abs(m_theta) < 90.0f) {
		const auto arcRadius = m_radius / glm::sin(glm::radians(glm::max(89.f, m_theta)));
		const auto center = m_position + arcRadius * m_up;
		const auto direction = glm::normalize(glm::rotate(m_up, glm::radians(angle), m_front));
		const auto point = center - arcRadius * direction;
		const auto distanceToCenter = glm::sin(glm::radians(angle)) * arcRadius / m_radius;
		return point - (angle < 0 ? m_leftHeightOffset : m_rightHeightOffset) * glm::pow(distanceToCenter, 2.f) * m_up;
	}
	const auto radius = m_radius;
	const auto center = m_position + radius * m_up;
	const auto direction = glm::rotate(m_up, glm::radians(angle), m_front);
	return center - radius * direction;
}

glm::vec3 SorghumSplineSegment::GetStemPoint(const float angle) const
{
	const auto direction = glm::rotate(m_up, glm::radians(angle), m_front);
	return m_position - m_radius * direction;
}



glm::vec3 SorghumSplineSegment::GetNormal(const float angle) const
{
	return glm::normalize(glm::rotate(m_up, glm::radians(angle), m_front));
}
