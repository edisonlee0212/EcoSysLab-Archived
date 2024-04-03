#include "SorghumSpline.hpp"
#include "SorghumLayer.hpp"

using namespace EcoSysLab;

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

SorghumSplineSegment SorghumSpline::Interpolate(int leftIndex, float a) const
{
	if(a < glm::epsilon<float>())
	{
		return m_segments.at(leftIndex);
	}
	if(1.f - a < glm::epsilon<float>())
	{
		return  m_segments.at(leftIndex + 1);
	}
	SorghumSplineSegment retVal{};
	const auto& s1 = m_segments.at(leftIndex);
	const auto& s2 = m_segments.at(leftIndex + 1);
	SorghumSplineSegment s0, s3;
	if (leftIndex == 0)
	{
		s0.m_position = 2.f * s1.m_position - s2.m_position;
		s0.m_front = 2.f * s1.m_front - s2.m_front;
		s0.m_up = 2.f * s1.m_up - s2.m_up;
		s0.m_radius = 2.f * s1.m_radius - s2.m_radius;
		s0.m_theta = 2.f * s1.m_theta - s2.m_theta;
		s0.m_leftHeightOffset = 2.f * s1.m_leftHeightOffset - s2.m_leftHeightOffset;
		s0.m_rightHeightOffset = 2.f * s1.m_rightHeightOffset - s2.m_rightHeightOffset;
	}else
	{
		s0 = m_segments.at(leftIndex - 1);
	}
	if (leftIndex < m_segments.size() - 1)
	{
		s3.m_position = 2.f * s2.m_position - s1.m_position;
		s3.m_front = 2.f * s2.m_front - s1.m_front;
		s3.m_up = 2.f * s2.m_up - s1.m_up;
		s3.m_radius = 2.f * s2.m_radius - s1.m_radius;
		s3.m_theta = 2.f * s2.m_theta - s1.m_theta;
		s3.m_leftHeightOffset = 2.f * s2.m_leftHeightOffset - s1.m_leftHeightOffset;
		s3.m_rightHeightOffset = 2.f * s2.m_rightHeightOffset - s1.m_rightHeightOffset;
	}
	else
	{
		s3 = m_segments.at(leftIndex + 2);
	}
	//Strands::CubicInterpolation(s0.m_position, s1.m_position, s2.m_position, s3.m_position, retVal.m_position, retVal.m_front, a);
	//retVal.m_front = glm::normalize(retVal.m_front);
	//retVal.m_up = Strands::CubicInterpolation(s0.m_up, s1.m_up, s2.m_up, s3.m_up, a);
	
	retVal.m_radius = glm::mix(s1.m_radius, s2.m_radius, a);//Strands::CubicInterpolation(s0.m_radius, s1.m_radius, s2.m_radius, s3.m_radius, a);
	retVal.m_theta = glm::mix(s1.m_theta, s2.m_theta, a); //Strands::CubicInterpolation(s0.m_theta, s1.m_theta, s2.m_theta, s3.m_theta, a);
	retVal.m_leftHeightOffset = glm::mix(s1.m_leftHeightOffset, s2.m_leftHeightOffset, a); //Strands::CubicInterpolation(s0.m_leftHeightOffset, s1.m_leftHeightOffset, s2.m_leftHeightOffset, s3.m_leftHeightOffset, a);
	retVal.m_rightHeightOffset = glm::mix(s1.m_rightHeightOffset, s2.m_rightHeightOffset, a); //Strands::CubicInterpolation(s0.m_rightHeightOffset, s1.m_rightHeightOffset, s2.m_rightHeightOffset, s3.m_rightHeightOffset, a);

	retVal.m_position = glm::mix(s1.m_position, s2.m_position, a);
	retVal.m_front = glm::normalize(glm::mix(s1.m_front, s2.m_front, a));
	retVal.m_up = glm::normalize(glm::mix(s1.m_up, s2.m_up, a));
	retVal.m_up = glm::normalize(glm::cross(glm::cross(retVal.m_front, retVal.m_up), retVal.m_front));
	return retVal;
}

void SorghumSpline::Subdivide(const float subdivisionDistance, std::vector<SorghumSplineSegment>& subdividedSegments) const
{
	std::vector<float> lengths;
	lengths.resize(m_segments.size() - 1);
	for(int i = 0; i < m_segments.size() - 1; i++)
	{
		lengths[i] = glm::distance(m_segments.at(i).m_position, m_segments.at(i + 1).m_position);
	}
	int currentIndex = 0;
	float accumulatedDistance = 0.f;
	subdividedSegments.emplace_back(m_segments.front());
	while(true)
	{
		accumulatedDistance += subdivisionDistance;
		const auto currentSegmentLength = lengths.at(currentIndex);
		if(accumulatedDistance > currentSegmentLength)
		{
			currentIndex++;
			accumulatedDistance -= currentSegmentLength;
		}
		if (currentIndex < lengths.size() - 1) subdividedSegments.emplace_back(Interpolate(currentIndex, accumulatedDistance / currentSegmentLength));
		else break;
	}
}
