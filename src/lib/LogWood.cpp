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

glm::vec4 LogWoodIntersection::GetColor(const float angle) const
{
	assert(m_boundary.size() == 360);
	const int startIndex = static_cast<int>(angle) % 360;
	const float a = angle - static_cast<int>(angle);
	const int endIndex = (startIndex + 1) % 360;
	return glm::mix(m_boundary[startIndex].m_color, m_boundary[endIndex].m_color, a);
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
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int index = height / heightStep;
	const float a = (height - heightStep * index) / heightStep;
	const auto actualAngle = glm::mod(angle, 360.0f);
	if (index < m_intersections.size() - 1)
	{
		return glm::mix(m_intersections.at(index).GetBoundaryPoint(actualAngle),
			m_intersections.at(index + 1).GetBoundaryPoint(actualAngle), a);
	}
	return m_intersections.back().GetBoundaryPoint(actualAngle);
}

float LogWood::GetCenterDistance(const float height, const float angle) const
{
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int index = height / heightStep;
	const float a = (height - heightStep * index) / heightStep;
	const auto actualAngle = glm::mod(angle, 360.0f);
	if (index < m_intersections.size() - 1)
	{
		return glm::mix(m_intersections.at(index).GetCenterDistance(actualAngle),
			m_intersections.at(index + 1).GetCenterDistance(actualAngle), a);
	}
	return m_intersections.back().GetCenterDistance(actualAngle);
}

float LogWood::GetDefectStatus(const float height, const float angle) const
{
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int index = height / heightStep;
	const float a = (height - heightStep * index) / heightStep;
	const auto actualAngle = glm::mod(angle, 360.0f);
	if (index < m_intersections.size() - 1)
	{
		return glm::mix(m_intersections.at(index).GetDefectStatus(actualAngle),
			m_intersections.at(index + 1).GetDefectStatus(actualAngle), a);
	}
	return m_intersections.back().GetDefectStatus(actualAngle);
}

glm::vec4 LogWood::GetColor(const float height, const float angle) const
{
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int index = height / heightStep;
	const float a = (height - heightStep * index) / heightStep;
	const auto actualAngle = glm::mod(angle, 360.0f);
	if (index < m_intersections.size() - 1)
	{
		return glm::mix(m_intersections.at(index).GetColor(actualAngle),
			m_intersections.at(index + 1).GetColor(actualAngle), a);
	}
	return m_intersections.back().GetColor(actualAngle);
}

LogWoodIntersectionBoundaryPoint& LogWood::GetBoundaryPoint(const float height, const float angle)
{
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int index = height / heightStep;
	const float a = (height - heightStep * index) / heightStep;
	const auto actualAngle = glm::mod(angle, 360.0f);
	if (index < m_intersections.size() - 1)
	{
		if (a < 0.5f)
		{
			return m_intersections.at(index).m_boundary.at(actualAngle);
		}
		return m_intersections.at(index + 1).m_boundary.at(actualAngle);
	}
	return m_intersections.back().m_boundary.at(actualAngle);
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
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int index = height / heightStep;
	const float a = (height - heightStep * index) / heightStep;
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
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int heightStepSize = heightRange / heightStep;
	for (int yIndex = -heightStepSize; yIndex <= heightStepSize; yIndex++)
	{
		const auto sinVal = glm::abs(static_cast<float>(yIndex)) / heightStepSize;
		const auto sinAngle = glm::asin(sinVal);
		const float maxAngleRange = glm::cos(sinAngle);
		for (int xIndex = -angleRange; xIndex <= angleRange; xIndex++)
		{
			const auto actualYIndex = yIndex + height / heightStep;
			if (actualYIndex < 0 || actualYIndex >= m_intersections.size()) continue;
			if (glm::abs(static_cast<float>(xIndex)) / angleRange > maxAngleRange) continue;
			const auto actualXIndex = static_cast<int>(xIndex + angle) % 360;
			m_intersections.at(actualYIndex).m_boundary.at(actualXIndex).m_defectStatus = 1.0f;
		}
	}
}

void LogWood::EraseDefectRegion(const float height, const float angle, const float heightRange, const float angleRange)
{
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	const int heightStepSize = heightRange / heightStep;
	for (int yIndex = -heightStepSize; yIndex <= heightStepSize; yIndex++)
	{
		const auto sinVal = glm::abs(static_cast<float>(yIndex)) / heightStepSize;
		const auto sinAngle = glm::asin(sinVal);
		const float maxAngleRange = glm::cos(sinAngle);
		for (int xIndex = -angleRange; xIndex <= angleRange; xIndex++)
		{
			const auto actualYIndex = yIndex + height / heightStep;
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
		for (auto& boundaryPoint : intersection.m_boundary) {
			boundaryPoint.m_defectStatus = 0.0f;
			boundaryPoint.m_color = glm::vec4(212.f / 255, 175.f / 255, 55.f / 255, 1);
		}
	}
}


bool LogWood::RayCastSelection(const glm::mat4& transform, const float pointDistanceThreshold, const Ray& ray, float& height, float& angle) const
{
	float minDistance = FLT_MAX;
	bool found = false;
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	for (int yIndex = 0; yIndex < m_intersections.size(); yIndex++)
	{
		const auto testHeight = yIndex * heightStep;
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

std::vector<LogGradeFaceCutting> CalculateCuttings(const std::vector<bool>& defectMarks, const float heightStep, const float minDistance)
{
	int lastDefectIndex = 0.0f;
	std::vector<LogGradeFaceCutting> cuttings{};
	for(int intersectionIndex = 0; intersectionIndex < defectMarks.size(); intersectionIndex++)
	{
		if(defectMarks[intersectionIndex] || intersectionIndex == defectMarks.size() - 1)
		{
			if(heightStep * (intersectionIndex - lastDefectIndex) >= minDistance)
			{
				LogGradeFaceCutting cutting;
				cutting.m_start = heightStep * lastDefectIndex;
				cutting.m_end = heightStep * intersectionIndex;
				cuttings.emplace_back(cutting);
			}
			lastDefectIndex = intersectionIndex;
		}
	}
	return cuttings;
}

bool TestCutting(const std::vector<LogGradeFaceCutting>& cuttings, const int maxNumber, const float minProportion, const float logLength, float& minCuttingLength, float& proportion)
{
	const auto cuttingNumber = cuttings.size();
	float cuttingSumLength = 0.0f;
	minCuttingLength = FLT_MAX;
	for (const auto& cutting : cuttings)
	{
		const float length = cutting.m_end - cutting.m_start;
		cuttingSumLength += length;
		minCuttingLength = glm::min(minCuttingLength, length);
	}
	proportion = cuttingSumLength / logLength;
	if (cuttingNumber <= maxNumber && proportion >= minProportion)
	{
		return true;
	}
	return false;
}

LogGrading LogWood::CalculateGradingData(const int angleOffset) const
{
	LogGrading logGradingData{};
	logGradingData.m_angleOffset = angleOffset;
	logGradingData.m_scalingDiameter = GetMinAverageDistance();
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	for(int faceIndex = 0; faceIndex < 4; faceIndex++)
	{
		auto& face = logGradingData.m_faces[faceIndex];
		face.m_startAngle = (angleOffset + faceIndex * 90) % 360;
		face.m_endAngle = (angleOffset + faceIndex * 90 + 90) % 360;
		std::vector<bool> defectMarks;
		defectMarks.resize(m_intersections.size());
		for(int intersectionIndex = 0; intersectionIndex < m_intersections.size(); intersectionIndex++)
		{
			const auto& intersection = m_intersections[intersectionIndex];
			for(int angle = 0; angle < 90; angle++)
			{
				const int actualAngle = (face.m_startAngle + angle) % 360;
				if(intersection.m_boundary[actualAngle].m_defectStatus != 0.0f)
				{
					defectMarks[intersectionIndex] = true;
					break;
				}
			}
		}
		bool succeed = false;
		if(m_butt 
			&& logGradingData.m_scalingDiameter >= 0.3302f && logGradingData.m_scalingDiameter <= 0.4064f
			&& m_lengthWithoutTrim >= 3.048f)
		{
			//F1: Butt, Scaling diameter 13-15(16), Length 10+
			const auto cuttings7 = CalculateCuttings(defectMarks, heightStep, 2.1336f);
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings7, 2, 5.0f / 6.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if(succeed)
			{
				face.m_faceGrade = 1;
				face.m_cuttings = cuttings7;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}
		if(!succeed && logGradingData.m_scalingDiameter > 0.4064f && logGradingData.m_scalingDiameter <= 0.508f
			&& m_lengthWithoutTrim >= 3.048f)
		{
			//F1: Butt & uppers, Scaling diameter 16-19(20), Length 10+
			const auto cuttings5 = CalculateCuttings(defectMarks, heightStep, 1.524f);
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings5, 2, 5.0f / 6.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if (succeed)
			{
				face.m_faceGrade = 2;
				face.m_cuttings = cuttings5;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}
		
		if(!succeed && logGradingData.m_scalingDiameter > 0.508f
			&& m_lengthWithoutTrim >= 3.048f)
		{
			//F1: Butt & uppers, Scaling diameter 20+, Length 10+
			const auto cuttings3 = CalculateCuttings(defectMarks, heightStep, 0.9144f);
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings3, 2, 5.0f / 6.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if (succeed)
			{
				face.m_faceGrade = 3;
				face.m_cuttings = cuttings3;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}

		const auto cuttings3 = CalculateCuttings(defectMarks, heightStep, 0.9144f);
		if (!succeed && logGradingData.m_scalingDiameter > 0.2794f
			&& m_lengthWithoutTrim >= 3.048f)
		{
			//F2: Butt & uppers, Scaling diameter 11+, Length 10+
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings3, 2, 2.0f / 3.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if (succeed)
			{
				face.m_faceGrade = 4;
				face.m_cuttings = cuttings3;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}
		if (!succeed && logGradingData.m_scalingDiameter > 0.3048f
			&& m_lengthWithoutTrim > 2.4384f && m_lengthWithoutTrim <= 3.048f)
		{
			//F2: Butt & uppers, Scaling diameter 12+, Length 8-9(10)
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings3, 2, 3.0f / 4.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if (succeed)
			{
				face.m_faceGrade = 5;
				face.m_cuttings = cuttings3;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}
		if (!succeed && logGradingData.m_scalingDiameter > 0.3048f
			&& m_lengthWithoutTrim > 3.048f && m_lengthWithoutTrim <= 3.6576f)
		{
			//F2: Butt & uppers, Scaling diameter 12+, Length 10-11(12)
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings3, 2, 2.0f / 3.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if (succeed)
			{
				face.m_faceGrade = 6;
				face.m_cuttings = cuttings3;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}
		if (!succeed && logGradingData.m_scalingDiameter > 0.3048f
			&& m_lengthWithoutTrim > 3.6576f)
		{
			//F2: Butt & uppers, Scaling diameter 12+, Length 12+
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings3, 3, 2.0f / 3.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if (succeed)
			{
				face.m_faceGrade = 7;
				face.m_cuttings = cuttings3;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}
		const auto cuttings2 = CalculateCuttings(defectMarks, heightStep, 0.6096f);
		if (!succeed && logGradingData.m_scalingDiameter > 0.2032f
			&& m_lengthWithoutTrim > 2.4384f)
		{
			//F3: Butt & uppers, Scaling diameter 12+, Length 12+
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings2, 999, 1.0f / 2.0f, m_lengthWithoutTrim, minCuttingLength, proportion);
			if (succeed)
			{
				face.m_faceGrade = 8;
				face.m_cuttings = cuttings2;
				face.m_clearCuttingMinLength = minCuttingLength;
				face.m_clearCuttingMinProportion = proportion;
			}
		}
		if(!succeed)
		{
			float minCuttingLength = 0.0f;
			float proportion = 0.0f;
			succeed = TestCutting(cuttings2, 999, 0.f, m_lengthWithoutTrim, minCuttingLength, proportion);
			face.m_faceGrade = 9;
			face.m_cuttings = cuttings2;
			face.m_clearCuttingMinLength = minCuttingLength;
			face.m_clearCuttingMinProportion = proportion;
		}

		
	}
	return logGradingData;
}

void LogWood::ColorBasedOnGrading(const LogGrading& logGradingData)
{
	const float heightStep = m_lengthWithoutTrim / m_intersections.size();
	for (const auto& face : logGradingData.m_faces)
	{
		for (int intersectionIndex = 0; intersectionIndex < m_intersections.size(); intersectionIndex++)
		{
			auto& intersection = m_intersections[intersectionIndex];
			for (int angle = 0; angle < 90; angle++)
			{
				const int actualAngle = (face.m_startAngle + angle) % 360;
				auto& point = intersection.m_boundary[actualAngle];
				if (point.m_defectStatus != 0.0f)
				{
					point.m_color = glm::vec4(1, 0, 0, 1);
					continue;
				}
				const float height = heightStep * intersectionIndex;
				bool isCutting = false;
				for(const auto& cutting : face.m_cuttings)
				{
					if(height >= cutting.m_start && height <= cutting.m_end)
					{
						if(face.m_faceGrade <= 3)
						{
							point.m_color = glm::vec4(212.f / 255, 175.f / 255, 55.f / 255, 1);
						}else if(face.m_faceGrade <= 7)
						{
							point.m_color = glm::vec4(170.f / 255, 169.f / 255, 173.f / 255, 1);
						}else if(face.m_faceGrade <= 8)
						{
							point.m_color = glm::vec4(131.f / 255, 105.f / 255, 83.f / 255, 1);
						}else
						{
							point.m_color = glm::vec4(0.1f, 0.1f, 0.1f, 1);
						}
						
						isCutting = true;
						break;
					}
				}
				if(!isCutting)
				{
					point.m_color = glm::vec4(0.2f, 0.0f, 0.0f, 1);
				}
			}
		}
	}
}
