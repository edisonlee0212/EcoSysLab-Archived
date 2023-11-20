#include "EnvironmentGrid.hpp"
using namespace EcoSysLab;

float EnvironmentGrid::IlluminationEstimation(const glm::vec3& position, glm::vec3& lightDirection) const
{
	const auto& data = m_voxel.Peek(position);
	const float lightIntensity = glm::max(0.0f, 1.0f - data.m_shadowIntensity);
	if (lightIntensity == 0.0f)
	{
		lightDirection = glm::vec3(0.0f);
	}
	else if(data.m_shadowIntensity == 0.0f)
	{
		lightDirection = glm::vec3(0.0f, 1.0f, 0.0f);
	}
	else{
		lightDirection = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f) + glm::normalize(data.m_shadowDirection) * data.m_shadowIntensity);
	}

	return lightIntensity;
}

void EnvironmentGrid::AddShadowValue(const glm::vec3& position, float value)
{
	const auto voxelMinBound = m_voxel.GetMinBound();
	const auto dx = m_voxel.GetVoxelSize();
	const auto voxelResolution = m_voxel.GetResolution();
	if (m_settings.m_distanceMultiplier == 0.0f) return;
	const float maxRadius = glm::pow(value, 1.0f / m_settings.m_distancePowerFactor) / m_settings.m_distanceMultiplier;
	const int xCenter = (position.x - voxelMinBound.x) / dx;
	const int yCenter = (position.y - voxelMinBound.y) / dx;
	const int zCenter = (position.z - voxelMinBound.z) / dx;
		
	for (int y = yCenter - static_cast<int>(maxRadius / dx); y <= yCenter - 1; y++)
	{
		if(y < 0 || y > voxelResolution.y - 1) continue;
		for (int x = xCenter - static_cast<int>(maxRadius / dx); x <= xCenter + static_cast<int>(maxRadius / dx); x++)
		{
			if (x < 0 || x > voxelResolution.x - 1) continue;
			for (int z = zCenter - static_cast<int>(maxRadius / dx); z <= zCenter + static_cast<int>(maxRadius / dx); z++)
			{
				if (z < 0 || z > voxelResolution.z - 1) continue;
				auto voxelCenter = m_voxel.GetPosition({ x, y, z });
				const auto positionDiff = voxelCenter - position;
				
				const auto angle = glm::atan(glm::sqrt(positionDiff.x * positionDiff.x + positionDiff.z * positionDiff.z) / positionDiff.y);
				const auto distance = glm::length(positionDiff);
				const float shadowIntensity = glm::cos(angle) * value - glm::pow(distance * m_settings.m_distanceMultiplier, m_settings.m_distancePowerFactor);
				if (shadowIntensity < 0.0f) continue;
				const auto direction = glm::normalize(positionDiff);
				auto& data = m_voxel.Ref(glm::ivec3(x, y, z));
				data.m_shadowIntensity += shadowIntensity;
				data.m_shadowDirection += direction * shadowIntensity;
			}
		}
	}
}

void EnvironmentGrid::AddBiomass(const glm::vec3& position, const float value)
{
	auto& data = m_voxel.Ref(position);
	data.m_totalBiomass += value;
}

void EnvironmentGrid::AddNode(const InternodeVoxelRegistration& registration)
{
	auto& data = m_voxel.Ref(registration.m_position);
	data.m_internodeVoxelRegistrations.emplace_back(registration);
}

