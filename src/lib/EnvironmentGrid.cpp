#include "EnvironmentGrid.hpp"
using namespace EcoSysLab;

float EnvironmentGrid::Sample(const glm::vec3& position, glm::vec3& lightDirection) const
{
	const auto& data = m_voxel.Peek(position);
	lightDirection = data.m_lightDirection;
	return data.m_lightIntensity;
}

void EnvironmentGrid::AddShadowValue(const glm::vec3& position, float value)
{
	auto& data = m_voxel.Ref(position);
	data.m_selfShadow += value;
}

void EnvironmentGrid::LightPropagation()
{
	const auto resolution = m_voxel.GetResolution();
	const int shadowDiskSize = glm::ceil(m_settings.m_shadowDetectionRadius / m_voxelSize);
	Jobs::RunParallelFor(resolution.x * resolution.z, [&](unsigned i)
	{
			const int x = i / resolution.z;
			const int z = i % resolution.z;
			auto& targetVoxel = m_voxel.Ref(glm::ivec3(x, resolution.y - 1, z));
			targetVoxel.m_lightIntensity = 1.f;
	});
	for (int y = resolution.y - 2; y >= 0; y--) {
		Jobs::RunParallelFor(resolution.x * resolution.z, [&](unsigned i)
			{
				const int x = i / resolution.z;
				const int z = i % resolution.z;
				float sum = 0.0f;
				float max = 0.f;
				for (int xOffset = -shadowDiskSize; xOffset <= shadowDiskSize; xOffset++)
				{
					
					for (int zOffset = -shadowDiskSize; zOffset <= shadowDiskSize; zOffset++)
					{
						if (y + 1 > resolution.y - 1) continue;
						const auto otherVoxelCenter = glm::ivec3(x + xOffset, y + 1, z + zOffset);
						const auto positionDiff = m_voxel.GetPosition(otherVoxelCenter) - m_voxel.GetPosition(glm::ivec3(x, y, z));
						const float distance = glm::length(positionDiff);
						if (distance > m_settings.m_shadowDetectionRadius) continue;
						const float distanceLoss = glm::pow(glm::max(0.0f, (m_settings.m_shadowDetectionRadius - distance) / m_settings.m_shadowDetectionRadius), m_settings.m_shadowDistanceLoss);
						if (x + xOffset < 0 || x + xOffset > resolution.x - 1 || z + zOffset < 0 || z + zOffset > resolution.z - 1)
						{
							sum += distanceLoss;
							max += distanceLoss;
						}
						else {
							const auto& targetVoxel = m_voxel.Ref(otherVoxelCenter);
							
							sum += glm::clamp(targetVoxel.m_lightIntensity * distanceLoss * (1.f - m_settings.m_shadowBaseLoss * targetVoxel.m_selfShadow), 0.0f, 1.0f);
							max += distanceLoss;
						}
					}
				}
				auto& voxel = m_voxel.Ref(glm::ivec3(x, y, z));
				voxel.m_lightIntensity = sum / max;
				voxel.m_shadowIntensity = 1.f - voxel.m_lightIntensity;
			}
		);
	}

	const int lightSpaceSize = glm::ceil(m_settings.m_lightDetectionRadius / m_voxelSize);
	for (int y = resolution.y - 1; y >= 0; y--) {
		Jobs::RunParallelFor(resolution.x * resolution.z, [&](unsigned i)
			{
				const int x = i / resolution.z;
				const int z = i % resolution.z;
				glm::vec3 sum = glm::vec3(0.0f);
				for (int xOffset = -lightSpaceSize; xOffset <= lightSpaceSize; xOffset++)
				{
					for (int zOffset = -lightSpaceSize; zOffset <= lightSpaceSize; zOffset++)
					{
						for (int yOffset = 1; yOffset <= lightSpaceSize; yOffset++)
						{
							if (y + yOffset < 0 || y + yOffset > resolution.y - 1) continue;
							const auto otherVoxelCenter = glm::ivec3(x + xOffset, y + yOffset, z + zOffset);
							const auto positionDiff = m_voxel.GetPosition(otherVoxelCenter) - m_voxel.GetPosition(glm::ivec3(x, y, z));
							const float distance = glm::length(positionDiff); // glm::sqrt(static_cast<float>(xOffset) * static_cast<float>(xOffset) + static_cast<float>(zOffset) * static_cast<float>(zOffset));
							if (distance > m_settings.m_lightDetectionRadius) continue;

							if (x + xOffset < 0 || x + xOffset > resolution.x - 1 || z + zOffset < 0 || z + zOffset > resolution.z - 1)
							{
								sum += positionDiff;
							}
							else {
								const auto& targetVoxel = m_voxel.Ref(otherVoxelCenter);
								sum += targetVoxel.m_lightIntensity * positionDiff;
							}
						}
					}
				}
				auto& voxel = m_voxel.Ref(glm::ivec3(x, y, z));
				if (glm::length(sum) > glm::epsilon<float>()) voxel.m_lightDirection = glm::normalize(sum);
				else voxel.m_lightDirection = glm::vec3(0.0f, 1.0f, 0.0f);
			}
		);
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

