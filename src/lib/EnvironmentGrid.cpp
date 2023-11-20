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
	auto& data = m_voxel.Ref(position);
	data.m_shadowIntensity += value;
}

void EnvironmentGrid::ShadowPropagation()
{
	const auto resolution = m_voxel.GetResolution();
	for (int y = resolution.y - 2; y >= 0; y--) {
		Jobs::ParallelFor(resolution.x * resolution.z, [&](unsigned i)
			{
				const int x = i / resolution.z;
				const int z = i % resolution.z;
				float sum = 0.0f;
				auto shadowDirection = glm::vec3(0.0f);
				for(int xOffset = -2; xOffset <= 2; xOffset++)
				{
					for (int zOffset = -2; zOffset <= 2; zOffset++)
					{
						const float distance = glm::sqrt(static_cast<float>(xOffset) * static_cast<float>(xOffset) + static_cast<float>(zOffset) * static_cast<float>(zOffset));
						if(distance > 2.f) continue;
						if (x + xOffset < 0 || x + xOffset > resolution.x - 1) continue;
						if (z + zOffset < 0 || z + zOffset > resolution.z - 1) continue;
						const auto otherVoxelCenter = glm::ivec3(x + xOffset, y + 1, z + zOffset);
						const float shadowIntensity = m_voxel.Ref(otherVoxelCenter).m_shadowIntensity;
						float loss = (2.f - distance) / 2.f;
						loss = glm::pow(loss, m_settings.m_distancePowerFactor);
						sum += shadowIntensity * loss;
						const auto positionDiff = m_voxel.GetPosition(otherVoxelCenter) - m_voxel.GetPosition(glm::ivec3(x, y, z));
						const auto direction = glm::normalize(positionDiff);
						shadowDirection += direction * shadowIntensity;
					}
				}
				auto& voxel = m_voxel.Ref(glm::ivec3(x, y, z));
				voxel.m_shadowIntensity += m_settings.m_shadowPropagateLoss * sum;
				if (voxel.m_shadowIntensity > glm::epsilon<float>()) {
					voxel.m_shadowDirection = glm::normalize(shadowDirection);
				}else
				{
					voxel.m_shadowDirection = glm::vec3(0.0f);
				}
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

