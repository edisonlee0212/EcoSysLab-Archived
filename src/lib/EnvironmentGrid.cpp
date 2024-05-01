#include "EnvironmentGrid.hpp"
using namespace EcoSysLab;

float EnvironmentGrid::Sample(const glm::vec3& position, glm::vec3& lightDirection) const
{
	const auto coordinate = m_voxel.GetCoordinate(position);
	const auto& data = m_voxel.Peek(coordinate);
	lightDirection = data.m_lightDirection;
	return data.m_lightIntensity;

	const auto resolution = m_voxel.GetResolution();
	const auto voxelCenter = m_voxel.GetPosition(coordinate);
	float topIntensity;
	if (coordinate.y < resolution.y - 1) topIntensity = m_voxel.Peek(glm::ivec3(coordinate.x, coordinate.y + 1, coordinate.z)).m_lightIntensity;
	else topIntensity = data.m_lightIntensity;

	topIntensity = (topIntensity + data.m_lightIntensity) / 2.f;
	float bottomIntensity;
	if (coordinate.y > 0) bottomIntensity = m_voxel.Peek(glm::ivec3(coordinate.x, coordinate.y - 1, coordinate.z)).m_lightIntensity;
	else bottomIntensity = data.m_lightIntensity;
	bottomIntensity = (bottomIntensity + data.m_lightIntensity) / 2.f;
	const float a = (position.y - voxelCenter.y + 0.5f * m_voxelSize) / m_voxelSize;
	assert(a < 1.f);
	return glm::mix(bottomIntensity, topIntensity, a);
}

void EnvironmentGrid::AddShadowValue(const glm::vec3& position, const float value)
{
	auto& data = m_voxel.Ref(position);
	data.m_selfShadow += value;
}

void EnvironmentGrid::LightPropagation()
{
	const auto resolution = m_voxel.GetResolution();
	const int shadowDiskSize = glm::ceil(m_settings.m_detectionRadius / m_voxelSize);
	Jobs::RunParallelFor(resolution.x * resolution.z, [&](unsigned i)
		{
			const int x = i / resolution.z;
			const int z = i % resolution.z;
			auto& targetVoxel = m_voxel.Ref(glm::ivec3(x, resolution.y - 1, z));
			targetVoxel.m_lightIntensity = 1.f;
		});
	for (int y = resolution.y - 2; y >= 0; y--) {
		Jobs::RunParallelFor(resolution.x * resolution.z, [&](unsigned i, unsigned workerIndex)
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
						if (distance > m_settings.m_detectionRadius) continue;
						const float baseLossFactor = m_voxelSize / distance;
						const float distanceLoss = glm::pow(glm::max(0.0f, baseLossFactor), m_settings.m_shadowDistanceLoss);
						if (x + xOffset < 0 || x + xOffset > resolution.x - 1 || z + zOffset < 0 || z + zOffset > resolution.z - 1)
						{
							sum += distanceLoss;
							max += distanceLoss;
						}
						else {
							const auto& targetVoxel = m_voxel.Ref(otherVoxelCenter);
							sum += glm::clamp(targetVoxel.m_lightIntensity * distanceLoss * (1.f - targetVoxel.m_selfShadow), 0.0f, 1.0f);
							max += distanceLoss;
						}
					}
				}
				auto& voxel = m_voxel.Ref(glm::ivec3(x, y, z));
				voxel.m_lightIntensity = sum / max;
			}
		);
	}
	for (int iteration = 0; iteration < m_settings.m_blurIteration; iteration++) {
		for (int y = resolution.y - 2; y >= 0; y--) {
			Jobs::RunParallelFor(resolution.x * resolution.z, [&](unsigned i, unsigned workerIndex)
				{
					const int x = i / resolution.z;
					const int z = i % resolution.z;

					const float selfIntensity = m_voxel.Ref(glm::ivec3(x, y, z)).m_lightIntensity;
					float intensity = selfIntensity * .4f;
					intensity += m_voxel.Ref(glm::ivec3(x, y + 1, z)).m_lightIntensity * .1f;
					if (y > 0) intensity += m_voxel.Ref(glm::ivec3(x, y - 1, z)).m_lightIntensity * .1f;
					else intensity += selfIntensity * .1f;

					if (x > 0) intensity += m_voxel.Ref(glm::ivec3(x - 1, y, z)).m_lightIntensity * .1f;
					else intensity += selfIntensity * .1f;

					if (z > 0) intensity += m_voxel.Ref(glm::ivec3(x, y, z - 1)).m_lightIntensity * .1f;
					else intensity += selfIntensity * .1f;

					if (x < resolution.x - 1) intensity += m_voxel.Ref(glm::ivec3(x + 1, y, z)).m_lightIntensity * .1f;
					else intensity += selfIntensity * .1f;

					if (z < resolution.z - 1) intensity += m_voxel.Ref(glm::ivec3(x, y, z + 1)).m_lightIntensity * .1f;
					else intensity += selfIntensity * .1f;

					m_voxel.Ref(glm::ivec3(x, y, z)).m_lightIntensity = intensity;
				}
			);
		}
	}
	const int lightSpaceSize = glm::ceil(m_settings.m_detectionRadius / m_voxelSize);
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
							const float distance = glm::length(positionDiff);
							if (distance > m_settings.m_detectionRadius) continue;

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

