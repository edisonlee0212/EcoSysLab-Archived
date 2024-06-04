#include "BillboardCloud.hpp"
using namespace EvoEngine;



std::vector<BillboardCloud::Cluster> BillboardCloud::StochasticClusterize(std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings)
{
	BoundingSphere boundingSphere;
	boundingSphere.Initialize(m_elements);
	const auto& settings = clusterizeSettings.m_foliageClusterizationSettings;
	float epsilon = boundingSphere.m_radius * glm::clamp(1.f - settings.m_density, 0.05f, 1.f);

	m_skippedTriangles.clear();
	auto remainingTriangles = operatingTriangles;
	std::vector<Cluster> retVal;

	int epoch = 0;
	while (!remainingTriangles.empty())
	{
		Cluster newCluster;
		float maxArea = 0.f;
		std::vector<int> selectedTriangleIndices;
		std::mutex voteMutex;
		Jobs::RunParallelFor(settings.m_iteration, [&](unsigned iteration)
			{
				int seedTriangleIndex = glm::linearRand(0, static_cast<int>(remainingTriangles.size()) - 1);
				ClusterTriangle seedTriangle = remainingTriangles.at(seedTriangleIndex);
				const auto perturb0 = glm::linearRand(-epsilon, epsilon);
				const auto perturb1 = glm::linearRand(-epsilon, epsilon);
				const auto perturb2 = glm::linearRand(-epsilon, epsilon);
				const auto& seedTriangleElement = m_elements.at(seedTriangle.m_elementIndex);
				const auto& seedTriangleIndices = seedTriangleElement.m_triangles.at(seedTriangle.m_triangleIndex);
				const auto seedTriangleNormal = CalculateNormal(seedTriangle);
				glm::vec3 seedTriangleP0 = seedTriangleElement.m_vertices.at(seedTriangleIndices.x).m_position + perturb0 * seedTriangleNormal;
				glm::vec3 seedTriangleP1 = seedTriangleElement.m_vertices.at(seedTriangleIndices.y).m_position + perturb1 * seedTriangleNormal;
				glm::vec3 seedTriangleP2 = seedTriangleElement.m_vertices.at(seedTriangleIndices.z).m_position + perturb2 * seedTriangleNormal;
				auto testPlaneNormal = glm::normalize(glm::cross(seedTriangleP0 - seedTriangleP1, seedTriangleP0 - seedTriangleP2));
				if (glm::dot(testPlaneNormal, seedTriangleNormal) < 0.f)
				{
					testPlaneNormal = -testPlaneNormal;
				}
				float testPlaneDistance = glm::dot(seedTriangleP0, testPlaneNormal);
				float area = 0.f;
				std::vector<int> currentPendingRemovalTriangles;
				std::vector<ClusterTriangle> trianglesForCluster;
				for (int testTriangleIndex = 0; testTriangleIndex < remainingTriangles.size(); testTriangleIndex++)
				{
					const auto& testTriangle = remainingTriangles.at(testTriangleIndex);
					const auto& testTriangleElement = m_elements.at(testTriangle.m_elementIndex);
					const auto& testTriangleIndices = testTriangleElement.m_triangles.at(testTriangle.m_triangleIndex);
					const auto& testTriangleP0 = testTriangleElement.m_vertices.at(testTriangleIndices.x).m_position;
					const auto& testTriangleP1 = testTriangleElement.m_vertices.at(testTriangleIndices.y).m_position;
					const auto& testTriangleP2 = testTriangleElement.m_vertices.at(testTriangleIndices.z).m_position;


					if (!settings.m_fillBand && glm::abs(testPlaneDistance - glm::dot(testTriangleP0, testPlaneNormal)) <= epsilon * settings.m_sampleRange
						&& glm::abs(testPlaneDistance - glm::dot(testTriangleP1, testPlaneNormal)) <= epsilon * settings.m_sampleRange
						&& glm::abs(testPlaneDistance - glm::dot(testTriangleP2, testPlaneNormal)) <= epsilon * settings.m_sampleRange)
					{
						trianglesForCluster.emplace_back(remainingTriangles.at(testTriangleIndex));
					}
					if (glm::abs(testPlaneDistance - glm::dot(testTriangleP0, testPlaneNormal)) > epsilon) continue;
					if (glm::abs(testPlaneDistance - glm::dot(testTriangleP1, testPlaneNormal)) > epsilon) continue;
					if (glm::abs(testPlaneDistance - glm::dot(testTriangleP2, testPlaneNormal)) > epsilon) continue;
					// increment projected area (Angular area Contribution)
					// use projected area Contribution
					float angle = glm::acos(glm::abs(glm::dot(testPlaneNormal, CalculateNormal(testTriangle))));
					float angular = (glm::pi<float>() / 2.f - angle) / (glm::pi<float>() / 2.f);
					area += CalculateArea(testTriangle) * angular;

					// save reference to T with billboard plane
					currentPendingRemovalTriangles.emplace_back(testTriangleIndex);
				}

				if (settings.m_fillBand)
				{
					for (auto& operatingTriangle : operatingTriangles)
					{
						const auto& testTriangle = operatingTriangle;
						const auto& testTriangleElement = m_elements.at(testTriangle.m_elementIndex);
						const auto& testTriangleIndices = testTriangleElement.m_triangles.at(testTriangle.m_triangleIndex);
						const auto& testTriangleP0 = testTriangleElement.m_vertices.at(testTriangleIndices.x).m_position;
						const auto& testTriangleP1 = testTriangleElement.m_vertices.at(testTriangleIndices.y).m_position;
						const auto& testTriangleP2 = testTriangleElement.m_vertices.at(testTriangleIndices.z).m_position;

						if (glm::abs(testPlaneDistance - glm::dot(testTriangleP0, testPlaneNormal)) <= epsilon * settings.m_sampleRange
							&& glm::abs(testPlaneDistance - glm::dot(testTriangleP1, testPlaneNormal)) <= epsilon * settings.m_sampleRange
							&& glm::abs(testPlaneDistance - glm::dot(testTriangleP2, testPlaneNormal)) <= epsilon * settings.m_sampleRange)
						{
							trianglesForCluster.emplace_back(operatingTriangle);
						}
					}
				}
				if (!currentPendingRemovalTriangles.empty()) {
					std::lock_guard lock(voteMutex);
					if (area > maxArea)
					{
						//Update cluster.
						newCluster.m_clusterPlane = Plane(testPlaneNormal, testPlaneDistance);
						newCluster.m_triangles = trianglesForCluster;
						selectedTriangleIndices = currentPendingRemovalTriangles;
					}
				}
			}
		);

		if (selectedTriangleIndices.empty())
		{
			m_skippedTriangles.insert(m_skippedTriangles.end(), remainingTriangles.begin(), remainingTriangles.end());
			break;
		}

		if(!newCluster.m_triangles.empty()) retVal.emplace_back(std::move(newCluster));

		//Remove selected triangle from the remaining triangle.
		for (auto it = selectedTriangleIndices.rbegin(); it != selectedTriangleIndices.rend(); ++it)
		{
			remainingTriangles[*it] = remainingTriangles.back();
			remainingTriangles.pop_back();
		}
		epoch++;
		if (settings.m_timeout != 0 && epoch >= settings.m_timeout)
		{
			EVOENGINE_ERROR("Stochastic clustering timeout!")
				break;
		}
	}
	return retVal;
}
