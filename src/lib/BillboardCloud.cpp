#include "BillboardCloud.hpp"

using namespace EvoEngine;

BillboardCloud::Cluster BillboardCloud::Project(const Cluster& cluster, const BillboardProjectSettings& settings)
{
	Cluster projectedCluster{};
	const auto& plane = cluster.m_modelSpaceProjectionPlane;
	for (const auto& instancedElement : cluster.m_instancedElements)
	{
		const auto& content = instancedElement.m_content;
		const auto& modelSpaceTransform = instancedElement.m_modelSpaceTransform;
		projectedCluster.m_elements.emplace_back();
		const auto& triangles = content.m_triangles;
		const auto& vertices = content.m_mesh->UnsafeGetVertices();
		const auto& particleInfoList = content.m_particleInfoList->PeekParticleInfoList();
		std::vector<Vertex> projectedVertices;
		std::vector<glm::uvec3> projectedTriangles;

		projectedTriangles.resize(triangles.size() * particleInfoList.size());
		projectedVertices.resize(triangles.size() * 3 * particleInfoList.size());

		Jobs::RunParallelFor(particleInfoList.size(), [&](const unsigned particleIndex)
			{
				const auto& particleInfo = particleInfoList[particleIndex];
				const auto transform = modelSpaceTransform.m_value * particleInfo.m_instanceMatrix.m_value;
				const auto vertexStartIndex = triangles.size() * 3 * particleIndex;
				const auto triangleStartIndex = triangles.size() * particleIndex;
				for (auto triangleIndex = 0; triangleIndex < triangles.size(); triangleIndex++) {
					projectedTriangles[triangleIndex + triangleStartIndex] = glm::uvec3(
						triangleIndex * 3 + vertexStartIndex, 
						triangleIndex * 3 + 1 + vertexStartIndex, 
						triangleIndex * 3 + 2 + vertexStartIndex);
					//Vertices of projected triangle.
					auto& pp0 = projectedVertices[triangleIndex * 3 + vertexStartIndex];
					auto& pp1 = projectedVertices[triangleIndex * 3 + 1 + vertexStartIndex];
					auto& pp2 = projectedVertices[triangleIndex * 3 + 2 + vertexStartIndex];

					const auto& v0 = vertices[triangles[triangleIndex].x];
					const auto& v1 = vertices[triangles[triangleIndex].y];
					const auto& v2 = vertices[triangles[triangleIndex].z];

					const auto p0 = transform * glm::vec4(v0.m_position, 1.f);
					const auto p1 = transform * glm::vec4(v1.m_position, 1.f);
					const auto p2 = transform * glm::vec4(v2.m_position, 1.f);

					const float t0 = (plane.m_a * p0.x + plane.m_b * p0.y + plane.m_c * p0.z + plane.m_d) /
						(plane.m_a * plane.m_a + plane.m_b * plane.m_b + plane.m_c * plane.m_c);
					pp0 = v0;
					pp0.m_normal = transform * glm::vec4(v0.m_normal, 0.f);
					pp0.m_tangent = transform * glm::vec4(v0.m_tangent, 0.f);
					pp0.m_position = glm::vec3(
						p0.x - plane.m_a * t0,
						p0.y - plane.m_b * t0,
						p0.z - plane.m_c * t0);

					const float t1 = (plane.m_a * p1.x + plane.m_b * p1.y + plane.m_c * p1.z + plane.m_d) /
						(plane.m_a * plane.m_a + plane.m_b * plane.m_b + plane.m_c * plane.m_c);
					pp1 = v1;
					pp1.m_normal = transform * glm::vec4(v1.m_normal, 0.f);
					pp1.m_tangent = transform * glm::vec4(v1.m_tangent, 0.f);
					pp1.m_position = glm::vec3(
						p1.x - plane.m_a * t1,
						p1.y - plane.m_b * t1,
						p1.z - plane.m_c * t1);

					const float t2 = (plane.m_a * p2.x + plane.m_b * p2.y + plane.m_c * p2.z + plane.m_d) /
						(plane.m_a * plane.m_a + plane.m_b * plane.m_b + plane.m_c * plane.m_c);
					pp2 = v2;
					pp2.m_normal = transform * glm::vec4(v2.m_normal, 0.f);
					pp2.m_tangent = transform * glm::vec4(v2.m_tangent, 0.f);
					pp2.m_position = glm::vec3(
						p2.x - plane.m_a * t2,
						p2.y - plane.m_b * t2,
						p2.z - plane.m_c * t2);
				}
			});

		const auto projectedMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto& projectedContent = projectedCluster.m_elements.back().m_content;
		projectedContent.m_mesh = projectedMesh;
		projectedContent.m_material = content.m_material;
		projectedContent.m_triangles = projectedTriangles;
		projectedMesh->SetVertices({ true, true, true, true }, projectedVertices, projectedTriangles);
	}
	for (const auto& element : cluster.m_elements)
	{
		const auto& content = element.m_content;
		const auto& transform = element.m_modelSpaceTransform;
		projectedCluster.m_elements.emplace_back();
		const auto& triangles = content.m_triangles;
		const auto& vertices = content.m_mesh->UnsafeGetVertices();

		std::vector<Vertex> projectedVertices;
		std::vector<glm::uvec3> projectedTriangles;

		projectedTriangles.resize(triangles.size());
		projectedVertices.resize(triangles.size() * 3);

		Jobs::RunParallelFor(triangles.size(), [&](const unsigned triangleIndex)
			{
				projectedTriangles[triangleIndex] = glm::uvec3(triangleIndex * 3, triangleIndex * 3 + 1, triangleIndex * 3 + 2);
				//Vertices of projected triangle.
				auto& pp0 = projectedVertices[triangleIndex * 3];
				auto& pp1 = projectedVertices[triangleIndex * 3 + 1];
				auto& pp2 = projectedVertices[triangleIndex * 3 + 2];

				const auto& v0 = vertices[triangles[triangleIndex].x];
				const auto& v1 = vertices[triangles[triangleIndex].y];
				const auto& v2 = vertices[triangles[triangleIndex].z];

				const auto p0 = transform.m_value * glm::vec4(v0.m_position, 1.f);
				const auto p1 = transform.m_value * glm::vec4(v1.m_position, 1.f);
				const auto p2 = transform.m_value * glm::vec4(v2.m_position, 1.f);

				const float t0 = (plane.m_a * p0.x + plane.m_b * p0.y + plane.m_c * p0.z + plane.m_d) /
					(plane.m_a * plane.m_a + plane.m_b * plane.m_b + plane.m_c * plane.m_c);
				pp0 = v0;
				pp0.m_normal = transform.m_value * glm::vec4(v0.m_normal, 0.f);
				pp0.m_tangent = transform.m_value * glm::vec4(v0.m_tangent, 0.f);
				pp0.m_position = glm::vec3(
					p0.x - plane.m_a * t0,
					p0.y - plane.m_b * t0,
					p0.z - plane.m_c * t0);

				const float t1 = (plane.m_a * p1.x + plane.m_b * p1.y + plane.m_c * p1.z + plane.m_d) /
					(plane.m_a * plane.m_a + plane.m_b * plane.m_b + plane.m_c * plane.m_c);
				pp1 = v1;
				pp1.m_normal = transform.m_value * glm::vec4(v1.m_normal, 0.f);
				pp1.m_tangent = transform.m_value * glm::vec4(v1.m_tangent, 0.f);
				pp1.m_position = glm::vec3(
					p1.x - plane.m_a * t1,
					p1.y - plane.m_b * t1,
					p1.z - plane.m_c * t1);

				const float t2 = (plane.m_a * p2.x + plane.m_b * p2.y + plane.m_c * p2.z + plane.m_d) /
					(plane.m_a * plane.m_a + plane.m_b * plane.m_b + plane.m_c * plane.m_c);
				pp2 = v2;
				pp2.m_normal = transform.m_value * glm::vec4(v2.m_normal, 0.f);
				pp2.m_tangent = transform.m_value * glm::vec4(v2.m_tangent, 0.f);
				pp2.m_position = glm::vec3(
					p2.x - plane.m_a * t2,
					p2.y - plane.m_b * t2,
					p2.z - plane.m_c * t2);
			});

		const auto projectedMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto& projectedContent = projectedCluster.m_elements.back().m_content;
		projectedContent.m_mesh = projectedMesh;
		projectedContent.m_material = content.m_material;
		projectedContent.m_triangles = projectedTriangles;
		projectedMesh->SetVertices({ true, true, true, true }, projectedVertices, projectedTriangles);
	}
	return projectedCluster;
}
