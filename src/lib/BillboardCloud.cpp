#include "BillboardCloud.hpp"

using namespace EvoEngine;

void BillboardCloud::ProjectToPlane(const Vertex& v0, const Vertex& v1, const Vertex& v2,
	Vertex& pV0, Vertex& pV1, Vertex& pV2, const glm::mat4& transform)
{
	const auto p0 = transform * glm::vec4(v0.m_position, 1.f);
	const auto p1 = transform * glm::vec4(v1.m_position, 1.f);
	const auto p2 = transform * glm::vec4(v2.m_position, 1.f);
	pV0 = v0;
	pV0.m_normal = transform * glm::vec4(v0.m_normal, 0.f);
	pV0.m_tangent = transform * glm::vec4(v0.m_tangent, 0.f);
	pV0.m_position = p0;
	pV0.m_position.z = 0.f;
	pV1 = v1;
	pV1.m_normal = transform * glm::vec4(v1.m_normal, 0.f);
	pV1.m_tangent = transform * glm::vec4(v1.m_tangent, 0.f);
	pV1.m_position = p1;
	pV1.m_position.z = 0.f;
	pV2 = v2;
	pV2.m_normal = transform * glm::vec4(v2.m_normal, 0.f);
	pV2.m_tangent = transform * glm::vec4(v2.m_tangent, 0.f);
	pV2.m_position = p2;
	pV2.m_position.z = 0.f;

}

BillboardCloud::Cluster BillboardCloud::Project(const Cluster& cluster, const ProjectSettings& settings)
{
	Cluster projectedCluster{};
	glm::vec3 frontAxis = cluster.m_planeNormal;
	glm::vec3 upAxis = cluster.m_planeYAxis;
	glm::vec3 leftAxis = glm::normalize(glm::cross(frontAxis, upAxis));
	upAxis = glm::normalize(glm::cross(leftAxis, frontAxis));
	glm::mat4 viewMatrix = glm::lookAt(cluster.m_clusterCenter, cluster.m_clusterCenter + cluster.m_planeNormal, cluster.m_planeYAxis);
	//glm::mat4 rotateMatrix = glm::transpose(glm::mat4(glm::vec4(leftAxis, 0.0f), glm::vec4(upAxis, 0.0f), glm::vec4(frontAxis, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
	//glm::mat4 translateMatrix = glm::translate(-cluster.m_clusterCenter);
	//const auto viewMatrix = translateMatrix * rotateMatrix;
	for (const auto& instancedElement : cluster.m_instancedElements)
	{
		const auto& content = instancedElement.m_content;
		const auto& modelSpaceTransform = instancedElement.m_modelSpaceTransform.m_value * viewMatrix;
		projectedCluster.m_elements.emplace_back();
		auto& newElement = projectedCluster.m_elements.back();
		//newElement.m_modelSpaceTransform.m_value *= glm::inverse(viewMatrix);
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
				const auto transform = modelSpaceTransform * particleInfo.m_instanceMatrix.m_value;
				const auto vertexStartIndex = triangles.size() * 3 * particleIndex;
				const auto triangleStartIndex = triangles.size() * particleIndex;
				for (auto triangleIndex = 0; triangleIndex < triangles.size(); triangleIndex++) {
					projectedTriangles[triangleIndex + triangleStartIndex] = glm::uvec3(
						triangleIndex * 3 + vertexStartIndex,
						triangleIndex * 3 + 1 + vertexStartIndex,
						triangleIndex * 3 + 2 + vertexStartIndex);
					//Vertices of projected triangle.
					auto& pV0 = projectedVertices[triangleIndex * 3 + vertexStartIndex];
					auto& pV1 = projectedVertices[triangleIndex * 3 + 1 + vertexStartIndex];
					auto& pV2 = projectedVertices[triangleIndex * 3 + 2 + vertexStartIndex];

					const auto& v0 = vertices[triangles[triangleIndex].x];
					const auto& v1 = vertices[triangles[triangleIndex].y];
					const auto& v2 = vertices[triangles[triangleIndex].z];

					ProjectToPlane(v0, v1, v2, pV0, pV1, pV2, transform);
				}
			});

		const auto projectedMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto& projectedContent = newElement.m_content;
		projectedContent.m_mesh = projectedMesh;
		projectedContent.m_material = content.m_material;
		projectedContent.m_triangles = projectedTriangles;
		projectedMesh->SetVertices({ true, true, true, true }, projectedVertices, projectedTriangles);
	}
	for (const auto& element : cluster.m_elements)
	{
		const auto& content = element.m_content;
		const auto& transform = element.m_modelSpaceTransform.m_value * viewMatrix;
		projectedCluster.m_elements.emplace_back();
		auto& newElement = projectedCluster.m_elements.back();
		//newElement.m_modelSpaceTransform.m_value *= glm::inverse(viewMatrix);
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
				auto& pV0 = projectedVertices[triangleIndex * 3];
				auto& pV1 = projectedVertices[triangleIndex * 3 + 1];
				auto& pV2 = projectedVertices[triangleIndex * 3 + 2];

				const auto& v0 = vertices[triangles[triangleIndex].x];
				const auto& v1 = vertices[triangles[triangleIndex].y];
				const auto& v2 = vertices[triangles[triangleIndex].z];

				ProjectToPlane(v0, v1, v2, pV0, pV1, pV2, transform);
			});

		const auto projectedMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto& projectedContent = newElement.m_content;
		projectedContent.m_mesh = projectedMesh;
		projectedContent.m_material = content.m_material;
		projectedContent.m_triangles = projectedTriangles;
		projectedMesh->SetVertices({ true, true, true, true }, projectedVertices, projectedTriangles);
	}
	return projectedCluster;
}

BillboardCloud::RenderContent BillboardCloud::Join(const Cluster& cluster, const JoinSettings& settings)
{
	RenderContent retVal{};



	return retVal;
}
