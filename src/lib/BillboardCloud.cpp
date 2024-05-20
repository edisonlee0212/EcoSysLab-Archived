#include "BillboardCloud.hpp"

using namespace EvoEngine;
#pragma region Rotating Calipers
void Swap(std::vector<glm::vec2>& points, const int a, const int b)
{
	const auto temp = points[a];
	points[a] = points[b];
	points[b] = temp;
}

float GetDistance(const glm::vec2& p1, const glm::vec2& p2)
{
	return (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
}

glm::vec2 GetDiff(const glm::vec2& p1, const glm::vec2& p2)
{
	glm::vec2 temp;
	temp.x = p1.x - p2.x;
	temp.y = p1.y - p2.y;

	return temp;
}

float GetCross(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2)
{
	return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

float GetDot(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2)
{
	return (p1.x - p0.x) * (p2.x - p0.x) + (p1.y - p0.y) * (p2.y - p0.y);
}

float CompareAngle(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2)
{
	const float cross = GetCross(p0, p1, p2);
	if (cross == 0.f) {
		return GetDistance(p0, p2) - GetDistance(p0, p1);
	}
	return cross;
}

void SortPoints(std::vector<glm::vec2>& points, const int left, const int right)
{
	if (left >= right) {
		return;
	}

	const int mid = (left + right) >> 1;
	Swap(points, left, mid);
	int last = left;
	for (int i = left + 1; i <= right; i++) {
		if (CompareAngle(points[0], points[i], points[left]) > 0) {
			Swap(points, i, ++last);
		}
	}
	Swap(points, left, last);
	SortPoints(points, left, last - 1);
	SortPoints(points, last + 1, right);
}

void GetConvex(std::vector<glm::vec2>& points, int& n)
{
	const int verticesSize = points.size();
	if (verticesSize < 4) {
		n = verticesSize;
		return;
	}
	// base point
	int base = 0;
	for (int i = 0; i < verticesSize; i++) {
		if (points[i].y == points[base].y && points[i].x < points[base].x) {
			base = i;
		}
		else if (points[i].y < points[base].y) {
			base = i;
		}
	}
	Swap(points, base, 0);

	// sort
	SortPoints(points, 1, verticesSize - 1);

	// calculate convex hull
	int top = 1;
	for (int i = 2; i < verticesSize; i++) {
		while (top > 0 && GetCross(points[top - 1], points[top], points[i]) <= 0) {
			top--;
		}
		points[++top] = points[i];
	}
	n = top;
}
void BillboardCloud::RotatingCalipers(std::vector<glm::vec2>& points, Rectangle& rectangle)
{
	int top, downLast = 0, rightLast = 0, upLast = 0, leftLast = 0;
	const auto pointSize = points.size();
	GetConvex(points, top);
	if (pointSize < 4) {
		points[top] = points[0];
	}
	else {
		points[++top] = points[0];
	}
	assert(pointSize > 2);

	int up = 0;
	float area = FLT_MAX;
	int left = 0;
	int right = 1;
	for (int down = 0; down < top; down++) {
		// find right
		while (GetDot(points[down], points[down + 1], points[right]) <= GetDot(points[down], points[down + 1], points[right + 1])) {
			right = (right + 1) % top;
		}

		// find up
		if (down == 0) {
			up = right;
		}
		while (GetCross(points[down], points[down + 1], points[up]) <= GetCross(points[down], points[down + 1], points[up + 1])) {
			up = (up + 1) % top;
		}

		// find down
		if (down == 0) {
			left = up;
		}
		while (GetDot(points[down], points[down + 1], points[left]) >= GetDot(points[down], points[down + 1], points[left + 1])) {
			left = (left + 1) % top;
		}

		const float distance = GetDistance(points[down], points[down + 1]);
		const float x = GetCross(points[down], points[down + 1], points[up]) / distance;
		glm::vec2 temp;
		temp.x = points[right].x + points[down].x - points[left].x;
		temp.y = points[right].y + points[down].y - points[left].y;

		if (const float y = GetDot(points[down], points[down + 1], temp); area > x * y) {
			area = x * y;
			downLast = down;
			rightLast = right;
			upLast = up;
			leftLast = left;
		}
	}

	// Calculate outer rectangle
	if (points[downLast + 1].y == points[downLast].y) {
		rectangle.m_points[0].x = points[leftLast].x;
		rectangle.m_points[0].y = points[downLast].y;

		rectangle.m_points[1].x = points[rightLast].x;
		rectangle.m_points[1].y = points[downLast].y;

		rectangle.m_points[2].x = points[rightLast].x;
		rectangle.m_points[2].y = points[upLast].y;

		rectangle.m_points[3].x = points[leftLast].x;
		rectangle.m_points[3].y = points[upLast].y;

	}
	else if (points[downLast + 1].x == points[downLast].x) {
		rectangle.m_points[0].x = points[downLast].x;
		rectangle.m_points[0].y = points[leftLast].y;

		rectangle.m_points[1].x = points[downLast].x;
		rectangle.m_points[1].y = points[rightLast].y;

		rectangle.m_points[2].x = points[upLast].x;
		rectangle.m_points[2].y = points[rightLast].y;

		rectangle.m_points[3].x = points[upLast].x;
		rectangle.m_points[3].y = points[leftLast].y;

	}
	else {
		const float k = (points[downLast + 1].y - points[downLast].y) / (points[downLast + 1].x - points[downLast].x);

		rectangle.m_points[0].x = (k * points[leftLast].y + points[leftLast].x - k * points[downLast].y + k * k * points[downLast].x) / (k * k + 1.f);
		rectangle.m_points[0].y = k * rectangle.m_points[0].x + points[downLast].y - k * points[downLast].x;

		rectangle.m_points[1].x = (k * points[rightLast].y + points[rightLast].x - k * points[downLast].y + k * k * points[downLast].x) / (k * k + 1.f);
		rectangle.m_points[1].y = k * rectangle.m_points[1].x + points[downLast].y - k * points[downLast].x;

		rectangle.m_points[2].x = (k * points[rightLast].y + points[rightLast].x - k * points[upLast].y + k * k * points[upLast].x) / (k * k + 1.f);
		rectangle.m_points[2].y = k * rectangle.m_points[2].x + points[upLast].y - k * points[upLast].x;

		rectangle.m_points[3].x = (k * points[leftLast].y + points[leftLast].x - k * points[upLast].y + k * k * points[upLast].x) / (k * k + 1.f);
		rectangle.m_points[3].y = k * rectangle.m_points[3].x + points[upLast].y - k * points[upLast].x;
	}
}

#pragma endregion

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
	pV0.m_position.z /= 10.f;
	pV1 = v1;
	pV1.m_normal = transform * glm::vec4(v1.m_normal, 0.f);
	pV1.m_tangent = transform * glm::vec4(v1.m_tangent, 0.f);
	pV1.m_position = p1;
	pV1.m_position.z /= 10.f;
	pV2 = v2;
	pV2.m_normal = transform * glm::vec4(v2.m_normal, 0.f);
	pV2.m_tangent = transform * glm::vec4(v2.m_tangent, 0.f);
	pV2.m_position = p2;
	pV2.m_position.z /= 10.f;
}

void BillboardCloud::Rectangle::Update()
{
	m_center = (m_points[0] + m_points[2]) * .5f;
	const auto vX = m_points[1] - m_points[0];
	const auto vY = m_points[2] - m_points[1];
	m_xAxis = glm::normalize(vX);
	m_yAxis = glm::normalize(vY);

	m_width = glm::length(vX);
	m_height = glm::length(vY);
}

BillboardCloud::ProjectedCluster BillboardCloud::Project(const Cluster& cluster, const ProjectSettings& settings)
{
	ProjectedCluster projectedCluster{};
	glm::vec3 frontAxis = cluster.m_planeNormal;
	glm::vec3 upAxis = cluster.m_planeYAxis;
	glm::vec3 leftAxis = glm::normalize(glm::cross(frontAxis, upAxis));
	upAxis = glm::normalize(glm::cross(leftAxis, frontAxis));
	//glm::mat4 viewMatrix = glm::lookAt(cluster.m_clusterCenter, cluster.m_clusterCenter + cluster.m_planeNormal, cluster.m_planeYAxis);
	glm::mat4 rotateMatrix = glm::transpose(glm::mat4(glm::vec4(leftAxis, 0.0f), glm::vec4(upAxis, 0.0f), glm::vec4(frontAxis, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
	//glm::mat4 translateMatrix = glm::translate(-cluster.m_clusterCenter);
	//const auto viewMatrix = translateMatrix * rotateMatrix;
	for (const auto& instancedElement : cluster.m_instancedElements)
	{
		const auto& content = instancedElement.m_content;
		const auto& modelSpaceTransform = instancedElement.m_modelSpaceTransform.m_value * rotateMatrix;
		projectedCluster.m_elements.emplace_back();
		auto& newElement = projectedCluster.m_elements.back();
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
		projectedMesh->SetVertices({ true, true, true, true }, projectedVertices, projectedTriangles);
	}
	for (const auto& element : cluster.m_elements)
	{
		const auto& content = element.m_content;
		const auto& transform = element.m_modelSpaceTransform.m_value * rotateMatrix;
		projectedCluster.m_elements.emplace_back();
		auto& newElement = projectedCluster.m_elements.back();
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
		projectedMesh->SetVertices({ true, true, true, true }, projectedVertices, projectedTriangles);
	}

	std::vector<glm::vec2> points;
	for (const auto& projectedElement : projectedCluster.m_elements)
	{
		const auto startIndex = points.size();
		const auto& vertices = projectedElement.m_content.m_mesh->UnsafeGetVertices();
		const auto newPointsSize = vertices.size();
		points.resize(startIndex + newPointsSize);
		Jobs::RunParallelFor(newPointsSize, [&](unsigned pointIndex)
			{
				const auto& position = vertices.at(pointIndex).m_position;
				points[startIndex + pointIndex] = glm::vec2(position.x, position.y);
			});
	}
	assert(points.size() > 2);
	if (points.size() == 3)
	{
		const auto& p0 = points[0];
		const auto& p1 = points[1];
		const auto& p2 = points[2];

		const auto e0 = glm::distance(p0, p1);
		const auto e1 = glm::distance(p1, p2);
		const auto e2 = glm::distance(p2, p0);
		glm::vec2 longestEdgeStart, longestEdgeEnd, otherPoint;
		if (e0 >= e1 && e0 >= e2) {
			longestEdgeStart = p0;
			longestEdgeEnd = p1;
			otherPoint = p2;

		}
		else if (e1 >= e0 && e1 >= e2) {
			longestEdgeStart = p1;
			longestEdgeEnd = p2;
			otherPoint = p0;
		}
		else {
			longestEdgeStart = p2;
			longestEdgeEnd = p0;
			otherPoint = p1;
		}
		float length = glm::length(longestEdgeEnd - longestEdgeStart);
		glm::vec2 lengthVector = glm::normalize(longestEdgeEnd - longestEdgeStart);
		float projectedDistance = glm::dot(otherPoint - longestEdgeStart, lengthVector);
		glm::vec2 projectedPoint = longestEdgeStart + projectedDistance * lengthVector;
		float width = glm::distance(otherPoint, projectedPoint);
		glm::vec2 widthVector = glm::normalize(otherPoint - projectedPoint);
		projectedCluster.m_billboardRectangle.m_points[0] = longestEdgeStart;
		projectedCluster.m_billboardRectangle.m_points[3] = longestEdgeStart + length * lengthVector;
		projectedCluster.m_billboardRectangle.m_points[1] = projectedCluster.m_billboardRectangle.m_points[0] + width * widthVector;
		projectedCluster.m_billboardRectangle.m_points[2] = projectedCluster.m_billboardRectangle.m_points[3] + width * widthVector;
	}
	else
	{
		RotatingCalipers(points, projectedCluster.m_billboardRectangle);
	}
	projectedCluster.m_billboardRectangle.Update();

	return projectedCluster;
}

BillboardCloud::RenderContent BillboardCloud::Join(const Cluster& cluster, const JoinSettings& settings)
{
	RenderContent retVal{};



	return retVal;
}
