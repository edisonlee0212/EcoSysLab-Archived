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
	//pV0.m_position.z = 0.f;
	pV1 = v1;
	pV1.m_normal = transform * glm::vec4(v1.m_normal, 0.f);
	pV1.m_tangent = transform * glm::vec4(v1.m_tangent, 0.f);
	pV1.m_position = p1;
	//pV1.m_position.z = 0.f;
	pV2 = v2;
	pV2.m_normal = transform * glm::vec4(v2.m_normal, 0.f);
	pV2.m_tangent = transform * glm::vec4(v2.m_tangent, 0.f);
	pV2.m_position = p2;
	//pV2.m_position.z = 0.f;
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
		projectedCluster.m_billboardRectangle = RotatingCalipers::GetMinAreaRectangle(std::move(points));
	}
	projectedCluster.m_billboardRectangle.Update();

	return projectedCluster;
}

BillboardCloud::RenderContent BillboardCloud::Join(const Cluster& cluster, const JoinSettings& settings)
{
	RenderContent retVal{};



	return retVal;
}

inline double Cross(const glm::vec2& origin, const glm::vec2& a, const glm::vec2& b)
{
	return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

struct PointComparator
{
	bool operator()(const glm::vec2& a, const glm::vec2& b) const
	{
		return a.x < b.x || (a.x == b.x && a.y < b.y);
	}
};

std::vector<glm::vec2> BillboardCloud::RotatingCalipers::ConvexHull(std::vector<glm::vec2> points)
{
	const size_t pointSize = points.size();
	size_t k = 0;
	if (pointSize <= 3) return points;

	std::vector<glm::vec2> retVal(2 * pointSize);
	std::sort(points.begin(), points.end(), PointComparator());

	for (size_t i = 0; i < pointSize; ++i)
	{
		while (k >= 2 && Cross(retVal[k - 2], retVal[k - 1], points[i]) <= 0) k--;
		retVal[k++] = points[i];
	}

	for (size_t i = pointSize - 1, t = k + 1; i > 0; --i)
	{
		while (k >= t && Cross(retVal[k - 2], retVal[k - 1], points[i - 1]) <= 0) k--;
		retVal[k++] = points[i - 1];
	}
	retVal.resize(k - 1);
	return retVal;
}

struct MinAreaState
{
	size_t m_bottom;
	size_t m_left;
	float m_height;
	float m_width;
	float m_baseA;
	float m_baseB;
	float m_area;
};

BillboardCloud::Rectangle BillboardCloud::RotatingCalipers::GetMinAreaRectangle(std::vector<glm::vec2> points)
{
	auto convexHull = ConvexHull(std::move(points));
	float minArea = FLT_MAX;
	size_t left = 0, bottom = 0, right = 0, top = 0;

	/* rotating calipers sides will always have coordinates
	 (a,b) (-b,a) (-a,-b) (b, -a)
	 */
	 /* this is a first base vector (a,b) initialized by (1,0) */

	glm::vec2 pt0 = convexHull[0];
	float leftX = pt0.x;
	float rightX = pt0.x;
	float topY = pt0.y;
	float bottomY = pt0.y;

	size_t n = convexHull.size();

	std::vector<glm::vec2> list(n);
	std::vector<float> lengths(n);

	for (size_t i = 0; i < n; i++)
	{
		if (pt0.x < leftX)
		{
			leftX = pt0.x;
			left = i;
		}
		if (pt0.x > rightX)
		{
			rightX = pt0.x;
			right = i;
		}
		if (pt0.y > topY)
		{
			topY = pt0.y;
			top = i;
		}
		if (pt0.y < bottomY)
		{
			bottomY = pt0.y;
			bottom = i;
		}

		glm::vec2 pt = convexHull[(i + 1) & (i + 1 < n ? -1 : 0)];
		float dx = pt.x - pt0.x;
		float dy = pt.y - pt0.y;

		list[i].x = dx;
		list[i].y = dy;

		lengths[i] = 1.f / sqrt(dx * dx + dy * dy);
		pt0 = pt;
	}

	// find convex hull orientation
	float ax = list[n - 1].x;
	float ay = list[n - 1].y;
	float orientation = 0, baseA = 0, baseB = 0;

	for (size_t i = 0; i < n; i++)
	{
		float bx = list[i].x;
		float by = list[i].y;
		if (float convexity = ax * by - ay * bx; convexity != 0.f)
		{
			orientation = convexity > 0 ? 1.0 : -1.0;
			break;
		}
		ax = bx;
		ay = by;
	}

	baseA = orientation;

	/*****************************************************************************************/
	/*                         init calipers position                                        */
	size_t seq[4];
	seq[0] = bottom;
	seq[1] = right;
	seq[2] = top;
	seq[3] = left;

	/*****************************************************************************************/
	/*                         Main loop - evaluate angles and rotate calipers               */

	MinAreaState minAreaState;

	/* all the edges will be checked while rotating calipers by 90 degrees */
	for (size_t k = 0; k < n; k++)
	{
		/* sinus of minimal angle */
		/*float sinus;*/

		/* compute cosine of angle between calipers side and polygon edge */
		/* dp - dot product */
		float dp0 = baseA * list[seq[0]].x + baseB * list[seq[0]].y;
		float dp1 = -baseB * list[seq[1]].x + baseA * list[seq[1]].y;
		float dp2 = -baseA * list[seq[2]].x - baseB * list[seq[2]].y;
		float dp3 = baseB * list[seq[3]].x - baseA * list[seq[3]].y;

		float cosAlpha = dp0 * lengths[seq[0]];
		float maxCos = cosAlpha;
		/* number of calipers edges, that has minimal angle with edge */
		int mainElement = 0;

		/* choose minimal angle */
		cosAlpha = dp1 * lengths[seq[1]];
		maxCos = cosAlpha > maxCos ? (mainElement = 1, cosAlpha) : maxCos;
		cosAlpha = dp2 * lengths[seq[2]];
		maxCos = cosAlpha > maxCos ? (mainElement = 2, cosAlpha) : maxCos;
		cosAlpha = dp3 * lengths[seq[3]];
		maxCos = cosAlpha > maxCos ? (mainElement = 3, cosAlpha) : maxCos;

		/*rotate calipers*/
		//get next base
		size_t tempPoint = seq[mainElement];
		float leadX = list[tempPoint].x * lengths[tempPoint];
		float leadY = list[tempPoint].y * lengths[tempPoint];
		switch (mainElement)
		{
		case 0:
			baseA = leadX;
			baseB = leadY;
			break;
		case 1:
			baseA = leadY;
			baseB = -leadX;
			break;
		case 2:
			baseA = -leadX;
			baseB = -leadY;
			break;
		case 3:
			baseA = -leadY;
			baseB = leadX;
			break;
		}

		/* change base point of main edge */
		seq[mainElement] += 1;
		seq[mainElement] = seq[mainElement] == n ? 0 : seq[mainElement];

		float dx = convexHull[seq[1]].x - convexHull[seq[3]].x;
		float dy = convexHull[seq[1]].y - convexHull[seq[3]].y;
		float width = dx * baseA + dy * baseB;
		dx = convexHull[seq[2]].x - convexHull[seq[0]].x;
		dy = convexHull[seq[2]].y - convexHull[seq[0]].y;
		float height = -dx * baseB + dy * baseA;
		if (float area = width * height; area <= minArea)
		{
			minArea = area;
			minAreaState.m_baseA = baseA;
			minAreaState.m_baseB = baseB;
			minAreaState.m_width = width;
			minAreaState.m_height = height;
			minAreaState.m_left = seq[3];
			minAreaState.m_bottom = seq[0];
			minAreaState.m_area = area;
		}
	}


	float a1 = minAreaState.m_baseA;
	float b1 = minAreaState.m_baseB;

	float a2 = -minAreaState.m_baseB;
	float b2 = minAreaState.m_baseA;

	float c1 = a1 * convexHull[minAreaState.m_left].x + convexHull[minAreaState.m_left].y * b1;
	float c2 = a2 * convexHull[minAreaState.m_bottom].x + convexHull[minAreaState.m_bottom].y * b2;

	float id = 1.f / (a1 * b2 - a2 * b1);

	float px = (c1 * b2 - c2 * b1) * id;
	float py = (a1 * c2 - a2 * c1) * id;

	glm::vec2 out0(px, py);
	glm::vec2 out1(a1 * minAreaState.m_width, b1 * minAreaState.m_width);
	glm::vec2 out2(a2 * minAreaState.m_height, b2 * minAreaState.m_height);

	Rectangle retVal;

	retVal.m_points[0] = out0;
	retVal.m_points[1] = out0 + out1;
	retVal.m_points[2] = out0 + out1 + out2;
	retVal.m_points[3] = out0 + out2;

	retVal.Update();

	return retVal;
}
