#include "BillboardCloud.hpp"

using namespace EvoEngine;

void BillboardCloud::ProjectToPlane(const Vertex& v0, const Vertex& v1, const Vertex& v2,
	Vertex& pV0, Vertex& pV1, Vertex& pV2, const glm::mat4& transform)
{
	pV0 = v0;
	pV0.m_normal = glm::normalize(transform * glm::vec4(v0.m_normal, 0.f));
	pV0.m_tangent = glm::normalize(transform * glm::vec4(v0.m_tangent, 0.f));
	pV0.m_position = transform * glm::vec4(v0.m_position, 1.f);

	pV1 = v1;
	pV1.m_normal = glm::normalize(transform * glm::vec4(v1.m_normal, 0.f));
	pV1.m_tangent = glm::normalize(transform * glm::vec4(v1.m_tangent, 0.f));
	pV1.m_position = transform * glm::vec4(v1.m_position, 1.f);

	pV2 = v2;
	pV2.m_normal = glm::normalize(transform * glm::vec4(v2.m_normal, 0.f));
	pV2.m_tangent = glm::normalize(transform * glm::vec4(v2.m_tangent, 0.f));
	pV2.m_position = transform * glm::vec4(v2.m_position, 1.f);
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

template<typename T>
struct CPUFramebuffer
{
	std::vector<float> m_depthBuffer;
	std::vector<T> m_colorBuffer;
	std::vector<std::mutex> m_pixelLocks;
	int m_width = 0;
	int m_height = 0;
	CPUFramebuffer(const size_t width, const size_t height)
	{
		m_depthBuffer = std::vector<float>(width * height);
		m_colorBuffer = std::vector<T>(width * height);
		std::fill(m_colorBuffer.begin(), m_colorBuffer.end(), T(0.f));
		std::fill(m_depthBuffer.begin(), m_depthBuffer.end(), -FLT_MAX);
		m_pixelLocks = std::vector<std::mutex>(width * height);
		m_width = width;
		m_height = height;
	}

	[[nodiscard]] bool CompareZ(const int u, const int v, const float z) const
	{
		if (u < 0 || v < 0 || u > m_width - 1 || v > m_height - 1)
			return false;

		const int uv = u + m_width * v;
		return z >= m_depthBuffer[uv];
	}

	void SetPixel(const int u, const int v, const float z, const T& color)
	{
		if (u < 0 || v < 0 || u > m_width - 1 || v > m_height - 1)
			return;

		const int uv = u + m_width * v;
		std::lock_guard lock(m_pixelLocks[uv]);
		if (z < m_depthBuffer[uv])
			return;
		m_depthBuffer[uv] = z;
		m_colorBuffer[uv] = color;
	}
};



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

glm::vec2 BillboardCloud::Rectangle::Transform(const glm::vec2& target) const
{
	glm::vec2 retVal = target;
	//Recenter
	retVal -= m_center;
	const float x = glm::dot(retVal, m_xAxis);
	const float y = glm::dot(retVal, m_yAxis);
	retVal = glm::vec2(x, y) + glm::vec2(m_width, m_height) * .5f;
	return retVal;
}

glm::vec3 BillboardCloud::Rectangle::Transform(const glm::vec3& target) const
{
	glm::vec2 retVal = target;
	//Recenter
	retVal -= m_center;
	const float x = glm::dot(retVal, m_xAxis);
	const float y = glm::dot(retVal, m_yAxis);
	retVal = glm::vec2(x, y) + glm::vec2(m_width, m_height) * .5f;
	return { retVal, target.z };
}

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

inline
float EdgeFunction(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
	return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
}

void Barycentric3D(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, float& c0, float& c1, float& c2)
{
	const auto& v0 = b - a, v1 = c - a, v2 = p - a;
	const float d00 = glm::dot(v0, v0);
	const float d01 = glm::dot(v0, v1);
	const float d11 = glm::dot(v1, v1);
	const float d20 = glm::dot(v2, v0);
	const float d21 = glm::dot(v2, v1);
	const float den = d00 * d11 - d01 * d01;
	c1 = (d11 * d20 - d01 * d21) / den;
	c2 = (d00 * d21 - d01 * d20) / den;
	c0 = 1.0f - c1 - c2;
}

inline void Barycentric2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c, float& c0, float& c1, float& c2)
{
	const auto v0 = b - a, v1 = c - a, v2 = p - a;
	const float den = v0.x * v1.y - v1.x * v0.y;
	c1 = (v2.x * v1.y - v1.x * v2.y) / den;
	c2 = (v0.x * v2.y - v2.x * v0.y) / den;
	c0 = 1.0f - c1 - c2;
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

	//Calculate bounding triangle.
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


	//Rasterization
	{
		//Calculate texture size
		size_t textureWidth, textureHeight;
		const auto& boundingRectangle = projectedCluster.m_billboardRectangle;
		textureWidth = static_cast<size_t>(boundingRectangle.m_width * settings.m_textureSizeFactor);
		textureHeight = static_cast<size_t>(boundingRectangle.m_height * settings.m_textureSizeFactor);

		CPUFramebuffer<glm::vec4> albedoFrameBuffer(textureWidth, textureHeight);
		CPUFramebuffer<glm::vec3> normalFrameBuffer(textureWidth, textureHeight);
		CPUFramebuffer<float> roughnessFrameBuffer(textureWidth, textureHeight);
		CPUFramebuffer<float> metallicFrameBuffer(textureWidth, textureHeight);
		CPUFramebuffer<float> aoFrameBuffer(textureWidth, textureHeight);

		for (const auto& projectedElement : projectedCluster.m_elements)
		{
			const auto material = projectedElement.m_content.m_material;
			const auto albedoTexture = material->GetAlbedoTexture();
			std::vector<glm::vec4> albedoTextureData{};
			glm::ivec2 albedoTextureResolution;
			if (settings.m_transferAlbedoMap && albedoTexture)
			{
				albedoTexture->GetRgbaChannelData(albedoTextureData);
				albedoTextureResolution = albedoTexture->GetResolution();
			}
			const auto normalTexture = material->GetNormalTexture();
			std::vector<glm::vec3> normalTextureData{};
			glm::ivec2 normalTextureResolution;
			if (settings.m_transferNormalMap && normalTexture)
			{
				normalTexture->GetRgbChannelData(normalTextureData);
				normalTextureResolution = normalTexture->GetResolution();
			}
			const auto roughnessTexture = material->GetRoughnessTexture();
			std::vector<float> roughnessTextureData{};
			glm::ivec2 roughnessTextureResolution{};
			if (settings.m_transferRoughnessMap && roughnessTexture)
			{
				roughnessTexture->GetRedChannelData(roughnessTextureData);
				roughnessTextureResolution = roughnessTexture->GetResolution();
			}
			const auto metallicTexture = material->GetMetallicTexture();
			std::vector<float> metallicTextureData{};
			glm::ivec2 metallicTextureResolution;
			if (settings.m_transferMetallicMap && metallicTexture)
			{
				metallicTexture->GetRedChannelData(metallicTextureData);
				metallicTextureResolution = metallicTexture->GetResolution();
			}
			const auto aoTexture = material->GetAoTexture();
			std::vector<float> aoTextureData{};
			glm::ivec2 aoTextureResolution{};
			if (settings.m_transferAoMap && aoTexture)
			{
				aoTexture->GetRedChannelData(aoTextureData);
				aoTextureResolution = aoTexture->GetResolution();
			}
			//Adopted from https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/perspective-correct-interpolation-vertex-attributes.html
			const auto& vertices = projectedElement.m_content.m_mesh->UnsafeGetVertices();
			const auto& triangles = projectedElement.m_content.m_mesh->UnsafeGetTriangles();
			Jobs::RunParallelFor(triangles.size(), [&](const unsigned triangleIndex)
				{
					const auto& v0 = vertices[triangles[triangleIndex].x];
					const auto& v1 = vertices[triangles[triangleIndex].y];
					const auto& v2 = vertices[triangles[triangleIndex].z];

					glm::vec3 textureSpaceVertices[3];
					textureSpaceVertices[0] = boundingRectangle.Transform(v0.m_position) * settings.m_textureSizeFactor;
					textureSpaceVertices[1] = boundingRectangle.Transform(v1.m_position) * settings.m_textureSizeFactor;
					textureSpaceVertices[2] = boundingRectangle.Transform(v2.m_position) * settings.m_textureSizeFactor;

					//Bound check;
					auto minBound = glm::vec2(FLT_MAX, FLT_MAX);
					auto maxBound = glm::vec2(-FLT_MAX, -FLT_MAX);
					for (const auto& textureSpaceVertex : textureSpaceVertices)
					{
						minBound = glm::min(glm::vec2(textureSpaceVertex), minBound);
						maxBound = glm::max(glm::vec2(textureSpaceVertex), maxBound);
					}

					const auto left = static_cast<int>(minBound.x + 0.5f);
					const auto right = static_cast<int>(maxBound.x - 0.5f);
					const auto top = static_cast<int>(minBound.y + 0.5f);
					const auto bottom = static_cast<int>(maxBound.y - 0.5f);
					for (auto u = left; u <= right; u++)
					{
						for (auto v = top; v <= bottom; v++)
						{
							const auto p = glm::vec3(u + .5f, v + .5f, 0.f);
							float bc0, bc1, bc2;
							Barycentric2D(p, textureSpaceVertices[0], textureSpaceVertices[1], textureSpaceVertices[2], bc0, bc1, bc2);
							if (bc0 < 0.f || bc1 < 0.f || bc2 < 0.f) continue;
							float z = bc0 * textureSpaceVertices[0].z + bc1 * textureSpaceVertices[1].z + bc2 * textureSpaceVertices[2].z;
							//Early depth check.
							if (!albedoFrameBuffer.CompareZ(u, v, z)) continue;

							const auto texCoords = bc0 * v0.m_texCoord + bc1 * v1.m_texCoord + bc2 * v2.m_texCoord;
							auto albedo = glm::vec4(material->m_materialProperties.m_albedoColor, 1.f);
							float roughness = material->m_materialProperties.m_roughness;
							float metallic = material->m_materialProperties.m_metallic;
							float ao = 1.f;
							if (!albedoTextureData.empty())
							{
								int textureX = static_cast<int>(albedoTextureResolution.x * texCoords.x) % albedoTextureResolution.x;
								int textureY = static_cast<int>(albedoTextureResolution.y * texCoords.y) % albedoTextureResolution.y;
								if (textureX < 0) textureX += albedoTextureResolution.x;
								if (textureY < 0) textureY += albedoTextureResolution.y;

								const auto index = textureY * albedoTextureResolution.x + textureX;
								albedo = albedoTextureData[index];
							}
							//Alpha discard
							if(albedo.a < 0.1f) continue;
							auto normal = glm::normalize(bc0 * v0.m_normal + bc1 * v1.m_normal + bc2 * v2.m_normal);
							
							if (!normalTextureData.empty())
							{
								auto tangent = glm::normalize(bc0 * v0.m_tangent + bc1 * v1.m_tangent + bc2 * v2.m_tangent);
								const auto biTangent = glm::cross(normal, tangent);
								const auto tbn = glm::mat3(tangent, biTangent, normal);

								int textureX = static_cast<int>(normalTextureResolution.x * texCoords.x) % normalTextureResolution.x;
								int textureY = static_cast<int>(normalTextureResolution.y * texCoords.y) % normalTextureResolution.y;
								if (textureX < 0) textureX += normalTextureResolution.x;
								if (textureY < 0) textureY += normalTextureResolution.y;

								const auto index = textureY * normalTextureResolution.x + textureX;
								const auto sampled = glm::normalize(normalTextureData[index]) * 2.0f - glm::vec3(1.0f);
								normal = glm::normalize(tbn * sampled);
							}
							if(glm::dot(normal, glm::vec3(0, 0, 1)) < 0) normal = -normal;

							if (!roughnessTextureData.empty())
							{
								int textureX = static_cast<int>(roughnessTextureResolution.x * texCoords.x) % roughnessTextureResolution.x;
								int textureY = static_cast<int>(roughnessTextureResolution.y * texCoords.y) % roughnessTextureResolution.y;
								if (textureX < 0) textureX += roughnessTextureResolution.x;
								if (textureY < 0) textureY += roughnessTextureResolution.y;


								const auto index = textureY * roughnessTextureResolution.x + textureX;
								roughness = roughnessTextureData[index];

							}
							if (!metallicTextureData.empty())
							{
								int textureX = static_cast<int>(metallicTextureResolution.x * texCoords.x) % metallicTextureResolution.x;
								int textureY = static_cast<int>(metallicTextureResolution.y * texCoords.y) % metallicTextureResolution.y;
								if (textureX < 0) textureX += metallicTextureResolution.x;
								if (textureY < 0) textureY += metallicTextureResolution.y;

								const auto index = textureY * metallicTextureResolution.x + textureX;
								metallic = metallicTextureData[index];

							}
							if (!aoTextureData.empty())
							{
								int textureX = static_cast<int>(aoTextureResolution.x * texCoords.x) % aoTextureResolution.x;
								int textureY = static_cast<int>(aoTextureResolution.y * texCoords.y) % aoTextureResolution.y;
								if (textureX < 0) textureX += aoTextureResolution.x;
								if (textureY < 0) textureY += aoTextureResolution.y;


								const auto index = textureY * aoTextureResolution.x + textureX;
								ao = aoTextureData[index];

							}
							albedoFrameBuffer.SetPixel(u, v, z, albedo);
							normal = normal * 0.5f + glm::vec3(0.5f);
							normalFrameBuffer.SetPixel(u, v, z, normal);
							roughnessFrameBuffer.SetPixel(u, v, z, roughness);
							metallicFrameBuffer.SetPixel(u, v, z, metallic);
							aoFrameBuffer.SetPixel(u, v, z, ao);
						}
					}
				});
		}

		std::shared_ptr<Texture2D> albedoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		albedoTexture->SetRgbaChannelData(albedoFrameBuffer.m_colorBuffer, glm::uvec2(albedoFrameBuffer.m_width, albedoFrameBuffer.m_height));
		std::shared_ptr<Texture2D> normalTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		normalTexture->SetRgbChannelData(normalFrameBuffer.m_colorBuffer, glm::uvec2(normalFrameBuffer.m_width, normalFrameBuffer.m_height));
		std::shared_ptr<Texture2D> roughnessTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		roughnessTexture->SetRedChannelData(roughnessFrameBuffer.m_colorBuffer, glm::uvec2(roughnessFrameBuffer.m_width, roughnessFrameBuffer.m_height));
		std::shared_ptr<Texture2D> metallicTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		metallicTexture->SetRedChannelData(metallicFrameBuffer.m_colorBuffer, glm::uvec2(metallicFrameBuffer.m_width, metallicFrameBuffer.m_height));
		std::shared_ptr<Texture2D> aoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		aoTexture->SetRedChannelData(aoFrameBuffer.m_colorBuffer, glm::uvec2(aoFrameBuffer.m_width, aoFrameBuffer.m_height));
		projectedCluster.m_billboardMaterial = ProjectManager::CreateTemporaryAsset<Material>();
		projectedCluster.m_billboardMaterial->SetAlbedoTexture(albedoTexture);
		projectedCluster.m_billboardMaterial->SetNormalTexture(normalTexture);
		projectedCluster.m_billboardMaterial->SetRoughnessTexture(roughnessTexture);
		projectedCluster.m_billboardMaterial->SetMetallicTexture(metallicTexture);
		projectedCluster.m_billboardMaterial->SetAOTexture(aoTexture);

		projectedCluster.m_billboardMesh = ProjectManager::CreateTemporaryAsset<Mesh>();

		std::vector<Vertex> planeVertices;
		planeVertices.resize(4);
		planeVertices[0].m_position = glm::vec3(projectedCluster.m_billboardRectangle.m_points[0].x, projectedCluster.m_billboardRectangle.m_points[0].y, 0.f);
		planeVertices[0].m_texCoord = glm::vec2(0, 0);
		planeVertices[1].m_position = glm::vec3(projectedCluster.m_billboardRectangle.m_points[1].x, projectedCluster.m_billboardRectangle.m_points[1].y, 0.f);
		planeVertices[1].m_texCoord = glm::vec2(1, 0);
		planeVertices[2].m_position = glm::vec3(projectedCluster.m_billboardRectangle.m_points[2].x, projectedCluster.m_billboardRectangle.m_points[2].y, 0.f);
		planeVertices[2].m_texCoord = glm::vec2(1, 1);
		planeVertices[3].m_position = glm::vec3(projectedCluster.m_billboardRectangle.m_points[3].x, projectedCluster.m_billboardRectangle.m_points[3].y, 0.f);
		planeVertices[3].m_texCoord = glm::vec2(0, 1);
		std::vector<glm::uvec3> planeTriangles;
		planeTriangles.resize(2);
		planeTriangles[0] = { 0, 1, 2 };
		planeTriangles[1] = { 2, 3, 0 };
		projectedCluster.m_billboardMesh->SetVertices({}, planeVertices, planeTriangles);

	}
	return projectedCluster;
}

BillboardCloud::RenderContent BillboardCloud::Join(const Cluster& cluster, const JoinSettings& settings)
{
	RenderContent retVal{};



	return retVal;
}