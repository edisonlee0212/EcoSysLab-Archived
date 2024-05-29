#include "BillboardCloud.hpp"
#include "Prefab.hpp"
using namespace EvoEngine;
#pragma region Projection
void AppendTriangles(std::vector<BillboardCloud::ClusterTriangle>& triangles, const std::vector<BillboardCloud::Element>& elements)
{
	for (int elementIndex = 0; elementIndex < elements.size(); elementIndex++)
	{
		const auto& element = elements.at(elementIndex);
		for (int triangleIndex = 0; triangleIndex < element.m_triangles.size(); triangleIndex++)
		{
			BillboardCloud::ClusterTriangle clusterTriangle;
			clusterTriangle.m_elementIndex = elementIndex;
			clusterTriangle.m_triangleIndex = triangleIndex;
			triangles.emplace_back(clusterTriangle);
		}
	}
}

glm::vec3 BillboardCloud::ElementCollection::CalculateCentroid(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateCentroid(triangle.m_triangleIndex);
}


float BillboardCloud::ElementCollection::CalculateArea(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateArea(triangle.m_triangleIndex);
}

float BillboardCloud::ElementCollection::CalculateNormalDistance(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateNormalDistance(triangle.m_triangleIndex);
}

glm::vec3 BillboardCloud::ElementCollection::CalculateNormal(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateNormal(triangle.m_triangleIndex);
}
void BillboardCloud::ProjectClusters(const ProjectSettings& projectSettings)
{
	for (auto& elementCollection : m_elementCollections) elementCollection.Project(projectSettings);
}

inline void TransformVertex(const Vertex& v,
	Vertex& tV, const glm::mat4& transform)
{
	tV = v;
	tV.m_normal = glm::normalize(transform * glm::vec4(v.m_normal, 0.f));
	tV.m_tangent = glm::normalize(transform * glm::vec4(v.m_tangent, 0.f));
	tV.m_position = transform * glm::vec4(v.m_position, 1.f);
}

inline void TransformVertex(Vertex& v, const glm::mat4& transform)
{
	v.m_normal = glm::normalize(transform * glm::vec4(v.m_normal, 0.f));
	v.m_tangent = glm::normalize(transform * glm::vec4(v.m_tangent, 0.f));
	v.m_position = transform * glm::vec4(v.m_position, 1.f);
}

glm::vec3 BillboardCloud::Element::CalculateCentroid(const int triangleIndex) const
{
	return CalculateCentroid(m_triangles.at(triangleIndex));
}

float BillboardCloud::Element::CalculateArea(const int triangleIndex) const
{
	return CalculateArea(m_triangles.at(triangleIndex));
}

float BillboardCloud::Element::CalculateNormalDistance(const int triangleIndex) const
{
	const auto centroid = CalculateCentroid(triangleIndex);
	auto normal = CalculateNormal(triangleIndex);
	if (glm::dot(centroid, normal) < 0)
	{
		normal = -normal;
	}
	return glm::abs(glm::dot(centroid, normal));
}

glm::vec3 BillboardCloud::Element::CalculateNormal(const int triangleIndex) const
{
	return CalculateNormal(m_triangles.at(triangleIndex));
}

glm::vec3 BillboardCloud::Element::CalculateCentroid(const glm::uvec3& triangle) const
{
	const auto& a = m_vertices[triangle.x].m_position;
	const auto& b = m_vertices[triangle.y].m_position;
	const auto& c = m_vertices[triangle.z].m_position;

	return {
		(a.x + b.x + c.x) / 3,
			(a.y + b.y + c.y) / 3,
			(a.z + b.z + c.z) / 3
	};
}

float BillboardCloud::Element::CalculateArea(const glm::uvec3& triangle) const
{
	const auto& p0 = m_vertices[triangle.x].m_position;
	const auto& p1 = m_vertices[triangle.y].m_position;
	const auto& p2 = m_vertices[triangle.z].m_position;
	const float a = glm::length(p0 - p1);
	const float b = glm::length(p2 - p1);
	const float c = glm::length(p0 - p2);
	const float d = (a + b + c) / 2;
	return glm::sqrt(d * (d - a) * (d - b) * (d - c));
}

glm::vec3 BillboardCloud::Element::CalculateNormal(const glm::uvec3& triangle) const
{
	const auto& p0 = m_vertices[triangle.x].m_position;
	const auto& p1 = m_vertices[triangle.y].m_position;
	const auto& p2 = m_vertices[triangle.z].m_position;
	return glm::normalize(glm::cross(p0 - p1, p0 - p2));
}

float BillboardCloud::Element::CalculateNormalDistance(const glm::uvec3& triangle) const
{
	const auto centroid = CalculateCentroid(triangle);
	auto normal = CalculateNormal(triangle);
	if (glm::dot(centroid, normal) < 0)
	{
		normal = -normal;
	}
	return glm::abs(glm::dot(centroid, normal));
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

struct ProjectedTriangle
{
	Vertex m_projectedV0;
	Vertex m_projectedV1;
	Vertex m_projectedV2;
	Handle m_materialHandle;
};

void BillboardCloud::ElementCollection::Project(const ProjectSettings& projectSettings)
{
	std::unordered_map<Handle, PBRMaterial> pbrMaterials;
	for (auto& element : m_elements)
	{
		const auto& material = element.m_material;
		auto materialHandle = material->GetHandle();
		pbrMaterials[materialHandle].ApplyMaterial(material, projectSettings);
	}

	for (auto& cluster : m_clusters) Project(pbrMaterials, cluster, projectSettings);
}

void BillboardCloud::ElementCollection::PBRMaterial::ApplyMaterial(const std::shared_ptr<Material>& material,
	const ProjectSettings& projectSettings)
{
	m_baseAlbedo = glm::vec4(material->m_materialProperties.m_albedoColor, 1.f - material->m_materialProperties.m_transmission);
	m_baseRoughness = material->m_materialProperties.m_roughness;
	m_baseMetallic = material->m_materialProperties.m_metallic;
	m_baseAo = 1.f;

	const auto albedoTexture = material->GetAlbedoTexture();
	if (projectSettings.m_transferAlbedoMap && albedoTexture)
	{
		albedoTexture->GetRgbaChannelData(m_albedoTextureData);
		m_albedoTextureResolution = albedoTexture->GetResolution();
	}
	const auto normalTexture = material->GetNormalTexture();
	if (projectSettings.m_transferNormalMap && normalTexture)
	{
		normalTexture->GetRgbChannelData(m_normalTextureData);
		m_normalTextureResolution = normalTexture->GetResolution();
	}
	const auto roughnessTexture = material->GetRoughnessTexture();
	if (projectSettings.m_transferRoughnessMap && roughnessTexture)
	{
		roughnessTexture->GetRedChannelData(m_roughnessTextureData);
		m_roughnessTextureResolution = roughnessTexture->GetResolution();
	}
	const auto metallicTexture = material->GetMetallicTexture();
	if (projectSettings.m_transferMetallicMap && metallicTexture)
	{
		metallicTexture->GetRedChannelData(m_metallicTextureData);
		m_metallicTextureResolution = metallicTexture->GetResolution();
	}
	const auto aoTexture = material->GetAoTexture();
	if (projectSettings.m_transferAoMap && aoTexture)
	{
		aoTexture->GetRedChannelData(m_aoTextureData);
		m_aoTextureResolution = aoTexture->GetResolution();
	}
}

void BillboardCloud::ElementCollection::Project(std::unordered_map<Handle, PBRMaterial>& pbrMaterials, Cluster& cluster, const ProjectSettings& projectSettings) const
{
	const auto billboardFrontAxis = cluster.m_clusterPlane.GetNormal();
	auto billboardUpAxis = glm::vec3(billboardFrontAxis.y, billboardFrontAxis.z, billboardFrontAxis.x); //cluster.m_planeYAxis;
	const auto billboardLeftAxis = glm::normalize(glm::cross(billboardFrontAxis, billboardUpAxis));
	billboardUpAxis = glm::normalize(glm::cross(billboardLeftAxis, billboardFrontAxis));
	glm::mat4 rotateMatrix = glm::transpose(glm::mat4(glm::vec4(billboardLeftAxis, 0.0f), glm::vec4(billboardUpAxis, 0.0f), glm::vec4(billboardFrontAxis, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
	std::vector<ProjectedTriangle> projectedTriangles;
	projectedTriangles.resize(cluster.m_triangles.size());
	Jobs::RunParallelFor(cluster.m_triangles.size(), [&](const unsigned triangleIndex)
		{
			const auto& clusterTriangle = cluster.m_triangles.at(triangleIndex);
			auto& projectedTriangle = projectedTriangles[triangleIndex];
			const auto& element = m_elements.at(clusterTriangle.m_elementIndex);
			const auto& vertices = element.m_vertices;
			const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
			auto& v0 = vertices.at(triangle.x);
			auto& v1 = vertices.at(triangle.y);
			auto& v2 = vertices.at(triangle.z);

			auto& pV0 = projectedTriangle.m_projectedV0;
			auto& pV1 = projectedTriangle.m_projectedV1;
			auto& pV2 = projectedTriangle.m_projectedV2;

			TransformVertex(v0, pV0, rotateMatrix);
			TransformVertex(v1, pV1, rotateMatrix);
			TransformVertex(v2, pV2, rotateMatrix);

			projectedTriangle.m_materialHandle = element.m_material->GetHandle();
		});

	std::vector<glm::vec2> points;
	points.resize(projectedTriangles.size() * 3);
	Jobs::RunParallelFor(projectedTriangles.size(), [&](unsigned triangleIndex)
		{
			const auto& projectedTriangle = projectedTriangles.at(triangleIndex);
			points.at(triangleIndex * 3) = glm::vec2(projectedTriangle.m_projectedV0.m_position.x, projectedTriangle.m_projectedV0.m_position.y);
			points.at(triangleIndex * 3 + 1) = glm::vec2(projectedTriangle.m_projectedV1.m_position.x, projectedTriangle.m_projectedV1.m_position.y);
			points.at(triangleIndex * 3 + 2) = glm::vec2(projectedTriangle.m_projectedV2.m_position.x, projectedTriangle.m_projectedV2.m_position.y);
		});

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
		cluster.m_rectangle.m_points[0] = longestEdgeStart;
		cluster.m_rectangle.m_points[3] = longestEdgeStart + length * lengthVector;
		cluster.m_rectangle.m_points[1] = cluster.m_rectangle.m_points[0] + width * widthVector;
		cluster.m_rectangle.m_points[2] = cluster.m_rectangle.m_points[3] + width * widthVector;
	}
	else
	{
		cluster.m_rectangle = RotatingCalipers::GetMinAreaRectangle(std::move(points));
	}
	cluster.m_rectangle.Update();
	//Rasterization
	//Calculate texture size
	size_t textureWidth, textureHeight;
	const auto& boundingRectangle = cluster.m_rectangle;
	textureWidth = static_cast<size_t>(boundingRectangle.m_width * projectSettings.m_resolutionFactor);
	textureHeight = static_cast<size_t>(boundingRectangle.m_height * projectSettings.m_resolutionFactor);
	if (textureWidth < 1 || textureHeight < 1) return;
	CPUFramebuffer<glm::vec4> albedoFrameBuffer(textureWidth, textureHeight);
	CPUFramebuffer<glm::vec3> normalFrameBuffer(textureWidth, textureHeight);
	CPUFramebuffer<float> roughnessFrameBuffer(textureWidth, textureHeight);
	CPUFramebuffer<float> metallicFrameBuffer(textureWidth, textureHeight);
	CPUFramebuffer<float> aoFrameBuffer(textureWidth, textureHeight);



	float averageRoughness = 1.f;
	float averageMetallic = 0.0f;

	Jobs::RunParallelFor(projectedTriangles.size(), [&](const unsigned triangleIndex)
		{
			const auto& triangle = projectedTriangles[triangleIndex];
			const auto& v0 = triangle.m_projectedV0;
			const auto& v1 = triangle.m_projectedV1;
			const auto& v2 = triangle.m_projectedV2;
			const auto& material = pbrMaterials.at(triangle.m_materialHandle);
			glm::vec3 textureSpaceVertices[3];
			textureSpaceVertices[0] = boundingRectangle.Transform(v0.m_position) * projectSettings.m_resolutionFactor;
			textureSpaceVertices[1] = boundingRectangle.Transform(v1.m_position) * projectSettings.m_resolutionFactor;
			textureSpaceVertices[2] = boundingRectangle.Transform(v2.m_position) * projectSettings.m_resolutionFactor;

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
					auto albedo = material.m_baseAlbedo;
					float roughness = material.m_baseRoughness;
					float metallic = material.m_baseMetallic;
					float ao = material.m_baseAo;
					if (!material.m_albedoTextureData.empty())
					{
						int textureX = static_cast<int>(material.m_albedoTextureResolution.x * texCoords.x) % material.m_albedoTextureResolution.x;
						int textureY = static_cast<int>(material.m_albedoTextureResolution.y * texCoords.y) % material.m_albedoTextureResolution.y;
						if (textureX < 0) textureX += material.m_albedoTextureResolution.x;
						if (textureY < 0) textureY += material.m_albedoTextureResolution.y;

						const auto index = textureY * material.m_albedoTextureResolution.x + textureX;
						albedo = material.m_albedoTextureData[index];
					}
					//Alpha discard
					if (albedo.a < 0.1f) continue;
					auto normal = glm::normalize(bc0 * v0.m_normal + bc1 * v1.m_normal + bc2 * v2.m_normal);

					if (!material.m_normalTextureData.empty())
					{
						auto tangent = glm::normalize(bc0 * v0.m_tangent + bc1 * v1.m_tangent + bc2 * v2.m_tangent);
						const auto biTangent = glm::cross(normal, tangent);
						const auto tbn = glm::mat3(tangent, biTangent, normal);

						int textureX = static_cast<int>(material.m_normalTextureResolution.x * texCoords.x) % material.m_normalTextureResolution.x;
						int textureY = static_cast<int>(material.m_normalTextureResolution.y * texCoords.y) % material.m_normalTextureResolution.y;
						if (textureX < 0) textureX += material.m_normalTextureResolution.x;
						if (textureY < 0) textureY += material.m_normalTextureResolution.y;

						const auto index = textureY * material.m_normalTextureResolution.x + textureX;
						const auto sampled = glm::normalize(material.m_normalTextureData[index]) * 2.0f - glm::vec3(1.0f);
						normal = glm::normalize(tbn * sampled);
					}
					if (glm::dot(normal, glm::vec3(0, 0, 1)) < 0) normal = -normal;

					if (!material.m_roughnessTextureData.empty())
					{
						int textureX = static_cast<int>(material.m_roughnessTextureResolution.x * texCoords.x) % material.m_roughnessTextureResolution.x;
						int textureY = static_cast<int>(material.m_roughnessTextureResolution.y * texCoords.y) % material.m_roughnessTextureResolution.y;
						if (textureX < 0) textureX += material.m_roughnessTextureResolution.x;
						if (textureY < 0) textureY += material.m_roughnessTextureResolution.y;


						const auto index = textureY * material.m_roughnessTextureResolution.x + textureX;
						roughness = material.m_roughnessTextureData[index];

					}
					if (!material.m_metallicTextureData.empty())
					{
						int textureX = static_cast<int>(material.m_metallicTextureResolution.x * texCoords.x) % material.m_metallicTextureResolution.x;
						int textureY = static_cast<int>(material.m_metallicTextureResolution.y * texCoords.y) % material.m_metallicTextureResolution.y;
						if (textureX < 0) textureX += material.m_metallicTextureResolution.x;
						if (textureY < 0) textureY += material.m_metallicTextureResolution.y;

						const auto index = textureY * material.m_metallicTextureResolution.x + textureX;
						metallic = material.m_metallicTextureData[index];

					}
					if (!material.m_aoTextureData.empty())
					{
						int textureX = static_cast<int>(material.m_aoTextureResolution.x * texCoords.x) % material.m_aoTextureResolution.x;
						int textureY = static_cast<int>(material.m_aoTextureResolution.y * texCoords.y) % material.m_aoTextureResolution.y;
						if (textureX < 0) textureX += material.m_aoTextureResolution.x;
						if (textureY < 0) textureY += material.m_aoTextureResolution.y;


						const auto index = textureY * material.m_aoTextureResolution.x + textureX;
						ao = material.m_aoTextureData[index];

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

	cluster.m_billboardMaterial = ProjectManager::CreateTemporaryAsset<Material>();
	std::shared_ptr<Texture2D> albedoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
	albedoTexture->SetRgbaChannelData(albedoFrameBuffer.m_colorBuffer, glm::uvec2(albedoFrameBuffer.m_width, albedoFrameBuffer.m_height));
	cluster.m_billboardMaterial->SetAlbedoTexture(albedoTexture);
	if (projectSettings.m_transferNormalMap) {
		std::shared_ptr<Texture2D> normalTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		normalTexture->SetRgbChannelData(normalFrameBuffer.m_colorBuffer, glm::uvec2(normalFrameBuffer.m_width, normalFrameBuffer.m_height));
		cluster.m_billboardMaterial->SetNormalTexture(normalTexture);
	}
	if (projectSettings.m_transferRoughnessMap) {
		std::shared_ptr<Texture2D> roughnessTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		roughnessTexture->SetRedChannelData(roughnessFrameBuffer.m_colorBuffer, glm::uvec2(roughnessFrameBuffer.m_width, roughnessFrameBuffer.m_height));
		cluster.m_billboardMaterial->SetRoughnessTexture(roughnessTexture);
	}
	else
	{
		cluster.m_billboardMaterial->m_materialProperties.m_roughness = averageRoughness;
	}
	if (projectSettings.m_transferMetallicMap) {
		std::shared_ptr<Texture2D> metallicTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		metallicTexture->SetRedChannelData(metallicFrameBuffer.m_colorBuffer, glm::uvec2(metallicFrameBuffer.m_width, metallicFrameBuffer.m_height));
		cluster.m_billboardMaterial->SetMetallicTexture(metallicTexture);
	}
	else
	{
		cluster.m_billboardMaterial->m_materialProperties.m_metallic = averageMetallic;
	}
	if (projectSettings.m_transferAoMap) {
		std::shared_ptr<Texture2D> aoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		aoTexture->SetRedChannelData(aoFrameBuffer.m_colorBuffer, glm::uvec2(aoFrameBuffer.m_width, aoFrameBuffer.m_height));
		cluster.m_billboardMaterial->SetAOTexture(aoTexture);
	}

	cluster.m_billboardMesh = ProjectManager::CreateTemporaryAsset<Mesh>();

	std::vector<Vertex> planeVertices;
	planeVertices.resize(4);

	const auto inverseRotateMatrix = glm::inverse(rotateMatrix);

	planeVertices[0].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[0].x, cluster.m_rectangle.m_points[0].y, 0.f, 0.f);
	planeVertices[0].m_texCoord = glm::vec2(0, 0);
	planeVertices[1].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[1].x, cluster.m_rectangle.m_points[1].y, 0.f, 0.f);
	planeVertices[1].m_texCoord = glm::vec2(1, 0);
	planeVertices[2].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[2].x, cluster.m_rectangle.m_points[2].y, 0.f, 0.f);
	planeVertices[2].m_texCoord = glm::vec2(1, 1);
	planeVertices[3].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[3].x, cluster.m_rectangle.m_points[3].y, 0.f, 0.f);
	planeVertices[3].m_texCoord = glm::vec2(0, 1);


	planeVertices[0].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;
	planeVertices[1].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;
	planeVertices[2].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;
	planeVertices[3].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;

	std::vector<glm::uvec3> planeTriangles;
	planeTriangles.resize(2);
	planeTriangles[0] = { 0, 1, 2 };
	planeTriangles[1] = { 2, 3, 0 };
	cluster.m_billboardMesh->SetVertices({}, planeVertices, planeTriangles);
}

#pragma endregion
#pragma region IO
Entity BillboardCloud::BuildEntity(const std::shared_ptr<Scene>& scene) const
{
	if (m_elementCollections.empty()) return {};
	const auto owner = scene->CreateEntity("Billboard Cloud");
	for (const auto& elementCollection : m_elementCollections) {
		for (const auto& cluster : elementCollection.m_clusters) {
			if (!cluster.m_billboardMesh || !cluster.m_billboardMaterial) continue;
			const auto projectedElementEntity = scene->CreateEntity("Billboard");
			const auto elementMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(projectedElementEntity).lock();
			elementMeshRenderer->m_mesh = cluster.m_billboardMesh;
			elementMeshRenderer->m_material = cluster.m_billboardMaterial;
			scene->SetParent(projectedElementEntity, owner);
		}
	}
	return owner;
}
void BillboardCloud::PreprocessPrefab(std::vector<ElementCollection>& elementCollections, const std::shared_ptr<Prefab>& currentPrefab, const Transform& parentModelSpaceTransform)
{
	Transform transform{};
	for (const auto& dataComponent : currentPrefab->m_dataComponents)
	{
		if (dataComponent.m_type == Typeof<Transform>())
		{
			transform = *std::reinterpret_pointer_cast<Transform>(dataComponent.m_data);
		}
	}
	transform.m_value = parentModelSpaceTransform.m_value * transform.m_value;
	for (const auto& privateComponent : currentPrefab->m_privateComponents)
	{

		if (privateComponent.m_data->GetTypeName() == "MeshRenderer")
		{
			std::vector<AssetRef> assetRefs;
			privateComponent.m_data->CollectAssetRef(assetRefs);
			std::shared_ptr<Mesh> mesh{};
			std::shared_ptr<Material> material{};
			for (auto& assetRef : assetRefs)
			{
				if (const auto testMesh = assetRef.Get<Mesh>())
				{
					mesh = testMesh;
				}
				else if (const auto testMaterial = assetRef.Get<Material>())
				{
					material = testMaterial;
				}
			}
			if (mesh && material)
			{
				elementCollections.emplace_back();
				auto& targetCluster = elementCollections.back();
				targetCluster.m_elements.emplace_back();
				auto& element = targetCluster.m_elements.back();
				element.m_vertices = mesh->UnsafeGetVertices();
				element.m_material = material;
				element.m_triangles = mesh->UnsafeGetTriangles();
				Jobs::RunParallelFor(element.m_vertices.size(), [&](unsigned vertexIndex)
					{
						TransformVertex(element.m_vertices.at(vertexIndex), parentModelSpaceTransform.m_value);

					});
			}
		}
	}
	for (const auto& childPrefab : currentPrefab->m_children)
	{
		PreprocessPrefab(elementCollections, childPrefab, transform);
	}
}

void BillboardCloud::PreprocessPrefab(ElementCollection& elementCollection, const std::shared_ptr<Prefab>& currentPrefab,
	const Transform& parentModelSpaceTransform)
{
	Transform transform{};
	for (const auto& dataComponent : currentPrefab->m_dataComponents)
	{
		if (dataComponent.m_type == Typeof<Transform>())
		{
			transform = *std::reinterpret_pointer_cast<Transform>(dataComponent.m_data);
		}
	}
	transform.m_value = parentModelSpaceTransform.m_value * transform.m_value;
	for (const auto& privateComponent : currentPrefab->m_privateComponents)
	{

		if (privateComponent.m_data->GetTypeName() == "MeshRenderer")
		{
			std::vector<AssetRef> assetRefs;
			privateComponent.m_data->CollectAssetRef(assetRefs);
			std::shared_ptr<Mesh> mesh{};
			std::shared_ptr<Material> material{};
			for (auto& assetRef : assetRefs)
			{
				if (const auto testMesh = assetRef.Get<Mesh>())
				{
					mesh = testMesh;
				}
				else if (const auto testMaterial = assetRef.Get<Material>())
				{
					material = testMaterial;
				}
			}
			if (mesh && material)
			{
				elementCollection.m_elements.emplace_back();
				auto& element = elementCollection.m_elements.back();
				element.m_vertices = mesh->UnsafeGetVertices();
				element.m_material = material;
				element.m_triangles = mesh->UnsafeGetTriangles();
				Jobs::RunParallelFor(element.m_vertices.size(), [&](unsigned vertexIndex)
					{
						TransformVertex(element.m_vertices.at(vertexIndex), parentModelSpaceTransform.m_value);
					});
			}
		}
	}
	for (const auto& childPrefab : currentPrefab->m_children)
	{
		PreprocessPrefab(elementCollection, childPrefab, transform);
	}
}

#pragma endregion
#pragma region Clusterization
void BillboardCloud::BuildClusters(const std::shared_ptr<Prefab>& prefab, const ClusterizationSettings& clusterizeSettings, const bool combine)
{
	if (!clusterizeSettings.m_append) m_elementCollections.clear();
	if (combine) {
		m_elementCollections.emplace_back();
		auto& newElementCollection = m_elementCollections.back();
		PreprocessPrefab(newElementCollection, prefab, Transform());
		newElementCollection.Clusterize(clusterizeSettings);
	}
	else
	{
		PreprocessPrefab(m_elementCollections, prefab, Transform());

		for (auto& elementCollection : m_elementCollections)
		{
			elementCollection.Clusterize(clusterizeSettings);
		}
	}
}

void BillboardCloud::ElementCollection::Clusterize(const ClusterizationSettings& clusterizeSettings)
{
	m_clusters.clear();
	switch (clusterizeSettings.m_clusterizeMode)
	{
	case ClusterizationMode::PassThrough:
	{
		m_clusters.emplace_back();
		auto& cluster = m_clusters.back();
		AppendTriangles(cluster.m_triangles, m_elements);
	}
	break;
	case ClusterizationMode::Stochastic:
	{
		std::vector<ClusterTriangle> operatingTriangles;
		AppendTriangles(operatingTriangles, m_elements);
		m_clusters = StochasticClusterize(std::move(operatingTriangles), clusterizeSettings);
	}
	break;
	case ClusterizationMode::Default:
	{
		std::vector<ClusterTriangle> operatingTriangles;
		AppendTriangles(operatingTriangles, m_elements);
		m_clusters = DefaultClusterize(std::move(operatingTriangles), clusterizeSettings);
	}
	break;
	}
}
struct BoundingSphere
{
	glm::vec3 m_center = glm::vec3(0.f);
	float m_radius = 0.f;

	void Initialize(const std::vector<BillboardCloud::Element>& elements)
	{
		size_t triangleSize = 0;
		auto positionSum = glm::vec3(0.f);

		//TODO: Parallelize
		for (const auto& element : elements)
		{
			for (const auto& triangle : element.m_triangles)
			{
				triangleSize++;
				positionSum += element.CalculateCentroid(triangle);
			}
		}
		m_center = positionSum / static_cast<float>(triangleSize);

		m_radius = 0.f;
		for (const auto& element : elements)
		{
			for (const auto& vertex : element.m_vertices)
			{
				m_radius = glm::max(m_radius, glm::distance(m_center, vertex.m_position));
			}
		}
	}
};


class Discretization {
public:
	class Bin
	{
	public:
		float m_density;
		float m_thetaMin;
		float m_thetaMax;
		float m_phiMin;
		float m_phiMax;
		float m_roMin;
		float m_roMax;
		float m_thetaCenter;
		float m_phiCenter;
		float m_roCenter;
		glm::vec3 m_centerNormal;

		Bin()
			:m_density(0.0f),
			m_thetaMin(0.0f),
			m_thetaMax(0.0f),
			m_phiMin(0.0f),
			m_phiMax(0.0f),
			m_roMin(0.0f),
			m_roMax(0.0f),
			m_thetaCenter(0.0f),
			m_phiCenter(0.0f),
			m_roCenter(0.0f)
		{
		}

		Bin(const float thetaMin, const float thetaMax, const float phiMin, const float phiMax, const float roMin, const float roMax, const float density = 0)
			: m_density(density), m_thetaMin(thetaMin), m_thetaMax(thetaMax), m_phiMin(phiMin), m_phiMax(phiMax), m_roMin(roMin), m_roMax(roMax)
		{
			CalculateCenter();
			CalculateCenterNormal();
		}

		Bin(const Bin& bin)
		{
			m_density = bin.m_density;
			m_thetaMin = bin.m_thetaMin;
			m_thetaMax = bin.m_thetaMax;
			m_phiMin = bin.m_phiMin;
			m_phiMax = bin.m_phiMax;
			m_roMin = bin.m_roMin;
			m_roMax = bin.m_roMax;
			m_thetaCenter = bin.m_thetaCenter;
			m_phiCenter = bin.m_phiCenter;
			m_roCenter = bin.m_roCenter;
			m_centerNormal = bin.m_centerNormal;
		}

		Bin& operator=(const Bin& bin) = default;

	private:
		void CalculateCenter()
		{
			m_thetaCenter = (m_thetaMin + m_thetaMax) / 2;
			m_phiCenter = (m_phiMin + m_phiMax) / 2;
			m_roCenter = (m_roMin + m_roMax) / 2;
		}

		// calculate bin center's normal
		void CalculateCenterNormal()
		{
			const float z = glm::sin(m_phiCenter);
			const float xy = glm::cos(m_phiCenter);
			const float x = xy * glm::cos(m_thetaCenter);
			const float y = xy * glm::sin(m_thetaCenter);

			m_centerNormal = glm::vec3(x, y, z);
		}
	};
	/// m_bins
	std::vector<std::vector<std::vector<Bin>>> m_bins;
	/// fail-safe mode para
	bool m_failSafeModeTriggered;
	/// fitted plane in fail-safe mode 
	std::vector<BillboardCloud::ClusterTriangle> m_bestFittedPlaneValidTriangle;

	float m_epsilon;
	float m_weightPenalty;
	/// discretization num
	int m_discretizeThetaNum;
	int m_discretizePhiNum;
	int m_discretizeRoNum;
	/// range
	float m_thetaMax;
	float m_thetaMin;
	float m_phiMax;
	float m_phiMin;
	float m_roMax;
	float m_roMin;
	/// gap
	float m_thetaGap;
	float m_phiGap;
	float m_roGap;

	Discretization(const float roMax, const float epsilon, const int discretizeThetaNum, const int discretizePhiNum, const int discretizeRoNum)
		:m_failSafeModeTriggered(false),
		m_epsilon(epsilon),
		m_weightPenalty(10),
		m_discretizeThetaNum(discretizeThetaNum),
		m_discretizePhiNum(discretizePhiNum),
		m_discretizeRoNum(discretizeRoNum),
		m_thetaMax(2 * glm::pi<float>()),
		m_thetaMin(0),
		m_phiMax(glm::pi<float>() / 2),   // recommend "10" in paper
		m_phiMin(-glm::pi<float>() / 2),
		m_roMax(roMax),
		m_roMin(0)
	{
		m_thetaGap = (m_thetaMax - m_thetaMin) / static_cast<float>(discretizeThetaNum);
		m_phiGap = (m_phiMax - m_phiMin) / static_cast<float>(discretizePhiNum);
		m_roGap = (roMax - m_roMin) / static_cast<float>(discretizeRoNum);

		for (int i = 0; i < discretizeRoNum; i++)
		{
			std::vector<std::vector<Bin>> tmp1;
			for (int j = 0; j < discretizePhiNum; j++)
			{
				std::vector<Bin> tmp2;
				for (int k = 0; k < discretizeThetaNum; k++)
				{
					const float thetaMinTmp = m_thetaGap * k + m_thetaMin;
					const float thetaMaxTmp = thetaMinTmp + m_thetaGap;
					const float phiMinTmp = m_phiGap * j + m_phiMin;
					const float phiMaxTmp = phiMinTmp + m_phiGap;
					const float roMinTmp = m_roGap * i + m_roMin;
					const float roMaxTmp = roMinTmp + m_roGap;
					Bin bin(thetaMinTmp, thetaMaxTmp, phiMinTmp, phiMaxTmp, roMinTmp, roMaxTmp);
					tmp2.emplace_back(bin);
				}
				tmp1.emplace_back(tmp2);
			}
			m_bins.emplace_back(tmp1);
		}
	}
	/// trans the spherical coordinate of a plane into the normal vector
	static glm::vec3 SphericalCoordToNormal(const float theta, const float phi)
	{
		const float z = glm::sin(phi);
		const float xy = glm::cos(phi);
		const float x = xy * glm::cos(theta);
		const float y = xy * glm::sin(theta);
		return { x, y, z };
	}
	/// compute the min and max value of ro in the case of triangle is valid for the specific theta and phi range
	[[nodiscard]] bool ComputeRoMinMax(glm::vec2& minMax, const BillboardCloud::Element& element, const BillboardCloud::ClusterTriangle& clusterTriangle, float curThetaMin, float curThetaMax, float curPhiMin, float curPhiMax) const
	{
		const auto normal1 = SphericalCoordToNormal(curThetaMin, curPhiMin);
		const auto normal2 = SphericalCoordToNormal(curThetaMin, curPhiMax);
		const auto normal3 = SphericalCoordToNormal(curThetaMax, curPhiMin);
		const auto normal4 = SphericalCoordToNormal(curThetaMax, curPhiMax);

		const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
		const auto p0 = element.m_vertices.at(triangle.x).m_position;
		const auto p1 = element.m_vertices.at(triangle.y).m_position;
		const auto p2 = element.m_vertices.at(triangle.z).m_position;

		const float roP0N1 = glm::dot(p0, normal1);
		const float roP0N2 = glm::dot(p0, normal2);
		const float roP0N3 = glm::dot(p0, normal3);
		const float roP0N4 = glm::dot(p0, normal4);

		const float roP1N1 = glm::dot(p1, normal1);
		const float roP1N2 = glm::dot(p1, normal2);
		const float roP1N3 = glm::dot(p1, normal3);
		const float roP1N4 = glm::dot(p1, normal4);

		const float roP2N1 = glm::dot(p2, normal1);
		const float roP2N2 = glm::dot(p2, normal2);
		const float roP2N3 = glm::dot(p2, normal3);
		const float roP2N4 = glm::dot(p2, normal4);

		const float tmp0[] = { roP0N1 - m_epsilon, roP0N2 - m_epsilon, roP0N3 - m_epsilon, roP0N4 - m_epsilon };
		const float tmp1[] = { roP1N1 - m_epsilon, roP1N2 - m_epsilon, roP1N3 - m_epsilon, roP1N4 - m_epsilon };
		const float tmp2[] = { roP2N1 - m_epsilon, roP2N2 - m_epsilon, roP2N3 - m_epsilon, roP2N4 - m_epsilon };
		const float roP0MinN = glm::min(glm::min(tmp0[0], tmp0[1]), glm::min(tmp0[2], tmp0[3]));
		const float roP1MinN = glm::min(glm::min(tmp1[0], tmp1[1]), glm::min(tmp1[2], tmp1[3]));
		const float roP2MinN = glm::min(glm::min(tmp2[0], tmp2[1]), glm::min(tmp2[2], tmp2[3]));
		const float tmp3[] = { roP0MinN, roP1MinN, roP2MinN };
		//roMin

		float roMaxPMinN = glm::max(tmp3[0], glm::max(tmp3[1], tmp3[2]));
		

		const float tmp4[] = { roP0N1 + m_epsilon, roP0N2 + m_epsilon, roP0N3 + m_epsilon, roP0N4 + m_epsilon };
		const float tmp5[] = { roP1N1 + m_epsilon, roP1N2 + m_epsilon, roP1N3 + m_epsilon, roP1N4 + m_epsilon };
		const float tmp6[] = { roP2N1 + m_epsilon, roP2N2 + m_epsilon, roP2N3 + m_epsilon, roP2N4 + m_epsilon };
		const float roP0MaxN = glm::max(glm::max(tmp4[0], tmp4[1]), glm::max(tmp4[2], tmp4[3]));
		const float roP1MaxN = glm::max(glm::max(tmp5[0], tmp5[1]), glm::max(tmp5[2], tmp5[3]));
		const float roP2MaxN = glm::max(glm::max(tmp6[0], tmp6[1]), glm::max(tmp6[2], tmp6[3]));
		const float tmp7[] = { roP0MaxN, roP1MaxN, roP2MaxN };
		//roMax
		float roMinPMaxN = glm::min(tmp7[0], glm::min(tmp7[1], tmp7[2]));
		if(roMinPMaxN < m_roMin) false;

		roMaxPMinN = glm::clamp(roMaxPMinN, m_roMin, m_roMax);
		roMinPMaxN = glm::min(roMinPMaxN, m_roMax);
		minMax = { roMaxPMinN, roMinPMaxN };
		return true;
	}
	void UpdateDensity(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, const bool add)
	{
		if (clusterTriangles.empty())
		{
			EVOENGINE_ERROR("ERROR: The size of the input triangles is 0, that means in the last iteration, there is no fitted plane found!");
			return;
		}

		const float time = clock();
		for (auto& clusterTriangle : clusterTriangles)
		{
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			const auto triangleNormal = element.CalculateNormal(clusterTriangle.m_triangleIndex);
			const auto triangleArea = element.CalculateArea(clusterTriangle.m_triangleIndex);
			for (int i = 0; i < m_discretizePhiNum; i++)        // phiCoord
			{
				for (int j = 0; j < m_discretizeThetaNum; j++)  // thetaCoord
				{
					glm::vec2 roMaxMin;
					if(!ComputeRoMinMax(roMaxMin, element, clusterTriangle,
						m_thetaMin + j * m_thetaGap,
						m_thetaMin + (j + 1) * m_thetaGap,
						m_phiMin + i * m_phiGap,
						m_phiMin + (i + 1) * m_phiGap))
					{
						continue;
					}
					//if (roMaxMin.x < 0.f && roMaxMin.y < 0.f)
					//	continue;
					const float roMin = roMaxMin.x;
					const float roMax = roMaxMin.y;

					// add coverage, m_bins between (roMin, roMax)
					const int roCoordMin = glm::clamp(static_cast<int>((roMin - m_roMin) / m_roGap), 0, m_discretizeRoNum - 1);
					const int roCoordMax = glm::clamp(static_cast<int>((roMax - m_roMin) / m_roGap), 0, m_discretizeRoNum - 1);

					if (roCoordMax - roCoordMin > 2)
					{
						if (add)
						{
							// use the center point's normal of the bin to calculate the projected triangle area
							m_bins[roCoordMin][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}
						else
						{
							m_bins[roCoordMin][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}
						for (int k = roCoordMin + 1; k < roCoordMax; k++)
						{
							if (add)
							{
								m_bins[k][i][j].m_density += triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[k][i][j].m_centerNormal));
							}
							else
							{
								m_bins[k][i][j].m_density -= triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[k][i][j].m_centerNormal));
							}
						}
						if (add)
						{
							m_bins[roCoordMax][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
						else
						{
							m_bins[roCoordMax][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
					}
					else if (roCoordMax - roCoordMin == 1)
					{
						if (add)
						{
							m_bins[roCoordMin][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}
						else
						{
							m_bins[roCoordMin][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}

						if (add)
						{
							m_bins[roCoordMax][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
						else
						{
							m_bins[roCoordMax][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
					}
					else if (roCoordMax - roCoordMin == 0)
					{
						if (add)
						{
							m_bins[roCoordMin][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal));
						}
						else
						{
							m_bins[roCoordMin][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal));
						}
					}

					// add penalty ,m_bins between (ro_min - epsilon, ro_min)
					if (roMin - m_epsilon - m_roMin > 0)
					{
						int roMinMinusEpsilonCoord = (roMin - m_epsilon - m_roMin) / m_roGap;
						for (int m = roMinMinusEpsilonCoord; m <= roCoordMin; m++)
						{
							if (add)
							{
								m_bins[m][i][j].m_density -= triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[m][i][j].m_centerNormal)) *
									m_weightPenalty;
							}
							else
							{
								m_bins[m][i][j].m_density += triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[m][i][j].m_centerNormal)) *
									m_weightPenalty;
							}
						}
					}
				}
			}
		}
		EVOENGINE_LOG("Updating_density... [time :" + std::to_string((clock() - time) / 1000.f) + "s]");
	}

	[[nodiscard]] float ComputeMaxDensity(glm::ivec3& binIndex) const
	{
		// pick bin with max density
		float maxDensity = -FLT_MAX;
		for (int i = 0; i < m_discretizeRoNum; i++)
		{
			for (int j = 0; j < m_discretizePhiNum; j++)
			{
				for (int k = 0; k < m_discretizeThetaNum; k++)
				{
					if (float tmp = m_bins[i][j][k].m_density; maxDensity < tmp)
					{
						maxDensity = tmp;
						binIndex.x = i;
						binIndex.y = j;
						binIndex.z = k;
					}
				}
			}
		}
		return maxDensity;
	}

	[[nodiscard]] std::vector<BillboardCloud::ClusterTriangle> ComputeBinValidSet(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, const Bin& bin) const
	{
		std::vector<BillboardCloud::ClusterTriangle> binValidSet;
		for (auto& clusterTriangle : clusterTriangles)
		{
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			// we use the notion of "simple validity":
			// that is a bin is valid for a triangle as long as there exists a valid plane for the triangle in the bin 
			// if the ro min and ro max is in the range of bin's ro range, we think this triangle is valid for the bin
			glm::vec2 roMinMax;
			if(!ComputeRoMinMax(roMinMax, element, clusterTriangle, bin.m_thetaMin, bin.m_thetaMax, bin.m_phiMin, bin.m_phiMax))
			{
				continue;
			}

			if (!(roMinMax.y < bin.m_roMin) &&
				!(roMinMax.x > bin.m_roMax))
			{
				binValidSet.emplace_back(clusterTriangle);
			}
		}
		return binValidSet;
	}
	std::vector<int> ComputePlaneValidSetIndex(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, const Plane& plane) const
	{
		std::vector<int> planeValidSetIndex;
		const auto planeNormal = plane.GetNormal();
		const auto planeDistance = plane.GetDistance();
		for (int i = 0; i < clusterTriangles.size(); i++)
		{
			const auto& clusterTriangle = clusterTriangles.at(i);
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
			const auto p0 = element.m_vertices.at(triangle.x).m_position;
			const auto p1 = element.m_vertices.at(triangle.y).m_position;
			const auto p2 = element.m_vertices.at(triangle.z).m_position;

			const float roP0N = glm::abs(glm::dot(p0, planeNormal));
			const float roP1N = glm::abs(glm::dot(p1, planeNormal));
			const float roP2N = glm::abs(glm::dot(p2, planeNormal));
			const float tmp0[] = { roP0N - m_epsilon ,roP1N - m_epsilon ,roP2N - m_epsilon };
			const float tmp1[] = { roP0N + m_epsilon ,roP1N + m_epsilon ,roP2N + m_epsilon };

			const float roMinTmp = glm::min(tmp0[0], glm::min(tmp0[1], tmp0[2]));
			const float roMaxTmp = glm::max(tmp1[0], glm::max(tmp1[1], tmp1[2]));

			if (planeDistance > roMinTmp && planeDistance < roMaxTmp)
				planeValidSetIndex.emplace_back(i);
		}
		return planeValidSetIndex;
	}
	void ComputeDensity(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, Bin& bin) const
	{
		for (auto& clusterTriangle : clusterTriangles)
		{
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			const auto triangleNormal = element.CalculateNormal(clusterTriangle.m_triangleIndex);
			const auto triangleArea = element.CalculateArea(clusterTriangle.m_triangleIndex);
			glm::vec2 roMinMax;
			if(!ComputeRoMinMax(roMinMax, element, clusterTriangle, bin.m_thetaMin, bin.m_thetaMax, bin.m_phiMin, bin.m_phiMax))
			{
				continue;
			}
			// add coverage
			const float curRoMin = roMinMax.x;
			const float curRoMax = roMinMax.y;
			const float curRoGap = bin.m_roMax - bin.m_roMin;
			if (curRoMin < bin.m_roMin && curRoMax > bin.m_roMin && curRoMax < bin.m_roMax)
			{
				bin.m_density += triangleArea * glm::abs(glm::dot(triangleNormal, bin.m_centerNormal)) * (curRoMax - bin.m_roMin) / curRoGap;
			}
			else if (curRoMin > bin.m_roMin && curRoMin < bin.m_roMax && curRoMax > bin.m_roMax)
			{
				bin.m_density += triangleArea * glm::abs(glm::dot(triangleNormal, bin.m_centerNormal)) * (bin.m_roMax - curRoMin) / curRoGap;
			}
			else if (curRoMin >= bin.m_roMin && curRoMax <= bin.m_roMax)
			{
				bin.m_density += triangleArea * glm::abs(glm::dot(triangleNormal, bin.m_centerNormal));
			}
		}
	}

	std::vector<Bin> GetNeighbors(const Bin& bin) const
	{
		const float curBinThetaGap = bin.m_thetaMax - bin.m_thetaMin;
		const float curBinPhiGap = bin.m_phiMax - bin.m_phiMin;
		const float curBinRoGap = bin.m_roMax - bin.m_roMin;

		std::vector<Bin> binsTmp;
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				for (int k = -1; k <= 1; k++)
				{
					float neighborThetaMin = bin.m_thetaMin + curBinThetaGap * k;
					float neighborThetaMax = neighborThetaMin + curBinThetaGap;
					float neighborPhiMin = bin.m_phiMin + curBinPhiGap * j;
					float neighborPhiMax = neighborPhiMin + curBinPhiGap;
					const float neighborRoMin = bin.m_roMin + curBinRoGap * i;
					const float neighborRoMax = neighborRoMin + curBinRoGap;

					if (neighborPhiMin < m_phiMin)
					{
						neighborPhiMin = -(neighborPhiMin - m_phiMin);
						neighborThetaMin = neighborThetaMin + glm::pi<float>();
					}
					if (neighborPhiMax > m_phiMax)
					{
						neighborPhiMax = glm::pi<float>() / 2 - (neighborPhiMax - m_phiMax);
						neighborThetaMin = neighborThetaMin + glm::pi<float>();
					}

					if (neighborThetaMin > 2 * glm::pi<float>())
					{
						neighborThetaMin = neighborThetaMin - 2 * glm::pi<float>();
					}
					if (neighborThetaMax > 2 * glm::pi<float>())
					{
						neighborThetaMax = neighborThetaMax - 2 * glm::pi<float>();
					}

					// if the bin's ro range is outside the specific range, discard it!
					if (neighborRoMin < m_roMin || neighborRoMax > m_roMax)
						continue;

					Bin binTmp(neighborThetaMin, neighborThetaMax, neighborPhiMin, neighborPhiMax, neighborRoMin, neighborRoMax);
					binsTmp.emplace_back(binTmp);
				}
			}
		}
		return binsTmp;
	}

	static std::vector<Bin> SubdivideBin(const Bin& bin)
	{
		std::vector<Bin> binsTmp;
		for (int i = 0; i <= 1; i++)
		{
			for (int j = 0; j <= 1; j++)
			{
				for (int k = 0; k <= 1; k++)
				{
					const float curThetaMin = bin.m_thetaMin + (bin.m_thetaMax - bin.m_thetaMin) / 2 * k;
					const float curThetaMax = curThetaMin + (bin.m_thetaMax - bin.m_thetaMin) / 2;
					const float curPhiMin = bin.m_phiMin + (bin.m_phiMax - bin.m_phiMin) / 2 * j;
					const float curPhiMax = curPhiMin + (bin.m_phiMax - bin.m_phiMin) / 2;
					const float curRoMin = bin.m_roMin + (bin.m_roMax - bin.m_roMin) / 2 * i;
					const float curRoMax = curRoMin + (bin.m_roMax - bin.m_roMin) / 2;

					Bin binTmp(curThetaMin, curThetaMax, curPhiMin, curPhiMax, curRoMin, curRoMax);
					binsTmp.emplace_back(binTmp);
				}
			}
		}
		return binsTmp;
	}
	[[nodiscard]] Plane RefineBin(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& validSet, const Bin& maxDensityBin)
	{
		Plane centerPlane{ maxDensityBin.m_centerNormal, maxDensityBin.m_roCenter };
		std::vector<int> centerPlaneValidSetIndex = ComputePlaneValidSetIndex(elements, validSet, centerPlane);
		if (centerPlaneValidSetIndex.size() == validSet.size())
		{
			if (maxDensityBin.m_thetaCenter > glm::pi<float>())
			{
				centerPlane = Plane(-centerPlane.GetNormal(), centerPlane.GetDistance());
			}
			float maxDis = 0.0f;
			for (auto& clusterTriangle : validSet)
			{
				const auto& element = elements.at(clusterTriangle.m_elementIndex);
				const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
				const auto p0 = element.m_vertices.at(triangle.x).m_position;
				const auto p1 = element.m_vertices.at(triangle.y).m_position;
				const auto p2 = element.m_vertices.at(triangle.z).m_position;
				float d0 = centerPlane.CalculatePointDistance(p0);
				float d1 = centerPlane.CalculatePointDistance(p1);
				float d2 = centerPlane.CalculatePointDistance(p2);
				maxDis = d0 > d1 ? d0 : d1;
				maxDis = maxDis > d2 ? maxDis : d2;
			}
			return centerPlane;
		}

		Bin binMax;
		binMax.m_density = FLT_MIN;

		// pick the bin and its 26 neighbors (if have)
		std::vector<Bin> neighborBins = GetNeighbors(maxDensityBin);
		for (auto& neighborBin : neighborBins)
		{
			// subdivide the bin into 8 bins
			std::vector<Bin> subdividedBins = SubdivideBin(neighborBin);
			for (auto& subdividedBin : subdividedBins)
			{
				// pick the subdivide bin with max density
				ComputeDensity(elements, validSet, subdividedBin);
				if (subdividedBin.m_density > binMax.m_density)
				{
					binMax = subdividedBin;
				}
			}
		}
		std::vector<BillboardCloud::ClusterTriangle> binMaxValidSet = ComputeBinValidSet(elements, validSet, binMax);
		if (binMaxValidSet.empty())
		{
			EVOENGINE_ERROR("ERROR: subBinMax has no valid set, we will simply return the last densest bin's center plane!");

			// if the centerPlane has no valid set in the current remain sets, the iteration will end up with infinite loop!!!
			if (!centerPlaneValidSetIndex.empty())
			{
				EVOENGINE_ERROR("INFO: but last densest bin's center plane has valid set");
				EVOENGINE_ERROR("INFO: so we can simply return the last densest bin's center plane!");

				return centerPlane;
			}
			else
			{
				EVOENGINE_ERROR("ERROR: the centerPlane has no valid set in the current remain sets too");
				EVOENGINE_ERROR("INFO: so we return the best fitted plane of the last densest bin's valid set ");

				m_failSafeModeTriggered = true;
				m_bestFittedPlaneValidTriangle = validSet;
				float totalArea = 0.f;
				glm::vec3 centroidSum = glm::vec3(0.f);
				for (auto& clusterTriangle : validSet)
				{
					const auto& element = elements.at(clusterTriangle.m_elementIndex);
					const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
					const auto triangleArea = element.CalculateArea(triangle);
					const auto centroid = element.CalculateCentroid(triangle);
					totalArea += triangleArea;
					centroidSum += centroid * triangleArea;

				}
				auto centroid = centroidSum / totalArea;
				auto normal = glm::normalize(glm::vec3(centroid.y, centroid.z, centroid.x));
				if (glm::dot(centroid, normal) < 0)
				{
					normal = -normal;
				}
				float distance = glm::abs(glm::dot(centroid, normal));
				return Plane(normal, distance);
			}
		}
		return RefineBin(elements, binMaxValidSet, binMax);
	}
};

std::vector<BillboardCloud::Cluster> BillboardCloud::ElementCollection::StochasticClusterize(std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings)
{
	BoundingSphere boundingSphere;
	boundingSphere.Initialize(m_elements);

	float epsilon = boundingSphere.m_radius * clusterizeSettings.m_stochasticClusterizationSettings.m_epsilon;

	float maxPlaneSize = boundingSphere.m_radius * clusterizeSettings.m_stochasticClusterizationSettings.m_maxPlaneSize;
	m_skippedTriangles.clear();

	std::vector<Cluster> retVal;

	int epoch = 0;
	while (!operatingTriangles.empty())
	{
		Cluster newCluster;
		float maxArea = 0.f;
		std::vector<int> selectedTriangleIndices;
		std::mutex voteMutex;
		Jobs::RunParallelFor(clusterizeSettings.m_stochasticClusterizationSettings.m_iteration, [&](unsigned iteration)
			{
				int seedTriangleIndex = glm::linearRand(0, static_cast<int>(operatingTriangles.size()) - 1);
				ClusterTriangle seedTriangle = operatingTriangles.at(seedTriangleIndex);
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
				const auto planeCenter = (seedTriangleP0 + seedTriangleP1 + seedTriangleP2) / 3.f;
				float testPlaneDistance = glm::dot(seedTriangleP0, testPlaneNormal);
				float area = 0.f;
				std::vector<int> currentIterationSelectedTriangleIndices;
				for (int testTriangleIndex = 0; testTriangleIndex < operatingTriangles.size(); testTriangleIndex++)
				{
					const auto& testTriangle = operatingTriangles.at(testTriangleIndex);
					const auto& testTriangleElement = m_elements.at(testTriangle.m_elementIndex);
					const auto& testTriangleIndices = testTriangleElement.m_triangles.at(testTriangle.m_triangleIndex);
					const auto& testTriangleP0 = testTriangleElement.m_vertices.at(testTriangleIndices.x).m_position;
					const auto& testTriangleP1 = testTriangleElement.m_vertices.at(testTriangleIndices.y).m_position;
					const auto& testTriangleP2 = testTriangleElement.m_vertices.at(testTriangleIndices.z).m_position;

					if (glm::distance(planeCenter, testTriangleP0) > maxPlaneSize) continue;


					if (glm::abs(testPlaneDistance - glm::dot(testTriangleP0, testPlaneNormal)) > epsilon) continue;
					if (glm::abs(testPlaneDistance - glm::dot(testTriangleP1, testPlaneNormal)) > epsilon) continue;
					if (glm::abs(testPlaneDistance - glm::dot(testTriangleP2, testPlaneNormal)) > epsilon) continue;

					// increment projected area (Angular area Contribution)
					// use projected area Contribution
					float angle = glm::acos(glm::abs(glm::dot(testPlaneNormal, CalculateNormal(testTriangle))));
					float angular = (glm::pi<float>() / 2.f - angle) / (glm::pi<float>() / 2.f);
					area += CalculateArea(testTriangle) * angular;

					// save reference to T with billboard plane
					currentIterationSelectedTriangleIndices.emplace_back(testTriangleIndex);

				}
				if (!currentIterationSelectedTriangleIndices.empty()) {
					std::lock_guard lock(voteMutex);
					if (area > maxArea)
					{
						//Update cluster.
						newCluster.m_clusterPlane = Plane(testPlaneNormal, testPlaneDistance);
						newCluster.m_triangles.clear();
						for (const auto& triangleIndex : currentIterationSelectedTriangleIndices)
						{
							newCluster.m_triangles.emplace_back(operatingTriangles.at(triangleIndex));
						}

						selectedTriangleIndices = currentIterationSelectedTriangleIndices;
					}
				}
			}
		);

		if (selectedTriangleIndices.empty())
		{
			m_skippedTriangles.insert(m_skippedTriangles.end(), operatingTriangles.begin(), operatingTriangles.end());
			break;
		}

		retVal.emplace_back(std::move(newCluster));

		//Remove selected triangle from the remaining triangle.
		for (auto it = selectedTriangleIndices.rbegin(); it != selectedTriangleIndices.rend(); ++it)
		{
			operatingTriangles[*it] = operatingTriangles.back();
			operatingTriangles.pop_back();
		}
		epoch++;
		if (epoch >= clusterizeSettings.m_stochasticClusterizationSettings.m_timeout)
		{
			EVOENGINE_ERROR("Stochastic clustering timeout!")
				break;
		}
	}
	return std::move(retVal);
}


std::vector<BillboardCloud::Cluster> BillboardCloud::ElementCollection::DefaultClusterize(
	std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings)
{
	BoundingSphere boundingSphere;
	boundingSphere.Initialize(m_elements);
	const auto& settings = clusterizeSettings.m_defaultClusterizationSettings;
	const int roNum = static_cast<int>(1.5f / settings.m_epsilonPercentage);
	const float epsilon = boundingSphere.m_radius * settings.m_epsilonPercentage;

	float maxNormalDistance = 0.f;
	for (const auto& triangle : operatingTriangles)
	{
		maxNormalDistance = glm::max(maxNormalDistance, CalculateNormalDistance(triangle));
	}

	m_skippedTriangles.clear();

	std::vector<Cluster> retVal;

	int epoch = 0;

	Discretization discretization(maxNormalDistance, epsilon, settings.m_thetaNum, settings.m_phiNum, roNum);
	discretization.UpdateDensity(m_elements, operatingTriangles, true);

	while (!operatingTriangles.empty())
	{
		for (int i = 0; i < operatingTriangles.size(); i++)
		{
			operatingTriangles[i].m_index = i;
		}
		std::vector<int> selectedTriangleIndices;
		Cluster newCluster;
		glm::ivec3 maxDensityBinCoordinate;
		float maxDensity = discretization.ComputeMaxDensity(maxDensityBinCoordinate);
		const auto& maxDensityBin = discretization.m_bins[maxDensityBinCoordinate.x][maxDensityBinCoordinate.y][maxDensityBinCoordinate.z];
		const auto binValidSet = discretization.ComputeBinValidSet(m_elements, operatingTriangles, maxDensityBin);
		std::vector<ClusterTriangle> planeValidSet;
		if (!binValidSet.empty())
		{
			newCluster.m_clusterPlane = discretization.RefineBin(m_elements, binValidSet, maxDensityBin);

			if (discretization.m_failSafeModeTriggered)
			{
				planeValidSet = discretization.m_bestFittedPlaneValidTriangle;
				discretization.m_bestFittedPlaneValidTriangle.clear();

				EVOENGINE_ERROR("Fitted_triangle_num: " + std::to_string(planeValidSet.size()));

				// update density by removing the fitted triangle 
				discretization.UpdateDensity(m_elements, planeValidSet, false);

				// store bbc and corresponding fitted triangles
				newCluster.m_triangles = planeValidSet;
				//bbc.emplace_back(refinedPlane);

				for (auto& triangle : planeValidSet)
				{
					selectedTriangleIndices.emplace_back(triangle.m_index);
				}
				discretization.m_failSafeModeTriggered = false;
			}
			else
			{
				// get the fitted triangles index in the whole triangles
				selectedTriangleIndices = discretization.ComputePlaneValidSetIndex(m_elements, operatingTriangles, newCluster.m_clusterPlane);

				EVOENGINE_ERROR("Fitted_triangle_num: " + std::to_string(selectedTriangleIndices.size()));

				for (int index : selectedTriangleIndices)
				{
					planeValidSet.emplace_back(operatingTriangles[index]);
				}

				// update density by removing the fitted triangles
				discretization.UpdateDensity(m_elements, planeValidSet, false);

				// store bbc and corresponding fitted triangles
				newCluster.m_triangles = planeValidSet;
			}
		}
		else
		{
			m_skippedTriangles = operatingTriangles;
			break;
		}
		retVal.emplace_back(std::move(newCluster));
		//Remove selected triangle from the remaining triangle.
		for (auto it = selectedTriangleIndices.rbegin(); it != selectedTriangleIndices.rend(); ++it)
		{
			operatingTriangles[*it] = operatingTriangles.back();
			operatingTriangles.pop_back();
		}
		epoch++;
		if (epoch >= settings.m_timeout)
		{
			EVOENGINE_ERROR("Default clustering timeout!")
				break;
		}
	}

	return retVal;
}
#pragma endregion