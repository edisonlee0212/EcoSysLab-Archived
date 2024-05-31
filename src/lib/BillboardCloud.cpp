#include "BillboardCloud.hpp"
#include "Prefab.hpp"
#include "xatlas/xatlas.h"
using namespace EvoEngine;
#pragma region Projection

std::vector<BillboardCloud::ClusterTriangle> BillboardCloud::CollectTriangles() const
{
	std::vector<ClusterTriangle> retVal;
	for (int elementIndex = 0; elementIndex < m_elements.size(); elementIndex++)
	{
		const auto& element = m_elements.at(elementIndex);
		for (int triangleIndex = 0; triangleIndex < element.m_triangles.size(); triangleIndex++)
		{
			ClusterTriangle clusterTriangle;
			clusterTriangle.m_elementIndex = elementIndex;
			clusterTriangle.m_triangleIndex = triangleIndex;
			retVal.emplace_back(clusterTriangle);
		}
	}
	return retVal;
}

glm::vec3 BillboardCloud::CalculateCentroid(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateCentroid(triangle.m_triangleIndex);
}


float BillboardCloud::CalculateArea(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateArea(triangle.m_triangleIndex);
}

float BillboardCloud::CalculateNormalDistance(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateNormalDistance(triangle.m_triangleIndex);
}

glm::vec3 BillboardCloud::CalculateNormal(const ClusterTriangle& triangle) const
{
	return m_elements.at(triangle.m_elementIndex).CalculateNormal(triangle.m_triangleIndex);
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

struct CPUDepthBuffer
{
	std::vector<float> m_depthBuffer;
	std::vector<std::mutex> m_pixelLocks;
	int m_width = 0;
	int m_height = 0;
	CPUDepthBuffer(const size_t width, const size_t height)
	{
		m_depthBuffer = std::vector<float>(width * height);
		Reset();
		m_pixelLocks = std::vector<std::mutex>(width * height);
		m_width = width;
		m_height = height;
	}

	void Reset()
	{
		std::fill(m_depthBuffer.begin(), m_depthBuffer.end(), -FLT_MAX);
	}

	[[nodiscard]] bool CompareZ(const int u, const int v, const float z) const
	{
		if (u < 0 || v < 0 || u > m_width - 1 || v > m_height - 1)
			return false;

		const int uv = u + m_width * v;
		return z >= m_depthBuffer[uv];
	}
};

template<typename T>
struct CPUColorBuffer
{
	std::vector<T> m_colorBuffer;
	std::vector<std::mutex> m_pixelLocks;
	int m_width = 0;
	int m_height = 0;
	CPUColorBuffer(const size_t width, const size_t height)
	{
		m_colorBuffer = std::vector<T>(width * height);
		std::fill(m_colorBuffer.begin(), m_colorBuffer.end(), T(0.f));
		m_pixelLocks = std::vector<std::mutex>(width * height);
		m_width = width;
		m_height = height;
	}

	void FillColor(const T& val)
	{
		std::fill(m_colorBuffer.begin(), m_colorBuffer.end(), val);
	}

	void SetPixel(const int u, const int v, const T& color)
	{
		if (u < 0 || v < 0 || u > m_width - 1 || v > m_height - 1)
			return;

		const int uv = u + m_width * v;
		std::lock_guard lock(m_pixelLocks[uv]);
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
	if (den == 0.f)
	{
		c1 = c2 = 0.f;
		c0 = 1.f;
	}
	else {
		c1 = (v2.x * v1.y - v1.x * v2.y) / den;
		c2 = (v0.x * v2.y - v2.x * v0.y) / den;
		c0 = 1.0f - c1 - c2;
	}
}
inline float Area(const glm::vec2& a, const glm::vec2& b) { return a.x * b.y - a.y * b.x; }

inline void Barycentric2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c, const glm::vec2& d, float& c0, float& c1, float& c2, float& c3)
{
	float r[4], t[4], u[4];
	const glm::vec2 v[4] = { a, b, c, d };
	glm::vec2 s[4];
	for (int i = 0; i < 4; i++) {
		s[i] = v[i] - p;
		r[i] = length(s[i]);
	}
	for (int i = 0; i < 4; i++) {
		const float area = Area(s[i], s[(i + 1) % 4]);
		const float dotResult = glm::dot(s[i], s[(i + 1) % 4]);
		if (area == 0.f) t[i] = 0.f;
		else {
			t[i] = (r[i] * r[(i + 1) % 4] - dotResult) / area;
		}
	}
	for (int i = 0; i < 4; i++) {
		if (r[i] == 0.f) u[i] = 0.f;
		else u[i] = (t[(i + 3) % 4] + t[i]) / r[i];
	}
	const auto sum = u[0] + u[1] + u[2] + u[3];
	assert(sum != 0.f);
	c0 = u[0] / sum;
	c1 = u[1] / sum;
	c2 = u[2] / sum;
	c3 = u[3] / sum;
}

void BillboardCloud::Project(const ProjectSettings& projectSettings)
{
	for (auto& cluster : m_clusters) Project(cluster, projectSettings);
}

void BillboardCloud::Join(const JoinSettings& joinSettings)
{
	xatlas::Atlas* atlas = xatlas::Create();

	for (const auto& cluster : m_clusters)
	{
		xatlas::MeshDecl meshDecl;
		meshDecl.vertexCount = cluster.m_billboardVertices.size();
		meshDecl.vertexPositionData = cluster.m_billboardVertices.data();
		meshDecl.vertexPositionStride = sizeof(Vertex);
		meshDecl.indexCount = cluster.m_billboardTriangles.size() * 3;
		meshDecl.indexData = cluster.m_billboardTriangles.data();
		meshDecl.indexFormat = xatlas::IndexFormat::UInt32;
		xatlas::AddMeshError error = xatlas::AddMesh(atlas, meshDecl, 1);
		if (error != xatlas::AddMeshError::Success) {
			EVOENGINE_ERROR("Error adding mesh!");
			break;
		}
	}
	xatlas::AddMeshJoin(atlas);
	xatlas::Generate(atlas);
	std::vector<Vertex> billboardCloudVertices;
	billboardCloudVertices.resize(m_clusters.size() * 4);
	std::vector<glm::uvec3> billboardCloudTriangles;
	billboardCloudTriangles.resize(m_clusters.size() * 2);
	Jobs::RunParallelFor(m_clusters.size(), [&](const unsigned clusterIndex)
		{
			const xatlas::Mesh& mesh = atlas->meshes[clusterIndex];
			auto& cluster = m_clusters[clusterIndex];
			cluster.m_rectangle.m_texCoords[0].x = mesh.vertexArray[0].uv[0] / static_cast<float>(atlas->width);
			cluster.m_rectangle.m_texCoords[0].y = mesh.vertexArray[0].uv[1] / static_cast<float>(atlas->height);
			cluster.m_rectangle.m_texCoords[1].x = mesh.vertexArray[1].uv[0] / static_cast<float>(atlas->width);
			cluster.m_rectangle.m_texCoords[1].y = mesh.vertexArray[1].uv[1] / static_cast<float>(atlas->height);
			cluster.m_rectangle.m_texCoords[2].x = mesh.vertexArray[2].uv[0] / static_cast<float>(atlas->width);
			cluster.m_rectangle.m_texCoords[2].y = mesh.vertexArray[2].uv[1] / static_cast<float>(atlas->height);
			cluster.m_rectangle.m_texCoords[3].x = mesh.vertexArray[3].uv[0] / static_cast<float>(atlas->width);
			cluster.m_rectangle.m_texCoords[3].y = mesh.vertexArray[3].uv[1] / static_cast<float>(atlas->height);

			billboardCloudVertices[4 * clusterIndex] = cluster.m_billboardVertices[0];
			billboardCloudVertices[4 * clusterIndex + 1] = cluster.m_billboardVertices[1];
			billboardCloudVertices[4 * clusterIndex + 2] = cluster.m_billboardVertices[2];
			billboardCloudVertices[4 * clusterIndex + 3] = cluster.m_billboardVertices[3];

			billboardCloudVertices[4 * clusterIndex].m_texCoord = cluster.m_rectangle.m_texCoords[0];
			billboardCloudVertices[4 * clusterIndex + 1].m_texCoord = cluster.m_rectangle.m_texCoords[1];
			billboardCloudVertices[4 * clusterIndex + 2].m_texCoord = cluster.m_rectangle.m_texCoords[2];
			billboardCloudVertices[4 * clusterIndex + 3].m_texCoord = cluster.m_rectangle.m_texCoords[3];

			billboardCloudTriangles[2 * clusterIndex] = cluster.m_billboardTriangles[0] + glm::uvec3(clusterIndex * 4);
			billboardCloudTriangles[2 * clusterIndex + 1] = cluster.m_billboardTriangles[1] + glm::uvec3(clusterIndex * 4);
		});

	m_billboardCloudMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	m_billboardCloudMesh->SetVertices({ false, false, true, false }, billboardCloudVertices, billboardCloudTriangles);
	xatlas::Destroy(atlas);
}
struct PBRMaterial
{
	glm::vec4 m_baseAlbedo = glm::vec4(1.f);
	float m_baseRoughness = 0.3f;
	float m_baseMetallic = 0.3f;
	float m_baseAo = 1.f;
	glm::ivec2 m_albedoTextureResolution = glm::ivec2(-1);
	std::vector<glm::vec4> m_albedoTextureData;

	glm::ivec2 m_normalTextureResolution = glm::ivec2(-1);
	std::vector<glm::vec3> m_normalTextureData;

	glm::ivec2 m_roughnessTextureResolution = glm::ivec2(-1);
	std::vector<float> m_roughnessTextureData;

	glm::ivec2 m_metallicTextureResolution = glm::ivec2(-1);
	std::vector<float> m_metallicTextureData;

	glm::ivec2 m_aoTextureResolution = glm::ivec2(-1);
	std::vector<float> m_aoTextureData;
	void Clear();
	void ApplyMaterial(const std::shared_ptr<Material>& material, const BillboardCloud::RasterizeSettings& rasterizeSettings);
};
void PBRMaterial::Clear()
{
	m_baseAlbedo = glm::vec4(1.f);
	m_baseRoughness = 0.3f;
	m_baseMetallic = 0.3f;
	m_baseAo = 1.f;
	m_albedoTextureResolution = glm::ivec2(-1);
	m_normalTextureResolution = glm::ivec2(-1);
	m_roughnessTextureResolution = glm::ivec2(-1);
	m_metallicTextureResolution = glm::ivec2(-1);
	m_aoTextureResolution = glm::ivec2(-1);

	m_albedoTextureData.clear();
	m_normalTextureData.clear();
	m_roughnessTextureData.clear();
	m_metallicTextureData.clear();
	m_aoTextureData.clear();
}

void PBRMaterial::ApplyMaterial(const std::shared_ptr<Material>& material,
	const BillboardCloud::RasterizeSettings& rasterizeSettings)
{
	m_baseAlbedo = glm::vec4(material->m_materialProperties.m_albedoColor, 1.f - material->m_materialProperties.m_transmission);
	m_baseRoughness = material->m_materialProperties.m_roughness;
	m_baseMetallic = material->m_materialProperties.m_metallic;
	m_baseAo = 1.f;

	const auto albedoTexture = material->GetAlbedoTexture();
	if (rasterizeSettings.m_transferAlbedoMap && albedoTexture)
	{
		albedoTexture->GetRgbaChannelData(m_albedoTextureData);
		m_albedoTextureResolution = albedoTexture->GetResolution();
	}
	const auto normalTexture = material->GetNormalTexture();
	if (rasterizeSettings.m_transferNormalMap && normalTexture)
	{
		normalTexture->GetRgbChannelData(m_normalTextureData);
		m_normalTextureResolution = normalTexture->GetResolution();
	}
	const auto roughnessTexture = material->GetRoughnessTexture();
	if (rasterizeSettings.m_transferRoughnessMap && roughnessTexture)
	{
		roughnessTexture->GetRedChannelData(m_roughnessTextureData);
		m_roughnessTextureResolution = roughnessTexture->GetResolution();
	}
	const auto metallicTexture = material->GetMetallicTexture();
	if (rasterizeSettings.m_transferMetallicMap && metallicTexture)
	{
		metallicTexture->GetRedChannelData(m_metallicTextureData);
		m_metallicTextureResolution = metallicTexture->GetResolution();
	}
	const auto aoTexture = material->GetAoTexture();
	if (rasterizeSettings.m_transferAoMap && aoTexture)
	{
		aoTexture->GetRedChannelData(m_aoTextureData);
		m_aoTextureResolution = aoTexture->GetResolution();
	}
}

void BillboardCloud::Rasterize(const RasterizeSettings& rasterizeSettings)
{
	if (rasterizeSettings.m_resolution.x < 1 || rasterizeSettings.m_resolution.y < 1) return;
	std::unordered_map<Handle, PBRMaterial> pbrMaterials;
	float averageRoughness = 0.f;
	float averageMetallic = 0.0f;
	float averageAo = 0.f;
	for (auto& element : m_elements)
	{
		const auto& material = element.m_material;
		auto materialHandle = material->GetHandle();
		pbrMaterials[materialHandle].ApplyMaterial(material, rasterizeSettings);

		averageRoughness += material->m_materialProperties.m_roughness;
		averageMetallic += material->m_materialProperties.m_metallic;
		averageAo += 1.f;
	}
	averageRoughness /= m_elements.size();
	averageMetallic /= m_elements.size();
	averageAo /= m_elements.size();

	CPUDepthBuffer depthBuffer(rasterizeSettings.m_resolution.x, rasterizeSettings.m_resolution.y);

	CPUColorBuffer<glm::vec4> albedoFrameBuffer(rasterizeSettings.m_resolution.x, rasterizeSettings.m_resolution.y);
	CPUColorBuffer<glm::vec3> normalFrameBuffer(rasterizeSettings.m_resolution.x, rasterizeSettings.m_resolution.y);
	CPUColorBuffer<float> roughnessFrameBuffer(rasterizeSettings.m_resolution.x, rasterizeSettings.m_resolution.y);
	CPUColorBuffer<float> metallicFrameBuffer(rasterizeSettings.m_resolution.x, rasterizeSettings.m_resolution.y);
	CPUColorBuffer<float> aoFrameBuffer(rasterizeSettings.m_resolution.x, rasterizeSettings.m_resolution.y);

	if (rasterizeSettings.m_debugFullFill) albedoFrameBuffer.FillColor(glm::vec4(1.f));

	if (rasterizeSettings.m_debugOpaque)
	{
		averageRoughness = 1.f;
		averageMetallic = 0.f;
		averageAo = 1.f;
		Jobs::RunParallelFor(m_clusters.size(), [&](const unsigned clusterIndex)
			{
				const auto& cluster = m_clusters[clusterIndex];
				glm::vec3 color = glm::ballRand(1.f);
				const auto& boundingRectangle = cluster.m_rectangle;

				for (int triangleIndex = 0; triangleIndex < 2; triangleIndex++) {

					glm::vec3 textureSpaceVertices[3];
					if (triangleIndex == 0) {
						textureSpaceVertices[0] = glm::vec3(boundingRectangle.m_texCoords[0] * glm::vec2(rasterizeSettings.m_resolution), 0.f);
						textureSpaceVertices[1] = glm::vec3(boundingRectangle.m_texCoords[1] * glm::vec2(rasterizeSettings.m_resolution), 0.f);
						textureSpaceVertices[2] = glm::vec3(boundingRectangle.m_texCoords[2] * glm::vec2(rasterizeSettings.m_resolution), 0.f);
					}
					else
					{
						textureSpaceVertices[0] = glm::vec3(boundingRectangle.m_texCoords[2] * glm::vec2(rasterizeSettings.m_resolution), 0.f);
						textureSpaceVertices[1] = glm::vec3(boundingRectangle.m_texCoords[3] * glm::vec2(rasterizeSettings.m_resolution), 0.f);
						textureSpaceVertices[2] = glm::vec3(boundingRectangle.m_texCoords[0] * glm::vec2(rasterizeSettings.m_resolution), 0.f);
					}
					//Bound check;
					auto minBound = glm::vec2(FLT_MAX, FLT_MAX);
					auto maxBound = glm::vec2(-FLT_MAX, -FLT_MAX);
					for (const auto& textureSpaceVertex : textureSpaceVertices)
					{
						minBound = glm::min(glm::vec2(textureSpaceVertex), minBound);
						maxBound = glm::max(glm::vec2(textureSpaceVertex), maxBound);
					}

					const auto left = static_cast<int>(minBound.x - 0.5f);
					const auto right = static_cast<int>(maxBound.x + 0.5f);
					const auto top = static_cast<int>(minBound.y - 0.5f);
					const auto bottom = static_cast<int>(maxBound.y + 0.5f);
					for (auto u = left; u <= right; u++)
					{
						for (auto v = top; v <= bottom; v++)
						{
							const auto p = glm::vec3(u + .5f, v + .5f, 0.f);
							float bc0, bc1, bc2;
							Barycentric2D(p, textureSpaceVertices[0], textureSpaceVertices[1], textureSpaceVertices[2], bc0, bc1, bc2);
							if (bc0 < 0.f || bc1 < 0.f || bc2 < 0.f) continue;
							auto albedo = glm::vec4(color, 1.f);
							albedoFrameBuffer.SetPixel(u, v, albedo);
						}
					}
				}
			}
		);
	}

	for (auto& cluster : m_clusters) {

		//Calculate texture size on atlas
		const auto& boundingRectangle = cluster.m_rectangle;

		//Rasterization
		Jobs::RunParallelFor(cluster.m_projectedTriangles.size(), [&](const unsigned triangleIndex)
			{
				const auto& triangle = cluster.m_projectedTriangles[triangleIndex];
				const auto& v0 = triangle.m_projectedVertices[0];
				const auto& v1 = triangle.m_projectedVertices[1];
				const auto& v2 = triangle.m_projectedVertices[2];
				const auto& material = pbrMaterials.at(triangle.m_materialHandle);

				glm::vec3 textureSpaceVertices[3];
				for (int i = 0; i < 3; i++)
				{
					const auto p = glm::vec2(triangle.m_projectedVertices[i].m_position.x, triangle.m_projectedVertices[i].m_position.y);
					const auto r0 = boundingRectangle.m_points[0];
					const auto r1 = boundingRectangle.m_points[1];
					const auto r2 = boundingRectangle.m_points[2];
					const auto r3 = boundingRectangle.m_points[3];

					float bc0, bc1, bc2, bc3;
					Barycentric2D(p, r0, r1, r2, r3, bc0, bc1, bc2, bc3);
					const auto textureSpacePosition = (boundingRectangle.m_texCoords[0] * bc0 + boundingRectangle.m_texCoords[1] * bc1 + boundingRectangle.m_texCoords[2] * bc2 + boundingRectangle.m_texCoords[3] * bc3) * glm::vec2(rasterizeSettings.m_resolution);
					textureSpaceVertices[i].x = textureSpacePosition.x;
					textureSpaceVertices[i].y = textureSpacePosition.y;
					textureSpaceVertices[i].z = triangle.m_projectedVertices[i].m_position.z;
				}

				//Bound check;
				auto minBound = glm::vec2(FLT_MAX, FLT_MAX);
				auto maxBound = glm::vec2(-FLT_MAX, -FLT_MAX);
				for (const auto& textureSpaceVertex : textureSpaceVertices)
				{
					minBound = glm::min(glm::vec2(textureSpaceVertex), minBound);
					maxBound = glm::max(glm::vec2(textureSpaceVertex), maxBound);
				}

				const auto left = static_cast<int>(minBound.x - 0.5f);
				const auto right = static_cast<int>(maxBound.x + 0.5f);
				const auto top = static_cast<int>(minBound.y - 0.5f);
				const auto bottom = static_cast<int>(maxBound.y + 0.5f);
				for (auto u = left; u <= right; u++)
				{
					for (auto v = top; v <= bottom; v++)
					{
						const auto p = glm::vec3(u + .5f, v + .5f, 0.f);
						float bc0, bc1, bc2;
						Barycentric2D(p, textureSpaceVertices[0], textureSpaceVertices[1], textureSpaceVertices[2], bc0, bc1, bc2);
						if (bc0 < 0.f || bc1 < 0.f || bc2 < 0.f) continue;
						float z = bc0 * v0.m_position.z + bc1 * v1.m_position.z + bc2 * v2.m_position.z;
						//Early depth check.
						if (!depthBuffer.CompareZ(u, v, z)) continue;

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
						albedoFrameBuffer.SetPixel(u, v, albedo);
						normal = normal * 0.5f + glm::vec3(0.5f);
						normalFrameBuffer.SetPixel(u, v, normal);
						roughnessFrameBuffer.SetPixel(u, v, roughness);
						metallicFrameBuffer.SetPixel(u, v, metallic);
						aoFrameBuffer.SetPixel(u, v, ao);
					}
				}
			});
	}

	m_billboardCloudMaterial = ProjectManager::CreateTemporaryAsset<Material>();
	std::shared_ptr<Texture2D> albedoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
	albedoTexture->SetRgbaChannelData(albedoFrameBuffer.m_colorBuffer, glm::uvec2(albedoFrameBuffer.m_width, albedoFrameBuffer.m_height));
	m_billboardCloudMaterial->SetAlbedoTexture(albedoTexture);
	if (rasterizeSettings.m_transferNormalMap) {
		std::shared_ptr<Texture2D> normalTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		normalTexture->SetRgbChannelData(normalFrameBuffer.m_colorBuffer, glm::uvec2(normalFrameBuffer.m_width, normalFrameBuffer.m_height));
		m_billboardCloudMaterial->SetNormalTexture(normalTexture);
	}
	if (rasterizeSettings.m_transferRoughnessMap) {
		std::shared_ptr<Texture2D> roughnessTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		roughnessTexture->SetRedChannelData(roughnessFrameBuffer.m_colorBuffer, glm::uvec2(roughnessFrameBuffer.m_width, roughnessFrameBuffer.m_height));
		m_billboardCloudMaterial->SetRoughnessTexture(roughnessTexture);
	}
	else
	{
		m_billboardCloudMaterial->m_materialProperties.m_roughness = averageRoughness;
	}
	if (rasterizeSettings.m_transferMetallicMap) {
		std::shared_ptr<Texture2D> metallicTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		metallicTexture->SetRedChannelData(metallicFrameBuffer.m_colorBuffer, glm::uvec2(metallicFrameBuffer.m_width, metallicFrameBuffer.m_height));
		m_billboardCloudMaterial->SetMetallicTexture(metallicTexture);
	}
	else
	{
		m_billboardCloudMaterial->m_materialProperties.m_metallic = averageMetallic;
	}
	if (rasterizeSettings.m_transferAoMap) {
		std::shared_ptr<Texture2D> aoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
		aoTexture->SetRedChannelData(aoFrameBuffer.m_colorBuffer, glm::uvec2(aoFrameBuffer.m_width, aoFrameBuffer.m_height));
		m_billboardCloudMaterial->SetAOTexture(aoTexture);
	}
}

void BillboardCloud::Generate(const GenerateSettings& generateSettings)
{
	Clusterize(generateSettings.m_clusterizationSettings);
	Project(generateSettings.m_projectSettings);
	Join(generateSettings.m_joinSettings);
	Rasterize(generateSettings.m_rasterizeSettings);
}

void BillboardCloud::Project(Cluster& cluster, const ProjectSettings& projectSettings) const
{
	const auto billboardFrontAxis = cluster.m_clusterPlane.GetNormal();
	auto billboardUpAxis = glm::vec3(billboardFrontAxis.y, billboardFrontAxis.z, billboardFrontAxis.x); //cluster.m_planeYAxis;
	const auto billboardLeftAxis = glm::normalize(glm::cross(billboardFrontAxis, billboardUpAxis));
	billboardUpAxis = glm::normalize(glm::cross(billboardLeftAxis, billboardFrontAxis));
	glm::mat4 rotateMatrix = glm::transpose(glm::mat4(glm::vec4(billboardLeftAxis, 0.0f), glm::vec4(billboardUpAxis, 0.0f), glm::vec4(billboardFrontAxis, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
	cluster.m_projectedTriangles.resize(cluster.m_triangles.size());
	Jobs::RunParallelFor(cluster.m_triangles.size(), [&](const unsigned triangleIndex)
		{
			const auto& clusterTriangle = cluster.m_triangles.at(triangleIndex);
			auto& projectedTriangle = cluster.m_projectedTriangles[triangleIndex];
			const auto& element = m_elements.at(clusterTriangle.m_elementIndex);
			const auto& vertices = element.m_vertices;
			const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
			auto& v0 = vertices.at(triangle.x);
			auto& v1 = vertices.at(triangle.y);
			auto& v2 = vertices.at(triangle.z);

			auto& pV0 = projectedTriangle.m_projectedVertices[0];
			auto& pV1 = projectedTriangle.m_projectedVertices[1];
			auto& pV2 = projectedTriangle.m_projectedVertices[2];

			TransformVertex(v0, pV0, rotateMatrix);
			TransformVertex(v1, pV1, rotateMatrix);
			TransformVertex(v2, pV2, rotateMatrix);

			projectedTriangle.m_materialHandle = element.m_material->GetHandle();
		});

	std::vector<glm::vec2> points;
	points.resize(cluster.m_projectedTriangles.size() * 3);
	Jobs::RunParallelFor(cluster.m_projectedTriangles.size(), [&](unsigned triangleIndex)
		{
			const auto& projectedTriangle = cluster.m_projectedTriangles.at(triangleIndex);
			points.at(triangleIndex * 3) = glm::vec2(projectedTriangle.m_projectedVertices[0].m_position.x, projectedTriangle.m_projectedVertices[0].m_position.y);
			points.at(triangleIndex * 3 + 1) = glm::vec2(projectedTriangle.m_projectedVertices[1].m_position.x, projectedTriangle.m_projectedVertices[1].m_position.y);
			points.at(triangleIndex * 3 + 2) = glm::vec2(projectedTriangle.m_projectedVertices[2].m_position.x, projectedTriangle.m_projectedVertices[2].m_position.y);
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
	//Generate billboard mesh
	cluster.m_billboardVertices.resize(4);
	const auto inverseRotateMatrix = glm::inverse(rotateMatrix);
	cluster.m_billboardVertices[0].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[0].x, cluster.m_rectangle.m_points[0].y, 0.f, 0.f);
	cluster.m_rectangle.m_texCoords[0] = cluster.m_billboardVertices[0].m_texCoord = glm::vec2(0, 0);
	cluster.m_billboardVertices[1].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[1].x, cluster.m_rectangle.m_points[1].y, 0.f, 0.f);
	cluster.m_rectangle.m_texCoords[1] = cluster.m_billboardVertices[1].m_texCoord = glm::vec2(1, 0);
	cluster.m_billboardVertices[2].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[2].x, cluster.m_rectangle.m_points[2].y, 0.f, 0.f);
	cluster.m_rectangle.m_texCoords[2] = cluster.m_billboardVertices[2].m_texCoord = glm::vec2(1, 1);
	cluster.m_billboardVertices[3].m_position = inverseRotateMatrix * glm::vec4(cluster.m_rectangle.m_points[3].x, cluster.m_rectangle.m_points[3].y, 0.f, 0.f);
	cluster.m_rectangle.m_texCoords[3] = cluster.m_billboardVertices[3].m_texCoord = glm::vec2(0, 1);

	cluster.m_billboardVertices[0].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;
	cluster.m_billboardVertices[1].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;
	cluster.m_billboardVertices[2].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;
	cluster.m_billboardVertices[3].m_position -= cluster.m_clusterPlane.m_d * billboardFrontAxis;

	cluster.m_billboardTriangles.resize(2);
	cluster.m_billboardTriangles[0] = { 0, 1, 2 };
	cluster.m_billboardTriangles[1] = { 2, 3, 0 };
}

#pragma endregion
#pragma region IO
Entity BillboardCloud::BuildEntity(const std::shared_ptr<Scene>& scene) const
{
	if (!m_billboardCloudMesh || !m_billboardCloudMaterial) return {};
	const auto owner = scene->CreateEntity("Billboard Cloud");

	const auto projectedElementEntity = scene->CreateEntity("Billboard Cloud");
	const auto elementMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(projectedElementEntity).lock();
	elementMeshRenderer->m_mesh = m_billboardCloudMesh;
	elementMeshRenderer->m_material = m_billboardCloudMaterial;
	scene->SetParent(projectedElementEntity, owner);


	return owner;
}

void BillboardCloud::ProcessPrefab(const std::shared_ptr<Prefab>& currentPrefab,
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
				m_elements.emplace_back();
				auto& element = m_elements.back();
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
		ProcessPrefab(childPrefab, transform);
	}
}

#pragma endregion
#pragma region Clusterization
void BillboardCloud::ProcessPrefab(const std::shared_ptr<Prefab>& prefab)
{
	ProcessPrefab(prefab, Transform());
}

void BillboardCloud::Clusterize(const ClusterizationSettings& clusterizeSettings)
{
	m_clusters.clear();
	switch (clusterizeSettings.m_clusterizeMode)
	{
	case static_cast<unsigned>(ClusterizationMode::FlipBook):
	{
		m_clusters.emplace_back();
		auto& cluster = m_clusters.back();
		cluster.m_triangles = CollectTriangles();
	}
	break;
	case static_cast<unsigned>(ClusterizationMode::Foliage):
	{
		std::vector<ClusterTriangle> operatingTriangles = CollectTriangles();
		m_clusters = StochasticClusterize(std::move(operatingTriangles), clusterizeSettings);
	}
	break;
	case static_cast<unsigned>(ClusterizationMode::Original):
	{
		std::vector<ClusterTriangle> operatingTriangles = CollectTriangles();
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
		if (roMinPMaxN < m_roMin) false;

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
					if (!ComputeRoMinMax(roMaxMin, element, clusterTriangle,
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
			if (!ComputeRoMinMax(roMinMax, element, clusterTriangle, bin.m_thetaMin, bin.m_thetaMax, bin.m_phiMin, bin.m_phiMax))
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
			if (!ComputeRoMinMax(roMinMax, element, clusterTriangle, bin.m_thetaMin, bin.m_thetaMax, bin.m_phiMin, bin.m_phiMax))
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

				if(settings.m_fillBand)
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

		retVal.emplace_back(std::move(newCluster));

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


std::vector<BillboardCloud::Cluster> BillboardCloud::DefaultClusterize(
	std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings)
{
	BoundingSphere boundingSphere;
	boundingSphere.Initialize(m_elements);
	const auto& settings = clusterizeSettings.m_originalClusterizationSettings;
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

	Discretization discretization(maxNormalDistance, epsilon, settings.m_discretizationSize, settings.m_discretizationSize, roNum);
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
		if (settings.m_timeout != 0 && epoch >= settings.m_timeout)
		{
			EVOENGINE_ERROR("Default clustering timeout!")
				break;
		}
	}

	return retVal;
}
#pragma endregion

bool BillboardCloud::OriginalClusterizationSettings::OnInspect()
{
	bool changed = false;
	if (ImGui::TreeNode("Original clusterization settings"))
	{
		if (ImGui::DragFloat("Epsilon percentage", &m_epsilonPercentage, 0.01f, 0.01f, 1.f)) changed = true;
		if (ImGui::DragInt("Discretization size", &m_discretizationSize, 1, 1, 1000)) changed = true;
		if (ImGui::DragInt("Timeout", &m_timeout, 1, 1, 1000)) changed = true;
		ImGui::TreePop();
	}
	return changed;
}

bool BillboardCloud::FoliageClusterizationSettings::OnInspect()
{
	bool changed = false;
	if (ImGui::TreeNode("Foliage clusterization settings"))
	{
		if (ImGui::DragFloat("Complexity", &m_density, 0.01f, 0.0f, 0.95f)) changed = true;
		if (ImGui::DragInt("Iteration", &m_iteration, 1, 1, 1000)) changed = true;
		if (ImGui::DragInt("Timeout", &m_timeout, 1, 1, 1000)) changed = true;
		if (ImGui::DragFloat("Density", &m_sampleRange, 0.01f, 0.1f, 2.f)) changed = true;

		if(ImGui::Checkbox("Fill band", &m_fillBand)) changed = true;
		ImGui::TreePop();
	}
	return changed;
}

bool BillboardCloud::ClusterizationSettings::OnInspect()
{
	bool changed = false;

	if (ImGui::TreeNode("Clusterization settings"))
	{
		if (ImGui::Combo("Clusterize mode",
			{ "FlipBook", "Original", "Foliage" },
			m_clusterizeMode)) {
			changed = true;
		}
		switch (m_clusterizeMode)
		{
		case static_cast<unsigned>(ClusterizationMode::FlipBook): break;
		case static_cast<unsigned>(ClusterizationMode::Foliage):
		{
			if (m_foliageClusterizationSettings.OnInspect()) changed = true;
		}
		break;
		case static_cast<unsigned>(ClusterizationMode::Original):
		{
			if (m_originalClusterizationSettings.OnInspect()) changed = true;
		}
		break;
		}
		ImGui::TreePop();
	}
	return changed;
}

bool BillboardCloud::ProjectSettings::OnInspect()
{
	bool changed = false;
	if (ImGui::TreeNode("Project settings"))
	{
		ImGui::TreePop();
	}
	return changed;
}

bool BillboardCloud::JoinSettings::OnInspect()
{
	bool changed = false;
	if (ImGui::TreeNode("Join settings"))
	{
		ImGui::TreePop();
	}
	return changed;
}

bool BillboardCloud::RasterizeSettings::OnInspect()
{
	bool changed = false;
	if (ImGui::TreeNode("Rasterize settings"))
	{
		if (ImGui::Checkbox("(Debug) Full Fill", &m_debugFullFill)) changed = true;
		if (ImGui::Checkbox("(Debug) Opaque", &m_debugOpaque)) changed = true;
		if (ImGui::Checkbox("Transfer albedo texture", &m_transferAlbedoMap)) changed = true;
		if (ImGui::Checkbox("Transfer normal texture", &m_transferNormalMap)) changed = true;
		if (ImGui::Checkbox("Transfer roughness texture", &m_transferRoughnessMap)) changed = true;
		if (ImGui::Checkbox("Transfer metallic texture", &m_transferMetallicMap)) changed = true;
		if (ImGui::Checkbox("Transfer ao texture", &m_transferAoMap)) changed = true;
		if (ImGui::DragInt2("Resolution", &m_resolution.x, 1, 1, 8192)) changed = true;
		ImGui::TreePop();
	}
	return changed;
}



bool BillboardCloud::GenerateSettings::OnInspect(const std::string& title)
{
	bool changed = false;
	if (ImGui::TreeNodeEx(title.c_str()))
	{
		if (m_clusterizationSettings.OnInspect()) changed = true;
		if (m_projectSettings.OnInspect()) changed = true;
		if (m_joinSettings.OnInspect()) changed = true;
		if (m_rasterizeSettings.OnInspect()) changed = true;
		ImGui::TreePop();
	}
	return changed;
}


