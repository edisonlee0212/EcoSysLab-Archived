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


void BillboardCloud::BoundingSphere::Initialize(const std::vector<BillboardCloud::Element>& elements)
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


std::vector<glm::vec3> BillboardCloud::ExtractPointCloud(const float density) const
{
	std::vector<glm::vec3> points;
	BoundingSphere boundingSphere;
	boundingSphere.Initialize(m_elements);

	const auto div = glm::pow(boundingSphere.m_radius * density, 2.f);
	for (const auto& element : m_elements)
	{
		std::unordered_set<unsigned> selectedIndices;
		for (const auto& triangle : element.m_triangles)
		{
			/*
			if(selectedIndices.find(triangle.x) == selectedIndices.end())
			{
				points.emplace_back(element.m_vertices[triangle.x].m_position);
				selectedIndices.emplace(triangle.x);
			}else if(selectedIndices.find(triangle.y) == selectedIndices.end())
			{
				points.emplace_back(element.m_vertices[triangle.y].m_position);
				selectedIndices.emplace(triangle.y);
			}else if(selectedIndices.find(triangle.z) == selectedIndices.end())
			{
				points.emplace_back(element.m_vertices[triangle.z].m_position);
				selectedIndices.emplace(triangle.z);
			}*/

			const auto& v0 = element.m_vertices[triangle.x].m_position;
			const auto& v1 = element.m_vertices[triangle.y].m_position;
			const auto& v2 = element.m_vertices[triangle.z].m_position;
			const auto area = element.CalculateArea(triangle);
			int pointCount = glm::max(static_cast<int>(area / div), 1);
			for (int i = 0; i < pointCount; i++) {
				float a = glm::linearRand(0.f, 1.f);
				float b = glm::linearRand(0.f, 1.f);
				if (a + b >= 1.f) {
					a = 1.f - a;
					b = 1.f - b;
				}
				const auto point = v0 + a * (v1 - v0) + b * (v2 - v0);
				points.emplace_back(point);
			}
		}
	}

	return points;
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

void BillboardCloud::ProcessEntity(const std::shared_ptr<Scene>& scene, const Entity& entity,
	const Transform& parentModelSpaceTransform)
{
	Transform transform = scene->GetDataComponent<Transform>(entity);
	transform.m_value = parentModelSpaceTransform.m_value * transform.m_value;
	if (scene->HasPrivateComponent<MeshRenderer>(entity))
	{
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
		const auto mesh = meshRenderer->m_mesh.Get<Mesh>();
		const auto material = meshRenderer->m_material.Get<Material>();
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

	for (const auto& childEntity : scene->GetChildren(entity))
	{
		ProcessEntity(scene, childEntity, transform);
	}
}

#pragma endregion
#pragma region Clusterization
void BillboardCloud::Process(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material)
{
	m_elements.emplace_back();
	auto& element = m_elements.back();
	element.m_vertices = mesh->UnsafeGetVertices();
	element.m_material = material;
	element.m_triangles = mesh->UnsafeGetTriangles();
}


void BillboardCloud::Process(const std::shared_ptr<Prefab>& prefab)
{
	ProcessPrefab(prefab, Transform());
}

void BillboardCloud::Process(const std::shared_ptr<Scene>& scene, const Entity& entity)
{
	ProcessEntity(scene, entity, Transform());
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



#pragma endregion

bool BillboardCloud::OriginalClusterizationSettings::OnInspect()
{
	bool changed = false;
	if (ImGui::TreeNode("Original clusterization settings"))
	{
		if (ImGui::DragFloat("Epsilon percentage", &m_epsilonPercentage, 0.01f, 0.01f, 1.f)) changed = true;
		if (ImGui::DragInt("Discretization size", &m_discretizationSize, 1, 1, 1000)) changed = true;
		if (ImGui::DragInt("Timeout", &m_timeout, 1, 1, 1000)) changed = true;

		ImGui::Checkbox("Skip remaining triangles", &m_skipRemainTriangles);
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

		if (ImGui::Checkbox("Fill band", &m_fillBand)) changed = true;
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


