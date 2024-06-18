#include "BillboardCloud.hpp"
#include "Prefab.hpp"
#include "xatlas/xatlas.h"
using namespace evo_engine;
#pragma region Projection

std::vector<BillboardCloud::ClusterTriangle> BillboardCloud::CollectTriangles() const {
  std::vector<ClusterTriangle> ret_val;
  for (int element_index = 0; element_index < elements.size(); element_index++) {
    const auto& element = elements.at(element_index);
    for (int triangle_index = 0; triangle_index < element.triangles.size(); triangle_index++) {
      ClusterTriangle cluster_triangle;
      cluster_triangle.element_index = element_index;
      cluster_triangle.triangle_index = triangle_index;
      ret_val.emplace_back(cluster_triangle);
    }
  }
  return ret_val;
}

glm::vec3 BillboardCloud::CalculateCentroid(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateCentroid(triangle.triangle_index);
}

float BillboardCloud::CalculateArea(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateArea(triangle.triangle_index);
}

float BillboardCloud::CalculateNormalDistance(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateNormalDistance(triangle.triangle_index);
}

glm::vec3 BillboardCloud::CalculateNormal(const ClusterTriangle& triangle) const {
  return elements.at(triangle.element_index).CalculateNormal(triangle.triangle_index);
}

inline void TransformVertex(const Vertex& v, Vertex& tV, const glm::mat4& transform) {
  tV = v;
  tV.normal = glm::normalize(transform * glm::vec4(v.normal, 0.f));
  tV.tangent = glm::normalize(transform * glm::vec4(v.tangent, 0.f));
  tV.position = transform * glm::vec4(v.position, 1.f);
}

inline void TransformVertex(Vertex& v, const glm::mat4& transform) {
  v.normal = glm::normalize(transform * glm::vec4(v.normal, 0.f));
  v.tangent = glm::normalize(transform * glm::vec4(v.tangent, 0.f));
  v.position = transform * glm::vec4(v.position, 1.f);
}

glm::vec3 BillboardCloud::Element::CalculateCentroid(const int triangle_index) const {
  return CalculateCentroid(triangles.at(triangle_index));
}

float BillboardCloud::Element::CalculateArea(const int triangle_index) const {
  return CalculateArea(triangles.at(triangle_index));
}

float BillboardCloud::Element::CalculateNormalDistance(const int triangle_index) const {
  const auto centroid = CalculateCentroid(triangle_index);
  auto normal = CalculateNormal(triangle_index);
  if (glm::dot(centroid, normal) < 0) {
    normal = -normal;
  }
  return glm::abs(glm::dot(centroid, normal));
}

glm::vec3 BillboardCloud::Element::CalculateNormal(const int triangle_index) const {
  return CalculateNormal(triangles.at(triangle_index));
}

glm::vec3 BillboardCloud::Element::CalculateCentroid(const glm::uvec3& triangle) const {
  const auto& a = vertices[triangle.x].position;
  const auto& b = vertices[triangle.y].position;
  const auto& c = vertices[triangle.z].position;

  return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
}

float BillboardCloud::Element::CalculateArea(const glm::uvec3& triangle) const {
  const auto& p0 = vertices[triangle.x].position;
  const auto& p1 = vertices[triangle.y].position;
  const auto& p2 = vertices[triangle.z].position;
  const float a = glm::length(p0 - p1);
  const float b = glm::length(p2 - p1);
  const float c = glm::length(p0 - p2);
  const float d = (a + b + c) / 2;
  return glm::sqrt(d * (d - a) * (d - b) * (d - c));
}

glm::vec3 BillboardCloud::Element::CalculateNormal(const glm::uvec3& triangle) const {
  const auto& p0 = vertices[triangle.x].position;
  const auto& p1 = vertices[triangle.y].position;
  const auto& p2 = vertices[triangle.z].position;
  return glm::normalize(glm::cross(p0 - p1, p0 - p2));
}

float BillboardCloud::Element::CalculateNormalDistance(const glm::uvec3& triangle) const {
  const auto centroid = CalculateCentroid(triangle);
  auto normal = CalculateNormal(triangle);
  if (glm::dot(centroid, normal) < 0) {
    normal = -normal;
  }
  return glm::abs(glm::dot(centroid, normal));
}

void BillboardCloud::Rectangle::Update() {
  center = (points[0] + points[2]) * .5f;
  const auto v_x = points[1] - points[0];
  const auto v_y = points[2] - points[1];
  x_axis = glm::normalize(v_x);
  y_axis = glm::normalize(v_y);

  width = glm::length(v_x);
  height = glm::length(v_y);
}

struct CPUDepthBuffer {
  std::vector<float> depth_buffer;
  std::vector<bool> modified;
  std::vector<std::mutex> pixel_locks;
  int width = 0;
  int height = 0;
  CPUDepthBuffer(const size_t width, const size_t height) {
    this->depth_buffer = std::vector<float>(width * height);
    this->modified = std::vector<bool>(width * height);
    Reset();
    this->pixel_locks = std::vector<std::mutex>(width * height);
    this->width = width;
    this->height = height;
  }

  void Reset() {
    std::fill(depth_buffer.begin(), depth_buffer.end(), -FLT_MAX);
    std::fill(modified.begin(), modified.end(), false);
  }

  [[nodiscard]] bool CompareZ(const int u, const int v, const float z) {
    if (u < 0 || v < 0 || u > width - 1 || v > height - 1)
      return false;
    const int uv = u + width * v;

    if (z >= depth_buffer[uv]) {
      std::lock_guard lock(pixel_locks[uv]);
      modified[uv] = true;
      depth_buffer[uv] = z;
      return true;
    }
    return false;
  }
};

template <typename T>
struct CPUColorBuffer {
  std::vector<T> color_buffer;
  std::vector<std::mutex> pixel_locks;
  int width = 0;
  int height = 0;
  CPUColorBuffer(const size_t width, const size_t height) {
    this->color_buffer = std::vector<T>(width * height);
    std::fill(this->color_buffer.begin(), this->color_buffer.end(), T(0.f));
    this->pixel_locks = std::vector<std::mutex>(width * height);
    this->width = width;
    this->height = height;
  }

  void FillColor(const T& val) {
    std::fill(color_buffer.begin(), color_buffer.end(), val);
  }

  void SetPixel(const int u, const int v, const T& color) {
    if (u < 0 || v < 0 || u > width - 1 || v > height - 1)
      return;

    const int uv = u + width * v;
    std::lock_guard lock(pixel_locks[uv]);
    color_buffer[uv] = color;
  }
};

struct PointComparator {
  bool operator()(const glm::vec2& a, const glm::vec2& b) const {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  }
};

glm::vec2 BillboardCloud::Rectangle::Transform(const glm::vec2& target) const {
  glm::vec2 retVal = target;
  // Recenter
  retVal -= center;
  const float x = glm::dot(retVal, x_axis);
  const float y = glm::dot(retVal, y_axis);
  retVal = glm::vec2(x, y) + glm::vec2(width, height) * .5f;
  return retVal;
}

glm::vec3 BillboardCloud::Rectangle::Transform(const glm::vec3& target) const {
  glm::vec2 retVal = target;
  // Recenter
  retVal -= center;
  const float x = glm::dot(retVal, x_axis);
  const float y = glm::dot(retVal, y_axis);
  retVal = glm::vec2(x, y) + glm::vec2(width, height) * .5f;
  return {retVal, target.z};
}

float Cross(const glm::vec2& origin, const glm::vec2& a, const glm::vec2& b) {
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<glm::vec2> BillboardCloud::RotatingCalipers::ConvexHull(std::vector<glm::vec2> points) {
  const size_t pointSize = points.size();
  size_t k = 0;
  if (pointSize <= 3)
    return points;

  std::vector<glm::vec2> retVal(2 * pointSize);
  std::sort(points.begin(), points.end(), PointComparator());

  for (size_t i = 0; i < pointSize; ++i) {
    while (k >= 2 && Cross(retVal[k - 2], retVal[k - 1], points[i]) <= 0)
      k--;
    retVal[k++] = points[i];
  }

  for (size_t i = pointSize - 1, t = k + 1; i > 0; --i) {
    while (k >= t && Cross(retVal[k - 2], retVal[k - 1], points[i - 1]) <= 0)
      k--;
    retVal[k++] = points[i - 1];
  }
  retVal.resize(k - 1);
  return retVal;
}

struct MinAreaState {
  size_t bottom;
  size_t left;
  float height;
  float width;
  float base_a;
  float base_b;
  float area;
};

BillboardCloud::Rectangle BillboardCloud::RotatingCalipers::GetMinAreaRectangle(std::vector<glm::vec2> points) {
  auto convex_hull = ConvexHull(std::move(points));
  float min_area = FLT_MAX;
  size_t left = 0, bottom = 0, right = 0, top = 0;

  /* rotating calipers sides will always have coordinates
   (a,b) (-b,a) (-a,-b) (b, -a)
   */
  /* this is a first base vector (a,b) initialized by (1,0) */

  glm::vec2 pt0 = convex_hull[0];
  float left_x = pt0.x;
  float right_x = pt0.x;
  float top_y = pt0.y;
  float bottom_y = pt0.y;

  size_t n = convex_hull.size();

  std::vector<glm::vec2> list(n);
  std::vector<float> lengths(n);

  for (size_t i = 0; i < n; i++) {
    if (pt0.x < left_x) {
      left_x = pt0.x;
      left = i;
    }
    if (pt0.x > right_x) {
      right_x = pt0.x;
      right = i;
    }
    if (pt0.y > top_y) {
      top_y = pt0.y;
      top = i;
    }
    if (pt0.y < bottom_y) {
      bottom_y = pt0.y;
      bottom = i;
    }

    glm::vec2 pt = convex_hull[(i + 1) & (i + 1 < n ? -1 : 0)];
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

  for (size_t i = 0; i < n; i++) {
    float bx = list[i].x;
    float by = list[i].y;
    if (float convexity = ax * by - ay * bx; convexity != 0.f) {
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
  for (size_t k = 0; k < n; k++) {
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
    // get next base
    size_t tempPoint = seq[mainElement];
    float leadX = list[tempPoint].x * lengths[tempPoint];
    float leadY = list[tempPoint].y * lengths[tempPoint];
    switch (mainElement) {
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

    float dx = convex_hull[seq[1]].x - convex_hull[seq[3]].x;
    float dy = convex_hull[seq[1]].y - convex_hull[seq[3]].y;
    float width = dx * baseA + dy * baseB;
    dx = convex_hull[seq[2]].x - convex_hull[seq[0]].x;
    dy = convex_hull[seq[2]].y - convex_hull[seq[0]].y;
    float height = -dx * baseB + dy * baseA;
    if (float area = width * height; area <= min_area) {
      min_area = area;
      minAreaState.base_a = baseA;
      minAreaState.base_b = baseB;
      minAreaState.width = width;
      minAreaState.height = height;
      minAreaState.left = seq[3];
      minAreaState.bottom = seq[0];
      minAreaState.area = area;
    }
  }

  float a1 = minAreaState.base_a;
  float b1 = minAreaState.base_b;

  float a2 = -minAreaState.base_b;
  float b2 = minAreaState.base_a;

  float c1 = a1 * convex_hull[minAreaState.left].x + convex_hull[minAreaState.left].y * b1;
  float c2 = a2 * convex_hull[minAreaState.bottom].x + convex_hull[minAreaState.bottom].y * b2;

  float id = 1.f / (a1 * b2 - a2 * b1);

  float px = (c1 * b2 - c2 * b1) * id;
  float py = (a1 * c2 - a2 * c1) * id;

  glm::vec2 out0(px, py);
  glm::vec2 out1(a1 * minAreaState.width, b1 * minAreaState.width);
  glm::vec2 out2(a2 * minAreaState.height, b2 * minAreaState.height);

  Rectangle retVal;

  retVal.points[0] = out0;
  retVal.points[1] = out0 + out1;
  retVal.points[2] = out0 + out1 + out2;
  retVal.points[3] = out0 + out2;

  return retVal;
}

inline float EdgeFunction(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
}

void Barycentric3D(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, float& c0, float& c1,
                   float& c2) {
  const auto &v0 = b - a, v1 = c - a, v2 = p - a;
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

inline void Barycentric2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c, float& c0,
                          float& c1, float& c2) {
  const auto v0 = b - a, v1 = c - a, v2 = p - a;
  const float den = v0.x * v1.y - v1.x * v0.y;
  if (den == 0.f) {
    c1 = c2 = 0.f;
    c0 = 1.f;
  } else {
    c1 = (v2.x * v1.y - v1.x * v2.y) / den;
    c2 = (v0.x * v2.y - v2.x * v0.y) / den;
    c0 = 1.0f - c1 - c2;
  }
}
inline float Area(const glm::vec2& a, const glm::vec2& b) {
  return a.x * b.y - a.y * b.x;
}

inline void Barycentric2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c,
                          const glm::vec2& d, float& c0, float& c1, float& c2, float& c3) {
  float r[4], t[4], u[4];
  const glm::vec2 v[4] = {a, b, c, d};
  glm::vec2 s[4];
  for (int i = 0; i < 4; i++) {
    s[i] = v[i] - p;
    r[i] = length(s[i]);
  }
  for (int i = 0; i < 4; i++) {
    const float area = Area(s[i], s[(i + 1) % 4]);
    const float dotResult = glm::dot(s[i], s[(i + 1) % 4]);
    if (area == 0.f)
      t[i] = 0.f;
    else {
      t[i] = (r[i] * r[(i + 1) % 4] - dotResult) / area;
    }
  }
  for (int i = 0; i < 4; i++) {
    if (r[i] == 0.f)
      u[i] = 0.f;
    else
      u[i] = (t[(i + 3) % 4] + t[i]) / r[i];
  }
  const auto sum = u[0] + u[1] + u[2] + u[3];
  assert(sum != 0.f);
  c0 = u[0] / sum;
  c1 = u[1] / sum;
  c2 = u[2] / sum;
  c3 = u[3] / sum;
}

void BillboardCloud::Project(const ProjectSettings& project_settings) {
  for (auto& cluster : clusters)
    Project(cluster, project_settings);
}

void BillboardCloud::Join(const JoinSettings& join_settings) {
  xatlas::Atlas* atlas = xatlas::Create();

  for (const auto& cluster : clusters) {
    xatlas::MeshDecl meshDecl;
    meshDecl.vertexCount = cluster.billboard_vertices.size();
    meshDecl.vertexPositionData = cluster.billboard_vertices.data();
    meshDecl.vertexPositionStride = sizeof(Vertex);
    meshDecl.indexCount = cluster.billboard_triangles.size() * 3;
    meshDecl.indexData = cluster.billboard_triangles.data();
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
  billboardCloudVertices.resize(clusters.size() * 4);
  std::vector<glm::uvec3> billboardCloudTriangles;
  billboardCloudTriangles.resize(clusters.size() * 2);
  Jobs::RunParallelFor(clusters.size(), [&](const unsigned clusterIndex) {
    const xatlas::Mesh& mesh = atlas->meshes[clusterIndex];
    auto& cluster = clusters[clusterIndex];
    cluster.rectangle.tex_coords[0].x = mesh.vertexArray[0].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[0].y = mesh.vertexArray[0].uv[1] / static_cast<float>(atlas->height);
    cluster.rectangle.tex_coords[1].x = mesh.vertexArray[1].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[1].y = mesh.vertexArray[1].uv[1] / static_cast<float>(atlas->height);
    cluster.rectangle.tex_coords[2].x = mesh.vertexArray[2].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[2].y = mesh.vertexArray[2].uv[1] / static_cast<float>(atlas->height);
    cluster.rectangle.tex_coords[3].x = mesh.vertexArray[3].uv[0] / static_cast<float>(atlas->width);
    cluster.rectangle.tex_coords[3].y = mesh.vertexArray[3].uv[1] / static_cast<float>(atlas->height);

    billboardCloudVertices[4 * clusterIndex] = cluster.billboard_vertices[0];
    billboardCloudVertices[4 * clusterIndex + 1] = cluster.billboard_vertices[1];
    billboardCloudVertices[4 * clusterIndex + 2] = cluster.billboard_vertices[2];
    billboardCloudVertices[4 * clusterIndex + 3] = cluster.billboard_vertices[3];

    billboardCloudVertices[4 * clusterIndex].tex_coord = cluster.rectangle.tex_coords[0];
    billboardCloudVertices[4 * clusterIndex + 1].tex_coord = cluster.rectangle.tex_coords[1];
    billboardCloudVertices[4 * clusterIndex + 2].tex_coord = cluster.rectangle.tex_coords[2];
    billboardCloudVertices[4 * clusterIndex + 3].tex_coord = cluster.rectangle.tex_coords[3];

    billboardCloudTriangles[2 * clusterIndex] = cluster.billboard_triangles[0] + glm::uvec3(clusterIndex * 4);
    billboardCloudTriangles[2 * clusterIndex + 1] = cluster.billboard_triangles[1] + glm::uvec3(clusterIndex * 4);
  });

  billboard_cloud_mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  billboard_cloud_mesh->SetVertices({false, false, true, false}, billboardCloudVertices, billboardCloudTriangles);
  xatlas::Destroy(atlas);
}
struct PBRMaterial {
  glm::vec4 m_baseAlbedo = glm::vec4(1.f);
  float m_baseRoughness = 0.3f;
  float m_baseMetallic = 0.3f;
  float m_baseAo = 1.f;
  glm::ivec2 m_albedoTextureResolution = glm::ivec2(-1);
  std::vector<glm::vec4> m_albedoTextureData;

  glm::ivec2 normalTextureResolution = glm::ivec2(-1);
  std::vector<glm::vec3> normalTextureData;

  glm::ivec2 roughnessTextureResolution = glm::ivec2(-1);
  std::vector<float> roughnessTextureData;

  glm::ivec2 metallicTextureResolution = glm::ivec2(-1);
  std::vector<float> metallicTextureData;

  glm::ivec2 m_aoTextureResolution = glm::ivec2(-1);
  std::vector<float> m_aoTextureData;
  void Clear();
  void ApplyMaterial(const std::shared_ptr<Material>& material,
                     const BillboardCloud::RasterizeSettings& rasterizeSettings);
};
void PBRMaterial::Clear() {
  m_baseAlbedo = glm::vec4(1.f);
  m_baseRoughness = 0.3f;
  m_baseMetallic = 0.3f;
  m_baseAo = 1.f;
  m_albedoTextureResolution = glm::ivec2(-1);
  normalTextureResolution = glm::ivec2(-1);
  roughnessTextureResolution = glm::ivec2(-1);
  metallicTextureResolution = glm::ivec2(-1);
  m_aoTextureResolution = glm::ivec2(-1);

  m_albedoTextureData.clear();
  normalTextureData.clear();
  roughnessTextureData.clear();
  metallicTextureData.clear();
  m_aoTextureData.clear();
}

void PBRMaterial::ApplyMaterial(const std::shared_ptr<Material>& material,
                                const BillboardCloud::RasterizeSettings& rasterizeSettings) {
  m_baseAlbedo =
      glm::vec4(material->material_properties.albedo_color, 1.f - material->material_properties.transmission);
  m_baseRoughness = material->material_properties.roughness;
  m_baseMetallic = material->material_properties.metallic;
  m_baseAo = 1.f;

  const auto albedoTexture = material->GetAlbedoTexture();
  if (rasterizeSettings.transfer_albedo_map && albedoTexture) {
    albedoTexture->GetRgbaChannelData(m_albedoTextureData);
    m_albedoTextureResolution = albedoTexture->GetResolution();
  }
  const auto normalTexture = material->GetNormalTexture();
  if (rasterizeSettings.transfer_normal_map && normalTexture) {
    normalTexture->GetRgbChannelData(normalTextureData);
    normalTextureResolution = normalTexture->GetResolution();
  }
  const auto roughnessTexture = material->GetRoughnessTexture();
  if (rasterizeSettings.transfer_roughness_map && roughnessTexture) {
    roughnessTexture->GetRedChannelData(roughnessTextureData);
    roughnessTextureResolution = roughnessTexture->GetResolution();
  }
  const auto metallicTexture = material->GetMetallicTexture();
  if (rasterizeSettings.transfer_metallic_map && metallicTexture) {
    metallicTexture->GetRedChannelData(metallicTextureData);
    metallicTextureResolution = metallicTexture->GetResolution();
  }
  const auto aoTexture = material->GetAoTexture();
  if (rasterizeSettings.transfer_ao_map && aoTexture) {
    aoTexture->GetRedChannelData(m_aoTextureData);
    m_aoTextureResolution = aoTexture->GetResolution();
  }
}

void BillboardCloud::Rasterize(const RasterizeSettings& rasterize_settings) {
  if (rasterize_settings.resolution.x < 1 || rasterize_settings.resolution.y < 1)
    return;
  std::unordered_map<Handle, PBRMaterial> pbrMaterials;
  float averageRoughness = 0.f;
  float averageMetallic = 0.0f;
  float averageAo = 0.f;
  for (auto& element : elements) {
    const auto& material = element.material;
    auto materialHandle = material->GetHandle();
    pbrMaterials[materialHandle].ApplyMaterial(material, rasterize_settings);

    averageRoughness += material->material_properties.roughness;
    averageMetallic += material->material_properties.metallic;
    averageAo += 1.f;
  }
  averageRoughness /= elements.size();
  averageMetallic /= elements.size();
  averageAo /= elements.size();

  CPUDepthBuffer depthBuffer(rasterize_settings.resolution.x, rasterize_settings.resolution.y);

  CPUColorBuffer<glm::vec4> albedoFrameBuffer(rasterize_settings.resolution.x, rasterize_settings.resolution.y);
  CPUColorBuffer<glm::vec3> normalFrameBuffer(rasterize_settings.resolution.x, rasterize_settings.resolution.y);
  CPUColorBuffer<float> roughnessFrameBuffer(rasterize_settings.resolution.x, rasterize_settings.resolution.y);
  CPUColorBuffer<float> metallicFrameBuffer(rasterize_settings.resolution.x, rasterize_settings.resolution.y);
  CPUColorBuffer<float> aoFrameBuffer(rasterize_settings.resolution.x, rasterize_settings.resolution.y);

  if (rasterize_settings.debug_full_fill)
    albedoFrameBuffer.FillColor(glm::vec4(1.f));

  if (rasterize_settings.debug_opaque) {
    averageRoughness = 1.f;
    averageMetallic = 0.f;
    averageAo = 1.f;
    Jobs::RunParallelFor(clusters.size(), [&](const unsigned clusterIndex) {
      const auto& cluster = clusters[clusterIndex];
      glm::vec3 color = glm::ballRand(1.f);
      const auto& boundingRectangle = cluster.rectangle;

      for (int triangleIndex = 0; triangleIndex < 2; triangleIndex++) {
        glm::vec3 textureSpaceVertices[3];
        if (triangleIndex == 0) {
          textureSpaceVertices[0] =
              glm::vec3(boundingRectangle.tex_coords[0] * glm::vec2(rasterize_settings.resolution), 0.f);
          textureSpaceVertices[1] =
              glm::vec3(boundingRectangle.tex_coords[1] * glm::vec2(rasterize_settings.resolution), 0.f);
          textureSpaceVertices[2] =
              glm::vec3(boundingRectangle.tex_coords[2] * glm::vec2(rasterize_settings.resolution), 0.f);
        } else {
          textureSpaceVertices[0] =
              glm::vec3(boundingRectangle.tex_coords[2] * glm::vec2(rasterize_settings.resolution), 0.f);
          textureSpaceVertices[1] =
              glm::vec3(boundingRectangle.tex_coords[3] * glm::vec2(rasterize_settings.resolution), 0.f);
          textureSpaceVertices[2] =
              glm::vec3(boundingRectangle.tex_coords[0] * glm::vec2(rasterize_settings.resolution), 0.f);
        }
        // Bound check;
        auto minBound = glm::vec2(FLT_MAX, FLT_MAX);
        auto maxBound = glm::vec2(-FLT_MAX, -FLT_MAX);
        for (const auto& textureSpaceVertex : textureSpaceVertices) {
          minBound = glm::min(glm::vec2(textureSpaceVertex), minBound);
          maxBound = glm::max(glm::vec2(textureSpaceVertex), maxBound);
        }

        const auto left = static_cast<int>(minBound.x - 0.5f);
        const auto right = static_cast<int>(maxBound.x + 0.5f);
        const auto top = static_cast<int>(minBound.y - 0.5f);
        const auto bottom = static_cast<int>(maxBound.y + 0.5f);
        for (auto u = left; u <= right; u++) {
          for (auto v = top; v <= bottom; v++) {
            const auto p = glm::vec3(u + .5f, v + .5f, 0.f);
            float bc0, bc1, bc2;
            Barycentric2D(p, textureSpaceVertices[0], textureSpaceVertices[1], textureSpaceVertices[2], bc0, bc1, bc2);
            if (bc0 < 0.f || bc1 < 0.f || bc2 < 0.f)
              continue;
            auto albedo = glm::vec4(color, 1.f);
            albedoFrameBuffer.SetPixel(u, v, albedo);
          }
        }
      }
    });
  }

  for (auto& cluster : clusters) {
    // Calculate texture size on atlas
    const auto& boundingRectangle = cluster.rectangle;

    // Rasterization
    Jobs::RunParallelFor(cluster.projected_triangles.size(), [&](const unsigned triangleIndex) {
      const auto& triangle = cluster.projected_triangles[triangleIndex];
      const auto& v0 = triangle.projected_vertices[0];
      const auto& v1 = triangle.projected_vertices[1];
      const auto& v2 = triangle.projected_vertices[2];
      const auto& material = pbrMaterials.at(triangle.material_handle);

      glm::vec3 texture_space_vertices[3];
      for (int i = 0; i < 3; i++) {
        const auto p =
            glm::vec2(triangle.projected_vertices[i].position.x, triangle.projected_vertices[i].position.y);
        const auto r0 = boundingRectangle.points[0];
        const auto r1 = boundingRectangle.points[1];
        const auto r2 = boundingRectangle.points[2];
        const auto r3 = boundingRectangle.points[3];

        float bc0, bc1, bc2, bc3;
        Barycentric2D(p, r0, r1, r2, r3, bc0, bc1, bc2, bc3);
        const auto texture_space_position =
            (boundingRectangle.tex_coords[0] * bc0 + boundingRectangle.tex_coords[1] * bc1 +
             boundingRectangle.tex_coords[2] * bc2 + boundingRectangle.tex_coords[3] * bc3) *
            glm::vec2(rasterize_settings.resolution);
        texture_space_vertices[i].x = texture_space_position.x;
        texture_space_vertices[i].y = texture_space_position.y;
        texture_space_vertices[i].z = triangle.projected_vertices[i].position.z;
      }

      // Bound check;
      auto minBound = glm::vec2(FLT_MAX, FLT_MAX);
      auto maxBound = glm::vec2(-FLT_MAX, -FLT_MAX);
      for (const auto& textureSpaceVertex : texture_space_vertices) {
        minBound = glm::min(glm::vec2(textureSpaceVertex), minBound);
        maxBound = glm::max(glm::vec2(textureSpaceVertex), maxBound);
      }

      const auto left = static_cast<int>(minBound.x - 0.5f);
      const auto right = static_cast<int>(maxBound.x + 0.5f);
      const auto top = static_cast<int>(minBound.y - 0.5f);
      const auto bottom = static_cast<int>(maxBound.y + 0.5f);
      for (auto u = left; u <= right; u++) {
        for (auto v = top; v <= bottom; v++) {
          const auto p = glm::vec3(u + .5f, v + .5f, 0.f);
          float bc0, bc1, bc2;
          Barycentric2D(p, texture_space_vertices[0], texture_space_vertices[1], texture_space_vertices[2], bc0, bc1, bc2);
          if (bc0 < 0.f || bc1 < 0.f || bc2 < 0.f)
            continue;
          float z = bc0 * v0.position.z + bc1 * v1.position.z + bc2 * v2.position.z;
          // Early depth check.
          if (!depthBuffer.CompareZ(u, v, z))
            continue;

          const auto texCoords = bc0 * v0.tex_coord + bc1 * v1.tex_coord + bc2 * v2.tex_coord;
          auto albedo = material.m_baseAlbedo;
          float roughness = material.m_baseRoughness;
          float metallic = material.m_baseMetallic;
          float ao = material.m_baseAo;
          if (!material.m_albedoTextureData.empty()) {
            int textureX = static_cast<int>(material.m_albedoTextureResolution.x * texCoords.x) %
                           material.m_albedoTextureResolution.x;
            int textureY = static_cast<int>(material.m_albedoTextureResolution.y * texCoords.y) %
                           material.m_albedoTextureResolution.y;
            if (textureX < 0)
              textureX += material.m_albedoTextureResolution.x;
            if (textureY < 0)
              textureY += material.m_albedoTextureResolution.y;

            const auto index = textureY * material.m_albedoTextureResolution.x + textureX;
            albedo = material.m_albedoTextureData[index];
          }
          // Alpha discard
          if (albedo.a < 0.1f)
            continue;
          auto normal = glm::normalize(bc0 * v0.normal + bc1 * v1.normal + bc2 * v2.normal);

          if (!material.normalTextureData.empty()) {
            auto tangent = glm::normalize(bc0 * v0.tangent + bc1 * v1.tangent + bc2 * v2.tangent);
            const auto biTangent = glm::cross(normal, tangent);
            const auto tbn = glm::mat3(tangent, biTangent, normal);

            int textureX = static_cast<int>(material.normalTextureResolution.x * texCoords.x) %
                           material.normalTextureResolution.x;
            int textureY = static_cast<int>(material.normalTextureResolution.y * texCoords.y) %
                           material.normalTextureResolution.y;
            if (textureX < 0)
              textureX += material.normalTextureResolution.x;
            if (textureY < 0)
              textureY += material.normalTextureResolution.y;

            const auto index = textureY * material.normalTextureResolution.x + textureX;
            const auto sampled = glm::normalize(material.normalTextureData[index]) * 2.0f - glm::vec3(1.0f);
            normal = glm::normalize(tbn * sampled);
          }
          if (glm::dot(normal, glm::vec3(0, 0, 1)) < 0)
            normal = -normal;

          if (!material.roughnessTextureData.empty()) {
            int textureX = static_cast<int>(material.roughnessTextureResolution.x * texCoords.x) %
                           material.roughnessTextureResolution.x;
            int textureY = static_cast<int>(material.roughnessTextureResolution.y * texCoords.y) %
                           material.roughnessTextureResolution.y;
            if (textureX < 0)
              textureX += material.roughnessTextureResolution.x;
            if (textureY < 0)
              textureY += material.roughnessTextureResolution.y;

            const auto index = textureY * material.roughnessTextureResolution.x + textureX;
            roughness = material.roughnessTextureData[index];
          }
          if (!material.metallicTextureData.empty()) {
            int textureX = static_cast<int>(material.metallicTextureResolution.x * texCoords.x) %
                           material.metallicTextureResolution.x;
            int textureY = static_cast<int>(material.metallicTextureResolution.y * texCoords.y) %
                           material.metallicTextureResolution.y;
            if (textureX < 0)
              textureX += material.metallicTextureResolution.x;
            if (textureY < 0)
              textureY += material.metallicTextureResolution.y;

            const auto index = textureY * material.metallicTextureResolution.x + textureX;
            metallic = material.metallicTextureData[index];
          }
          if (!material.m_aoTextureData.empty()) {
            int textureX =
                static_cast<int>(material.m_aoTextureResolution.x * texCoords.x) % material.m_aoTextureResolution.x;
            int textureY =
                static_cast<int>(material.m_aoTextureResolution.y * texCoords.y) % material.m_aoTextureResolution.y;
            if (textureX < 0)
              textureX += material.m_aoTextureResolution.x;
            if (textureY < 0)
              textureY += material.m_aoTextureResolution.y;

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

  if (rasterize_settings.dilate != 0) {
    bool lastIterationUpdated = true;
    while (lastIterationUpdated) {
      lastIterationUpdated = false;
      std::vector<bool> threadUpdated = std::vector<bool>(Jobs::GetWorkerSize());
      std::fill(threadUpdated.begin(), threadUpdated.end(), false);
      Jobs::RunParallelFor(depthBuffer.height, [&](unsigned lineIndex, unsigned threadIndex) {
        const int j = lineIndex;
        for (int i = 0; i < depthBuffer.width; i++) {
          const auto testIndex = i + j * depthBuffer.width;
          if (!depthBuffer.modified[testIndex]) {
            float sum = 0.f;
            bool hasLeft = false;
            const int leftIndex = i - 1 + j * depthBuffer.width;
            if (i > 0 && depthBuffer.modified[leftIndex]) {
              hasLeft = true;
              sum += 1.f;
            }
            bool hasRight = false;
            const int rightIndex = i + 1 + j * depthBuffer.width;
            if (i + 1 < depthBuffer.width && depthBuffer.modified[rightIndex]) {
              hasRight = true;
              sum += 1.f;
            }
            bool hasTop = false;
            const int topIndex = i + (j - 1) * depthBuffer.width;
            if (j > 0 && depthBuffer.modified[topIndex]) {
              hasTop = true;
              sum += 1.f;
            }
            bool hasBottom = false;
            const int bottomIndex = i + (j + 1) * depthBuffer.width;
            if (j + 1 < depthBuffer.height && depthBuffer.modified[bottomIndex]) {
              hasBottom = true;
              sum += 1.f;
            }
            if (sum == 0.f)
              continue;
            threadUpdated[threadIndex] = true;
            std::lock_guard lock(depthBuffer.pixel_locks[testIndex]);
            depthBuffer.modified[testIndex] = true;
            if (!albedoFrameBuffer.color_buffer.empty()) {
              glm::vec4 albedoColor{};
              if (hasLeft) {
                albedoColor += albedoFrameBuffer.color_buffer[leftIndex];
              }
              if (hasRight) {
                albedoColor += albedoFrameBuffer.color_buffer[rightIndex];
              }
              if (hasTop) {
                albedoColor += albedoFrameBuffer.color_buffer[topIndex];
              }
              if (hasBottom) {
                albedoColor += albedoFrameBuffer.color_buffer[bottomIndex];
              }
              albedoColor /= sum;

              albedoFrameBuffer.color_buffer[testIndex].x = albedoColor.x;
              albedoFrameBuffer.color_buffer[testIndex].y = albedoColor.y;
              albedoFrameBuffer.color_buffer[testIndex].z = albedoColor.z;
            }
            if (!normalFrameBuffer.color_buffer.empty()) {
              glm::vec3 normalColor{};
              if (hasLeft) {
                normalColor += normalFrameBuffer.color_buffer[leftIndex];
              }
              if (hasRight) {
                normalColor += normalFrameBuffer.color_buffer[rightIndex];
              }
              if (hasTop) {
                normalColor += normalFrameBuffer.color_buffer[topIndex];
              }
              if (hasBottom) {
                normalColor += normalFrameBuffer.color_buffer[bottomIndex];
              }
              normalColor /= sum;

              normalFrameBuffer.color_buffer[testIndex] = normalColor;
            }
            if (!roughnessFrameBuffer.color_buffer.empty()) {
              float roughnessColor{};
              if (hasLeft) {
                roughnessColor += roughnessFrameBuffer.color_buffer[leftIndex];
              }
              if (hasRight) {
                roughnessColor += roughnessFrameBuffer.color_buffer[rightIndex];
              }
              if (hasTop) {
                roughnessColor += roughnessFrameBuffer.color_buffer[topIndex];
              }
              if (hasBottom) {
                roughnessColor += roughnessFrameBuffer.color_buffer[bottomIndex];
              }
              roughnessColor /= sum;

              roughnessFrameBuffer.color_buffer[testIndex] = roughnessColor;
            }
            if (!metallicFrameBuffer.color_buffer.empty()) {
              float metallicColor{};
              if (hasLeft) {
                metallicColor += metallicFrameBuffer.color_buffer[leftIndex];
              }
              if (hasRight) {
                metallicColor += metallicFrameBuffer.color_buffer[rightIndex];
              }
              if (hasTop) {
                metallicColor += metallicFrameBuffer.color_buffer[topIndex];
              }
              if (hasBottom) {
                metallicColor += metallicFrameBuffer.color_buffer[bottomIndex];
              }
              metallicColor /= sum;
              metallicFrameBuffer.color_buffer[testIndex] = metallicColor;
            }
            if (!aoFrameBuffer.color_buffer.empty()) {
              float aoColor{};
              if (hasLeft) {
                aoColor += aoFrameBuffer.color_buffer[leftIndex];
              }
              if (hasRight) {
                aoColor += aoFrameBuffer.color_buffer[rightIndex];
              }
              if (hasTop) {
                aoColor += aoFrameBuffer.color_buffer[topIndex];
              }
              if (hasBottom) {
                aoColor += aoFrameBuffer.color_buffer[bottomIndex];
              }
              aoColor /= sum;
              aoFrameBuffer.color_buffer[testIndex] = aoColor;
            }
          }
        }
      });
      for (const auto&& i : threadUpdated)
        if (i) {
          lastIterationUpdated = true;
          break;
        }
    }
  }

  billboard_cloud_material = ProjectManager::CreateTemporaryAsset<Material>();
  std::shared_ptr<Texture2D> albedoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
  albedoTexture->SetRgbaChannelData(albedoFrameBuffer.color_buffer,
                                    glm::uvec2(albedoFrameBuffer.width, albedoFrameBuffer.height));
  billboard_cloud_material->SetAlbedoTexture(albedoTexture);
  if (rasterize_settings.transfer_normal_map) {
    std::shared_ptr<Texture2D> normalTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    normalTexture->SetRgbChannelData(normalFrameBuffer.color_buffer,
                                     glm::uvec2(normalFrameBuffer.width, normalFrameBuffer.height));
    billboard_cloud_material->SetNormalTexture(normalTexture);
  }
  if (rasterize_settings.transfer_roughness_map) {
    std::shared_ptr<Texture2D> roughnessTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    roughnessTexture->SetRedChannelData(roughnessFrameBuffer.color_buffer,
                                        glm::uvec2(roughnessFrameBuffer.width, roughnessFrameBuffer.height));
    billboard_cloud_material->SetRoughnessTexture(roughnessTexture);
  } else {
    billboard_cloud_material->material_properties.roughness = averageRoughness;
  }
  if (rasterize_settings.transfer_metallic_map) {
    std::shared_ptr<Texture2D> metallicTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    metallicTexture->SetRedChannelData(metallicFrameBuffer.color_buffer,
                                       glm::uvec2(metallicFrameBuffer.width, metallicFrameBuffer.height));
    billboard_cloud_material->SetMetallicTexture(metallicTexture);
  } else {
    billboard_cloud_material->material_properties.metallic = averageMetallic;
  }
  if (rasterize_settings.transfer_ao_map) {
    std::shared_ptr<Texture2D> aoTexture = ProjectManager::CreateTemporaryAsset<Texture2D>();
    aoTexture->SetRedChannelData(aoFrameBuffer.color_buffer,
                                 glm::uvec2(aoFrameBuffer.width, aoFrameBuffer.height));
    billboard_cloud_material->SetAoTexture(aoTexture);
  }
}

void BillboardCloud::Generate(const GenerateSettings& generate_settings) {
  Clusterize(generate_settings.clusterization_settings);
  Project(generate_settings.project_settings);
  Join(generate_settings.join_settings);
  Rasterize(generate_settings.rasterize_settings);
}

void BillboardCloud::Project(Cluster& cluster, const ProjectSettings& project_settings) const {
  const auto billboardFrontAxis = cluster.cluster_plane.GetNormal();
  auto billboardUpAxis =
      glm::vec3(billboardFrontAxis.y, billboardFrontAxis.z, billboardFrontAxis.x);  // cluster.m_planeYAxis;
  const auto billboardLeftAxis = glm::normalize(glm::cross(billboardFrontAxis, billboardUpAxis));
  billboardUpAxis = glm::normalize(glm::cross(billboardLeftAxis, billboardFrontAxis));
  glm::mat4 rotateMatrix =
      glm::transpose(glm::mat4(glm::vec4(billboardLeftAxis, 0.0f), glm::vec4(billboardUpAxis, 0.0f),
                               glm::vec4(billboardFrontAxis, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
  cluster.projected_triangles.resize(cluster.triangles.size());
  Jobs::RunParallelFor(cluster.triangles.size(), [&](const unsigned triangleIndex) {
    const auto& clusterTriangle = cluster.triangles.at(triangleIndex);
    auto& projectedTriangle = cluster.projected_triangles[triangleIndex];
    const auto& element = elements.at(clusterTriangle.element_index);
    const auto& vertices = element.vertices;
    const auto& triangle = element.triangles.at(clusterTriangle.triangle_index);
    auto& v0 = vertices.at(triangle.x);
    auto& v1 = vertices.at(triangle.y);
    auto& v2 = vertices.at(triangle.z);

    auto& pV0 = projectedTriangle.projected_vertices[0];
    auto& pV1 = projectedTriangle.projected_vertices[1];
    auto& pV2 = projectedTriangle.projected_vertices[2];

    TransformVertex(v0, pV0, rotateMatrix);
    TransformVertex(v1, pV1, rotateMatrix);
    TransformVertex(v2, pV2, rotateMatrix);

    projectedTriangle.material_handle = element.material->GetHandle();
  });

  std::vector<glm::vec2> points;
  points.resize(cluster.projected_triangles.size() * 3);
  Jobs::RunParallelFor(cluster.projected_triangles.size(), [&](unsigned triangleIndex) {
    const auto& projectedTriangle = cluster.projected_triangles.at(triangleIndex);
    points.at(triangleIndex * 3) = glm::vec2(projectedTriangle.projected_vertices[0].position.x,
                                             projectedTriangle.projected_vertices[0].position.y);
    points.at(triangleIndex * 3 + 1) = glm::vec2(projectedTriangle.projected_vertices[1].position.x,
                                                 projectedTriangle.projected_vertices[1].position.y);
    points.at(triangleIndex * 3 + 2) = glm::vec2(projectedTriangle.projected_vertices[2].position.x,
                                                 projectedTriangle.projected_vertices[2].position.y);
  });

  // Calculate bounding triangle.
  assert(points.size() > 2);
  if (points.size() == 3) {
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

    } else if (e1 >= e0 && e1 >= e2) {
      longestEdgeStart = p1;
      longestEdgeEnd = p2;
      otherPoint = p0;
    } else {
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
    cluster.rectangle.points[0] = longestEdgeStart;
    cluster.rectangle.points[3] = longestEdgeStart + length * lengthVector;
    cluster.rectangle.points[1] = cluster.rectangle.points[0] + width * widthVector;
    cluster.rectangle.points[2] = cluster.rectangle.points[3] + width * widthVector;
  } else {
    cluster.rectangle = RotatingCalipers::GetMinAreaRectangle(std::move(points));
  }
  cluster.rectangle.Update();
  // Generate billboard mesh
  cluster.billboard_vertices.resize(4);
  const auto inverseRotateMatrix = glm::inverse(rotateMatrix);
  cluster.billboard_vertices[0].position =
      inverseRotateMatrix * glm::vec4(cluster.rectangle.points[0].x, cluster.rectangle.points[0].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[0] = cluster.billboard_vertices[0].tex_coord = glm::vec2(0, 0);
  cluster.billboard_vertices[1].position =
      inverseRotateMatrix * glm::vec4(cluster.rectangle.points[1].x, cluster.rectangle.points[1].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[1] = cluster.billboard_vertices[1].tex_coord = glm::vec2(1, 0);
  cluster.billboard_vertices[2].position =
      inverseRotateMatrix * glm::vec4(cluster.rectangle.points[2].x, cluster.rectangle.points[2].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[2] = cluster.billboard_vertices[2].tex_coord = glm::vec2(1, 1);
  cluster.billboard_vertices[3].position =
      inverseRotateMatrix * glm::vec4(cluster.rectangle.points[3].x, cluster.rectangle.points[3].y, 0.f, 0.f);
  cluster.rectangle.tex_coords[3] = cluster.billboard_vertices[3].tex_coord = glm::vec2(0, 1);

  cluster.billboard_vertices[0].position -= cluster.cluster_plane.d * billboardFrontAxis;
  cluster.billboard_vertices[1].position -= cluster.cluster_plane.d * billboardFrontAxis;
  cluster.billboard_vertices[2].position -= cluster.cluster_plane.d * billboardFrontAxis;
  cluster.billboard_vertices[3].position -= cluster.cluster_plane.d * billboardFrontAxis;

  cluster.billboard_triangles.resize(2);
  cluster.billboard_triangles[0] = {0, 1, 2};
  cluster.billboard_triangles[1] = {2, 3, 0};
}


#pragma endregion
#pragma region IO
Entity BillboardCloud::BuildEntity(const std::shared_ptr<Scene>& scene) const {
  if (!billboard_cloud_mesh || !billboard_cloud_material)
    return {};
  const auto owner = scene->CreateEntity("Billboard Cloud");

  const auto projectedElementEntity = scene->CreateEntity("Billboard Cloud");
  const auto elementMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(projectedElementEntity).lock();
  elementMeshRenderer->mesh = billboard_cloud_mesh;
  elementMeshRenderer->material = billboard_cloud_material;
  scene->SetParent(projectedElementEntity, owner);

  return owner;
}

void BillboardCloud::BoundingSphere::Initialize(const std::vector<BillboardCloud::Element>& elements) {
  size_t triangleSize = 0;
  auto positionSum = glm::vec3(0.f);

  // TODO: Parallelize
  for (const auto& element : elements) {
    for (const auto& triangle : element.triangles) {
      triangleSize++;
      positionSum += element.CalculateCentroid(triangle);
    }
  }
  center = positionSum / static_cast<float>(triangleSize);

  radius = 0.f;
  for (const auto& element : elements) {
    for (const auto& vertex : element.vertices) {
      radius = glm::max(radius, glm::distance(center, vertex.position));
    }
  }
}

std::vector<glm::vec3> BillboardCloud::ExtractPointCloud(const float density) const {
  std::vector<glm::vec3> points;
  BoundingSphere boundingSphere;
  boundingSphere.Initialize(elements);

  const auto div = glm::pow(boundingSphere.radius * density, 2.f);
  for (const auto& element : elements) {
    std::unordered_set<unsigned> selectedIndices;
    for (const auto& triangle : element.triangles) {
      const auto& v0 = element.vertices[triangle.x].position;
      const auto& v1 = element.vertices[triangle.y].position;
      const auto& v2 = element.vertices[triangle.z].position;
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

void BillboardCloud::ProcessPrefab(const std::shared_ptr<Prefab>& current_prefab,
                                   const Transform& parent_model_space_transform) {
  Transform transform{};
  for (const auto& dataComponent : current_prefab->data_components) {
    if (dataComponent.data_component_type == Typeof<Transform>()) {
      transform = *std::reinterpret_pointer_cast<Transform>(dataComponent.data_component);
    }
  }
  transform.value = parent_model_space_transform.value * transform.value;
  for (const auto& privateComponent : current_prefab->private_components) {
    if (privateComponent.private_component->GetTypeName() == "MeshRenderer") {
      std::vector<AssetRef> assetRefs;
      privateComponent.private_component->CollectAssetRef(assetRefs);
      std::shared_ptr<Mesh> mesh{};
      std::shared_ptr<Material> material{};
      for (auto& assetRef : assetRefs) {
        if (const auto testMesh = assetRef.Get<Mesh>()) {
          mesh = testMesh;
        } else if (const auto testMaterial = assetRef.Get<Material>()) {
          material = testMaterial;
        }
      }
      if (mesh && material) {
        elements.emplace_back();
        auto& element = elements.back();
        element.vertices = mesh->UnsafeGetVertices();
        element.material = material;
        element.triangles = mesh->UnsafeGetTriangles();
        Jobs::RunParallelFor(element.vertices.size(), [&](unsigned vertexIndex) {
          TransformVertex(element.vertices.at(vertexIndex), parent_model_space_transform.value);
        });
      }
    }
  }
  for (const auto& child_prefab : current_prefab->child_prefabs) {
    ProcessPrefab(child_prefab, transform);
  }
}

void BillboardCloud::ProcessEntity(const std::shared_ptr<Scene>& scene, const Entity& entity,
                                   const Transform& parent_model_space_transform) {
  Transform transform = scene->GetDataComponent<Transform>(entity);
  transform.value = parent_model_space_transform.value * transform.value;
  if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
    const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    const auto mesh = meshRenderer->mesh.Get<Mesh>();
    const auto material = meshRenderer->material.Get<Material>();
    if (mesh && material) {
      elements.emplace_back();
      auto& element = elements.back();
      element.vertices = mesh->UnsafeGetVertices();
      element.material = material;
      element.triangles = mesh->UnsafeGetTriangles();
      Jobs::RunParallelFor(element.vertices.size(), [&](unsigned vertexIndex) {
        TransformVertex(element.vertices.at(vertexIndex), parent_model_space_transform.value);
      });
    }
  }

  for (const auto& childEntity : scene->GetChildren(entity)) {
    ProcessEntity(scene, childEntity, transform);
  }
}

#pragma endregion
#pragma region Clusterization
void BillboardCloud::Process(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material) {
  elements.emplace_back();
  auto& element = elements.back();
  element.vertices = mesh->UnsafeGetVertices();
  element.material = material;
  element.triangles = mesh->UnsafeGetTriangles();
}

void BillboardCloud::Process(const std::shared_ptr<Prefab>& prefab) {
  ProcessPrefab(prefab, Transform());
}

void BillboardCloud::Process(const std::shared_ptr<Scene>& scene, const Entity& entity) {
  ProcessEntity(scene, entity, Transform());
}

void BillboardCloud::Clusterize(const ClusterizationSettings& clusterize_settings) {
  clusters.clear();
  switch (clusterize_settings.clusterize_mode) {
    case static_cast<unsigned>(ClusterizationMode::FlipBook): {
      clusters.emplace_back();
      auto& cluster = clusters.back();
      cluster.triangles = CollectTriangles();
    } break;
    case static_cast<unsigned>(ClusterizationMode::Foliage): {
      std::vector<ClusterTriangle> operatingTriangles = CollectTriangles();
      clusters = StochasticClusterize(std::move(operatingTriangles), clusterize_settings);
    } break;
    case static_cast<unsigned>(ClusterizationMode::Original): {
      std::vector<ClusterTriangle> operatingTriangles = CollectTriangles();
      clusters = DefaultClusterize(std::move(operatingTriangles), clusterize_settings);
    } break;
  }
}

#pragma endregion

bool BillboardCloud::OriginalClusterizationSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Original clusterization settings")) {
    if (ImGui::DragFloat("Epsilon percentage", &epsilon_percentage, 0.01f, 0.01f, 1.f))
      changed = true;
    if (ImGui::DragInt("Discretization size", &discretization_size, 1, 1, 1000))
      changed = true;
    if (ImGui::DragInt("Timeout", &timeout, 1, 1, 1000))
      changed = true;

    ImGui::Checkbox("Skip remaining triangles", &skip_remain_triangles);
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::FoliageClusterizationSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Foliage clusterization settings")) {
    if (ImGui::DragFloat("Complexity", &density, 0.01f, 0.0f, 0.95f))
      changed = true;
    if (ImGui::DragInt("Iteration", &iteration, 1, 1, 1000))
      changed = true;
    if (ImGui::DragInt("Timeout", &timeout, 1, 1, 1000))
      changed = true;
    if (ImGui::DragFloat("Density", &sample_range, 0.01f, 0.1f, 2.f))
      changed = true;

    if (ImGui::Checkbox("Fill band", &fill_band))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::ClusterizationSettings::OnInspect() {
  bool changed = false;

  if (ImGui::TreeNode("Clusterization settings")) {
    if (ImGui::Combo("Clusterize mode", {"FlipBook", "Original", "Foliage"}, clusterize_mode)) {
      changed = true;
    }
    switch (clusterize_mode) {
      case static_cast<unsigned>(ClusterizationMode::FlipBook):
        break;
      case static_cast<unsigned>(ClusterizationMode::Foliage): {
        if (foliage_clusterization_settings.OnInspect())
          changed = true;
      } break;
      case static_cast<unsigned>(ClusterizationMode::Original): {
        if (original_clusterization_settings.OnInspect())
          changed = true;
      } break;
    }
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::ProjectSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Project settings")) {
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::JoinSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Join settings")) {
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::RasterizeSettings::OnInspect() {
  bool changed = false;
  if (ImGui::TreeNode("Rasterize settings")) {
    if (ImGui::Checkbox("(Debug) Full Fill", &debug_full_fill))
      changed = true;
    if (ImGui::Checkbox("(Debug) Opaque", &debug_opaque))
      changed = true;
    if (ImGui::Checkbox("Transfer albedo texture", &transfer_albedo_map))
      changed = true;
    if (ImGui::Checkbox("Transfer normal texture", &transfer_normal_map))
      changed = true;
    if (ImGui::Checkbox("Transfer roughness texture", &transfer_roughness_map))
      changed = true;
    if (ImGui::Checkbox("Transfer metallic texture", &transfer_metallic_map))
      changed = true;
    if (ImGui::Checkbox("Transfer ao texture", &transfer_ao_map))
      changed = true;
    if (ImGui::DragInt2("Resolution", &resolution.x, 1, 1, 8192))
      changed = true;

    if (ImGui::DragInt("Dilate", &dilate, 1, -1, 1024))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

bool BillboardCloud::GenerateSettings::OnInspect(const std::string& title) {
  bool changed = false;
  if (ImGui::TreeNodeEx(title.c_str())) {
    if (clusterization_settings.OnInspect())
      changed = true;
    if (project_settings.OnInspect())
      changed = true;
    if (join_settings.OnInspect())
      changed = true;
    if (rasterize_settings.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}
