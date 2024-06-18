#pragma once

#include "Material.hpp"
#include "Mesh.hpp"

namespace evo_engine {
class BillboardCloud {
 public:
  struct ClusterTriangle {
    int element_index = -1;
    int triangle_index = -1;

    int index;
  };
  struct ProjectedTriangle {
    Vertex projected_vertices[3];
    Handle material_handle;
  };

  struct Element {
    /**
     * Vertices must be in model space.
     */
    std::vector<Vertex> vertices;
    std::shared_ptr<Material> material;
    std::vector<glm::uvec3> triangles;

    [[nodiscard]] glm::vec3 CalculateCentroid(int triangle_index) const;
    [[nodiscard]] float CalculateArea(int triangle_index) const;
    [[nodiscard]] float CalculateNormalDistance(int triangle_index) const;
    [[nodiscard]] glm::vec3 CalculateNormal(int triangle_index) const;

    [[nodiscard]] glm::vec3 CalculateCentroid(const glm::uvec3& triangle) const;
    [[nodiscard]] float CalculateArea(const glm::uvec3& triangle) const;
    [[nodiscard]] glm::vec3 CalculateNormal(const glm::uvec3& triangle) const;
    [[nodiscard]] float CalculateNormalDistance(const glm::uvec3& triangle) const;

    [[nodiscard]] std::vector<std::vector<unsigned>> CalculateLevelSets(const glm::vec3& direction = glm::vec3(0, 1,
                                                                                                               0));
  };
  struct BoundingSphere {
    glm::vec3 center = glm::vec3(0.f);
    float radius = 0.f;

    void Initialize(const std::vector<Element>& elements);
  };

  struct Rectangle {
    glm::vec2 points[4];

    glm::vec2 tex_coords[4];

    void Update();

    glm::vec2 center;
    glm::vec2 x_axis;
    glm::vec2 y_axis;

    float width;
    float height;

    [[nodiscard]] glm::vec2 Transform(const glm::vec2& target) const;
    [[nodiscard]] glm::vec3 Transform(const glm::vec3& target) const;
  };
  struct ProjectSettings {
    bool OnInspect();
  };

  struct JoinSettings {
    bool OnInspect();
  };

  struct RasterizeSettings {
    bool debug_full_fill = false;
    bool debug_opaque = false;
    bool transfer_albedo_map = true;
    bool transfer_normal_map = true;
    bool transfer_roughness_map = true;
    bool transfer_metallic_map = true;
    bool transfer_ao_map = false;

    int dilate = -1;

    glm::ivec2 resolution = glm::ivec2(2048);

    bool OnInspect();
  };

  enum class ClusterizationMode {
    FlipBook,
    Original,
    Foliage,
  };

  struct FoliageClusterizationSettings {
    float density = 0.9f;
    int iteration = 400;
    int timeout = 0;
    float sample_range = 1.f;

    bool fill_band = true;
    bool OnInspect();
  };
  struct OriginalClusterizationSettings {
    float epsilon_percentage = 0.01f;
    int discretization_size = 10;
    int timeout = 0;
    bool skip_remain_triangles = false;
    bool OnInspect();
  };

  struct ClusterizationSettings {
    bool append = true;
    FoliageClusterizationSettings foliage_clusterization_settings{};
    OriginalClusterizationSettings original_clusterization_settings{};
    unsigned clusterize_mode = static_cast<unsigned>(ClusterizationMode::Foliage);

    bool OnInspect();
  };

  struct GenerateSettings {
    ClusterizationSettings clusterization_settings{};
    ProjectSettings project_settings{};
    JoinSettings join_settings{};
    RasterizeSettings rasterize_settings{};

    bool OnInspect(const std::string& title);
  };

  struct Cluster {
    Plane cluster_plane = Plane(glm::vec3(0, 0, 1), 0.f);

    std::vector<ClusterTriangle> triangles;

    std::vector<ProjectedTriangle> projected_triangles;
    /**
     * Billboard's bounding rectangle.
     */
    Rectangle rectangle;
    /**
     * Billboard's corresponding mesh.
     */
    std::vector<Vertex> billboard_vertices;
    std::vector<glm::uvec3> billboard_triangles;
  };

  std::vector<ClusterTriangle> CollectTriangles() const;

  std::vector<ClusterTriangle> skipped_triangles;

  std::vector<Element> elements;
  std::vector<Cluster> clusters;

  std::shared_ptr<Mesh> billboard_cloud_mesh;
  std::shared_ptr<Material> billboard_cloud_material;

  void Clusterize(const ClusterizationSettings& clusterize_settings);
  void Project(const ProjectSettings& project_settings);
  void Join(const JoinSettings& join_settings);
  void Rasterize(const RasterizeSettings& rasterize_settings);

  void Generate(const GenerateSettings& generate_settings);
  void Process(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material);
  void Process(const std::shared_ptr<Prefab>& prefab);
  void Process(const std::shared_ptr<Scene>& scene, const Entity& entity);

  [[nodiscard]] Entity BuildEntity(const std::shared_ptr<Scene>& scene) const;

  [[nodiscard]] std::vector<glm::vec3> ExtractPointCloud(float density) const;

 private:
  [[nodiscard]] glm::vec3 CalculateCentroid(const ClusterTriangle& triangle) const;
  [[nodiscard]] float CalculateArea(const ClusterTriangle& triangle) const;
  [[nodiscard]] float CalculateNormalDistance(const ClusterTriangle& triangle) const;
  [[nodiscard]] glm::vec3 CalculateNormal(const ClusterTriangle& triangle) const;
  void Project(Cluster& cluster, const ProjectSettings& project_settings) const;
  std::vector<Cluster> StochasticClusterize(std::vector<ClusterTriangle> operating_triangles,
                                            const ClusterizationSettings& clusterize_settings);
  std::vector<Cluster> DefaultClusterize(std::vector<ClusterTriangle> operating_triangles,
                                         const ClusterizationSettings& clusterize_settings);

  void ProcessPrefab(const std::shared_ptr<Prefab>& current_prefab, const Transform& parent_model_space_transform);
  void ProcessEntity(const std::shared_ptr<Scene>& scene, const Entity& entity,
                     const Transform& parent_model_space_transform);

  // Adopted from https://github.com/DreamVersion/RotatingCalipers
  class RotatingCalipers {
   public:
    static std::vector<glm::vec2> ConvexHull(std::vector<glm::vec2> points);
    static Rectangle GetMinAreaRectangle(std::vector<glm::vec2> points);
  };
};
}  // namespace EvoEngine