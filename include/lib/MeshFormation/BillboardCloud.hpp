#pragma once


#include "Mesh.hpp"
#include "Material.hpp"

namespace EvoEngine {
	class BillboardCloud {
	public:
		struct ClusterTriangle
		{
			int m_elementIndex = -1;
			int m_triangleIndex = -1;

			int m_index;
		};
		struct ProjectedTriangle
		{
			Vertex m_projectedVertices[3];
			Handle m_materialHandle;
		};

		struct Element {
			/**
			 * Vertices must be in model space.
			 */
			std::vector<Vertex> m_vertices;
			std::shared_ptr<Material> m_material;
			std::vector<glm::uvec3> m_triangles;

			[[nodiscard]] glm::vec3 CalculateCentroid(int triangleIndex) const;
			[[nodiscard]] float CalculateArea(int triangleIndex) const;
			[[nodiscard]] float CalculateNormalDistance(int triangleIndex) const;
			[[nodiscard]] glm::vec3 CalculateNormal(int triangleIndex) const;

			[[nodiscard]] glm::vec3 CalculateCentroid(const glm::uvec3& triangle) const;
			[[nodiscard]] float CalculateArea(const glm::uvec3& triangle) const;
			[[nodiscard]] glm::vec3 CalculateNormal(const glm::uvec3& triangle) const;
			[[nodiscard]] float CalculateNormalDistance(const glm::uvec3& triangle) const;
		};

		struct Rectangle
		{
			glm::vec2 m_points[4];

			glm::vec2 m_texCoords[4];

			void Update();

			glm::vec2 m_center;
			glm::vec2 m_xAxis;
			glm::vec2 m_yAxis;

			float m_width;
			float m_height;

			[[nodiscard]] glm::vec2 Transform(const glm::vec2& target) const;
			[[nodiscard]] glm::vec3 Transform(const glm::vec3& target) const;
		};
		struct ProjectSettings
		{
			bool OnInspect();
		};

		struct JoinSettings
		{
			bool OnInspect();
		};

		struct RasterizeSettings
		{
			bool m_transferAlbedoMap = true;
			bool m_transferNormalMap = true;
			bool m_transferRoughnessMap = true;
			bool m_transferMetallicMap = true;
			bool m_transferAoMap = false;

			glm::ivec2 m_resolution = glm::ivec2(2048);

			bool OnInspect();
		};

		enum class ClusterizationMode
		{
			PassThrough,
			Original,
			Stochastic
		};

		struct StochasticClusterizationSettings
		{
			float m_epsilon = 0.3f;
			int m_iteration = 200;
			int m_timeout = 500;
			float m_maxPlaneSize = 1.f;

			bool OnInspect();
		};
		struct OriginalClusterizationSettings
		{
			float m_epsilonPercentage = 0.01f;
			int m_discretizationSize = 10;
			int m_timeout = 300;

			bool OnInspect();
		};

		struct ClusterizationSettings
		{
			bool m_append = true;
			StochasticClusterizationSettings m_stochasticClusterizationSettings{};
			OriginalClusterizationSettings m_originalClusterizationSettings{};
			unsigned m_clusterizeMode = static_cast<unsigned>(ClusterizationMode::PassThrough);

			bool OnInspect();
		};

		struct GenerateSettings
		{
			ClusterizationSettings m_clusterizationSettings{};
			ProjectSettings m_projectSettings{};
			JoinSettings m_joinSettings {};
			RasterizeSettings m_rasterizeSettings {};

			bool OnInspect(const std::string& title);
		};

		struct Cluster
		{
			Plane m_clusterPlane = Plane(glm::vec3(0, 0, 1), 0.f);

			std::vector<ClusterTriangle> m_triangles;

			std::vector<ProjectedTriangle> m_projectedTriangles;
			/**
			 * Billboard's bounding rectangle.
			 */
			Rectangle m_rectangle;
			/**
			 * Billboard's corresponding mesh.
			 */
			std::vector<Vertex> m_billboardVertices;
			std::vector<glm::uvec3> m_billboardTriangles;

		};

		std::vector<ClusterTriangle> CollectTriangles() const;

		std::vector<ClusterTriangle> m_skippedTriangles;

		std::vector<Element> m_elements;
		std::vector<Cluster> m_clusters;

		std::shared_ptr<Mesh> m_billboardCloudMesh;
		std::shared_ptr<Material> m_billboardCloudMaterial;

		void Clusterize(const ClusterizationSettings& clusterizeSettings);
		void Project(const ProjectSettings& projectSettings);
		void Join(const JoinSettings& joinSettings);
		void Rasterize(const RasterizeSettings& rasterizeSettings);

		void Generate(const GenerateSettings& generateSettings);

		void ProcessPrefab(const std::shared_ptr<Prefab>& prefab);

		[[nodiscard]] Entity BuildEntity(const std::shared_ptr<Scene>& scene) const;
	private:

		[[nodiscard]] glm::vec3 CalculateCentroid(const ClusterTriangle& triangle) const;
		[[nodiscard]] float CalculateArea(const ClusterTriangle& triangle) const;
		[[nodiscard]] float CalculateNormalDistance(const ClusterTriangle& triangle) const;
		[[nodiscard]] glm::vec3 CalculateNormal(const ClusterTriangle& triangle) const;
		void Project(Cluster& cluster, const ProjectSettings& projectSettings) const;
		std::vector<Cluster> StochasticClusterize(std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings);
		std::vector<Cluster> DefaultClusterize(std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings);

		void ProcessPrefab(const std::shared_ptr<Prefab>& currentPrefab, const Transform& parentModelSpaceTransform);
		//Adopted from https://github.com/DreamVersion/RotatingCalipers
		class RotatingCalipers
		{
		public:
			static std::vector<glm::vec2> ConvexHull(std::vector<glm::vec2> points);
			static Rectangle GetMinAreaRectangle(std::vector<glm::vec2> points);
		};
	};
}