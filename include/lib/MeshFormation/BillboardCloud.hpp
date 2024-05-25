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
			[[nodiscard]] glm::vec3 CalculateNormal(int triangleIndex) const;

			[[nodiscard]] glm::vec3 CalculateCentroid(const glm::uvec3& triangle) const;
			[[nodiscard]] float CalculateArea(const glm::uvec3& triangle) const;
			[[nodiscard]] glm::vec3 CalculateNormal(const glm::uvec3& triangle) const;
		};

		struct Rectangle
		{
			glm::vec2 m_points[4];

			void Update();

			glm::vec2 m_center;
			glm::vec2 m_xAxis;
			glm::vec2 m_yAxis;

			float m_width;
			float m_height;

			[[nodiscard]] glm::vec2 Transform(const glm::vec2 &target) const;
			[[nodiscard]] glm::vec3 Transform(const glm::vec3 &target) const;
		};

		

		struct Cluster
		{
			Plane m_clusterPlane = Plane(glm::vec3(0, 0, 1), 0.f);

			

			std::vector<ClusterTriangle> m_triangles;

			/**
			 * Billboard's bounding rectangle.
			 */
			Rectangle m_rectangle;
			/**
			 * Billboard's corresponding mesh.
			 */
			std::shared_ptr<Mesh> m_billboardMesh;
			/**
			 * Billboard's corresponding material.
			 */
			std::shared_ptr<Material> m_billboardMaterial;

			
		};

		struct ProjectSettings
		{
			bool m_transferAlbedoMap = true;
			bool m_transferNormalMap = true;
			bool m_transferRoughnessMap = false;
			bool m_transferMetallicMap = false;
			bool m_transferAoMap = false;

			float m_resolutionFactor = 128;
		};

		enum class ClusterizationMode
		{
			PassThrough,
			Default,
			Stochastic
		};

		struct StochasticClusterizationSettings
		{
			float m_epsilon = 0.05f;
			int m_iteration = 500;
			int m_timeout = 200;
			float m_maxPlaneSize = 0.5f;
		};

		struct ClusterizationSettings
		{
			bool m_append = true;
			StochasticClusterizationSettings m_stochasticClusterizationSettings {};
			ClusterizationMode m_clusterizeMode = ClusterizationMode::PassThrough;
		};

		class ElementCollection {
			void Project(Cluster& cluster, const ProjectSettings& projectSettings) const;
			std::vector<Cluster> StochasticClusterize(std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings);
			std::vector<Cluster> DefaultClusterize(std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings);

		public:
			std::vector<ClusterTriangle> m_skippedTriangles;

			std::vector<Element> m_elements;
			std::vector<Cluster> m_clusters;

			[[nodiscard]] glm::vec3 CalculateCentroid(const ClusterTriangle& triangle) const;
			[[nodiscard]] float CalculateArea(const ClusterTriangle& triangle) const;
			[[nodiscard]] glm::vec3 CalculateNormal(const ClusterTriangle& triangle) const;

			void Clusterize(const ClusterizationSettings& clusterizeSettings);
			void Project(const ProjectSettings& projectSettings);
		};

		struct JoinSettings
		{
			glm::uvec2 m_resolution;
		};

		/**
		 * Each element collection corresponding to a group of mesh material combinations.
		 * The primitives are grouped together to be clustered.
		 */
		std::vector<ElementCollection> m_elementCollections {};

		void BuildClusters(const std::shared_ptr<Prefab> &prefab, const ClusterizationSettings& clusterizeSettings, bool combine);

		void ProjectClusters(const ProjectSettings& projectSettings);

		[[nodiscard]] Entity BuildEntity(const std::shared_ptr<Scene>& scene) const;
	private:
		
		static void PreprocessPrefab(std::vector<ElementCollection>& elementCollections, const std::shared_ptr<Prefab> &currentPrefab, const Transform& parentModelSpaceTransform);
		static void PreprocessPrefab(ElementCollection& elementCollection, const std::shared_ptr<Prefab> &currentPrefab, const Transform& parentModelSpaceTransform);

		//Adopted from https://github.com/DreamVersion/RotatingCalipers
		class RotatingCalipers
		{
		public:
			static std::vector<glm::vec2> ConvexHull(std::vector<glm::vec2> points);
			static Rectangle GetMinAreaRectangle(std::vector<glm::vec2> points);
		};
	};
}