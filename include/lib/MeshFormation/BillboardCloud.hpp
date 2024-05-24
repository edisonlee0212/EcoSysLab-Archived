#pragma once


#include "Mesh.hpp"
#include "Material.hpp"

namespace EvoEngine {
	class BillboardCloud {
	public:
		struct Element {
			std::vector<Vertex> m_vertices;
			std::shared_ptr<Material> m_material;
			std::vector<glm::uvec3> m_triangles;
			[[nodiscard]] glm::vec3 CalculateCentroid(const glm::uvec3& triangle) const;
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

		struct ClusterTriangle
		{
			int m_elementIndex = -1;
			int m_triangleIndex = -1;
		};

		struct Cluster
		{
			glm::vec3 m_clusterCenter = glm::vec3(0.0f);
			glm::vec3 m_planeNormal = glm::vec3(0, 0, 1);
			glm::vec3 m_planeYAxis = glm::vec3(0, 1, 0);

			std::vector<ClusterTriangle> m_triangles;

			Rectangle m_rectangle;
			std::shared_ptr<Mesh> m_mesh;
			std::shared_ptr<Material> m_material;
		};

		struct ProjectSettings
		{
			bool m_transferAlbedoMap = true;
			bool m_transferNormalMap = true;
			bool m_transferRoughnessMap = true;
			bool m_transferMetallicMap = true;
			bool m_transferAoMap = true;

			float m_resolutionFactor = 128.f;
		};

		enum class ClusterizeMode
		{
			PassThrough,
			Default,
			Stochastic
		};

		struct ClusterizeSettings
		{
			bool m_append = true;
			ClusterizeMode m_clusterizeMode = ClusterizeMode::PassThrough;
		};

		class ElementCollection {
			void Project(Cluster& cluster, const ProjectSettings& projectSettings);

		public:
			std::vector<Element> m_elements;
			std::vector<Cluster> m_clusters;

			void Clusterize(const ClusterizeSettings& clusterizeSettings);
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

		void BuildClusters(const std::shared_ptr<Prefab> &prefab, const ClusterizeSettings& clusterizeSettings, bool combine);

		void ProjectClusters(const ProjectSettings& projectSettings);

		[[nodiscard]] Entity BuildEntity(const std::shared_ptr<Scene>& scene) const;
	private:
		[[nodiscard]] static std::vector<Cluster> StochasticClusterize(const ElementCollection& elementCollection, const ClusterizeSettings& clusterizeSettings);

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