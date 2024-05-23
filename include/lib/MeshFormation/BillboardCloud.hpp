#pragma once


#include "Mesh.hpp"
#include "Material.hpp"

namespace EvoEngine {
	class BillboardCloud {
	public:
		struct RenderContent {
			std::shared_ptr<Mesh> m_mesh;
			std::shared_ptr<Material> m_material;

			std::vector<glm::uvec3> m_triangles;
		};

		struct Element {
			std::shared_ptr<RenderContent> m_content{};
			Transform m_modelSpaceTransform{};
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

		struct Cluster {
			glm::vec3 m_clusterCenter = glm::vec3(0.0f);
			glm::vec3 m_planeNormal = glm::vec3(0, 0, 1);
			glm::vec3 m_planeYAxis = glm::vec3(0, 1, 0);
			std::vector<Element> m_elements;
		};

		struct Billboard
		{
			glm::vec3 m_center = glm::vec3(0.0f);
			glm::vec3 m_planeNormal = glm::vec3(0, 0, 1);
			glm::vec3 m_planeYAxis = glm::vec3(0, 1, 0);

			Rectangle m_rectangle;
			std::shared_ptr<Mesh> m_mesh;
			std::shared_ptr<Material> m_material;
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

		struct ProjectSettings
		{
			bool m_transferAlbedoMap = true;
			bool m_transferNormalMap = true;
			bool m_transferRoughnessMap = true;
			bool m_transferMetallicMap = true;
			bool m_transferAoMap = true;

			float m_resolutionFactor = 128.f;
		};

		struct JoinSettings
		{
			glm::uvec2 m_resolution;

		};

		static Billboard Project(const Cluster& cluster, const ProjectSettings& projectSettings);

		std::vector<Cluster> m_clusters {};
		std::vector<Billboard> m_billboards {};

		[[nodiscard]] static std::vector<Cluster> Clusterize(const Cluster& targetCluster, const ClusterizeSettings& clusterizeSettings);

		void BuildClusters(const std::shared_ptr<Prefab> &prefab, const ClusterizeSettings& clusterizeSettings, bool combine);
		void ProjectClusters(const ProjectSettings& projectSettings);

		[[nodiscard]] Entity BuildEntity(const std::shared_ptr<Scene>& scene);
	private:
		static void PreprocessPrefab(std::vector<Cluster>& clusters, const std::shared_ptr<Prefab> &currentPrefab, const Transform& parentModelSpaceTransform);
		static void PreprocessPrefab(Cluster& combinedCluster, const std::shared_ptr<Prefab> &currentPrefab, const Transform& parentModelSpaceTransform);

		static void ProjectToPlane(const Vertex& v0, const Vertex& v1, const Vertex& v2,
			Vertex& pV0, Vertex& pV1, Vertex& pV2,
			const glm::mat4& transform);

		//Adopted from https://github.com/DreamVersion/RotatingCalipers
		class RotatingCalipers
		{
		public:
			static std::vector<glm::vec2> ConvexHull(std::vector<glm::vec2> points);
			static Rectangle GetMinAreaRectangle(std::vector<glm::vec2> points);
		};
	};
}