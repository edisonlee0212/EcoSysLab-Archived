#pragma once


#include "Mesh.hpp"
#include "Material.hpp"

namespace EvoEngine {
	class BillboardCloud {

	public:
		static void ProjectToPlane(const Vertex& v0, const Vertex& v1, const Vertex& v2,
			Vertex& pV0, Vertex& pV1, Vertex& pV2,
			const glm::mat4& transform);

		struct RenderContent {
			std::shared_ptr<Mesh> m_mesh;
			std::shared_ptr<Material> m_material;

			std::vector<glm::uvec3> m_triangles;
		};

		struct InstancedRenderContent {
			std::shared_ptr<ParticleInfoList> m_particleInfoList;
			std::shared_ptr<Mesh> m_mesh;
			std::shared_ptr<Material> m_material;

			std::vector<glm::uvec3> m_triangles;
		};

		struct Element {
			RenderContent m_content{};
			Transform m_modelSpaceTransform{};
		};

		struct InstancedElement {
			InstancedRenderContent m_content{};
			Transform m_modelSpaceTransform{};
		};

		struct ProjectedRenderContent
		{
			std::shared_ptr<Mesh> m_mesh;
			std::shared_ptr<Material> m_material;
		};

		struct ProjectedElement {
			ProjectedRenderContent m_content{};
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
		};

		//Adopted from https://github.com/DreamVersion/RotatingCalipers
		class RotatingCalipers
		{
		public:
			static std::vector<glm::vec2> ConvexHull(std::vector<glm::vec2> points);
			static Rectangle GetMinAreaRectangle(std::vector<glm::vec2>&& points);
		};

		struct ProjectedCluster
		{
			glm::vec3 m_clusterCenter = glm::vec3(0.0f);
			glm::vec3 m_planeNormal = glm::vec3(1, 0, 0);
			glm::vec3 m_planeYAxis = glm::vec3(0, 1, 0);
			std::vector<ProjectedElement> m_elements;

			Rectangle m_billboardRectangle;
			std::shared_ptr<Mesh> m_billboardMesh;
			std::shared_ptr<Material> m_billboardMaterial;
		};

		struct Cluster {
			glm::vec3 m_clusterCenter = glm::vec3(0.0f);
			glm::vec3 m_planeNormal = glm::vec3(1, 0, 0);
			glm::vec3 m_planeYAxis = glm::vec3(0, 1, 0);
			std::vector<Element> m_elements;
			std::vector<InstancedElement> m_instancedElements;
		};

		struct ProjectSettings
		{
			glm::uvec2 m_resolution;

		};

		struct JoinSettings
		{
			glm::uvec2 m_resolution;

		};

		static ProjectedCluster Project(const Cluster& cluster, const ProjectSettings& settings);

		static RenderContent Join(const Cluster& cluster, const JoinSettings& settings);
	};
}