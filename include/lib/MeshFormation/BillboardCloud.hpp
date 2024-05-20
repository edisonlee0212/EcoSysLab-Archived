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
		static Cluster Project(const Cluster& cluster, const ProjectSettings& settings);

		static RenderContent Join(const Cluster& cluster, const JoinSettings& settings);
	};
}