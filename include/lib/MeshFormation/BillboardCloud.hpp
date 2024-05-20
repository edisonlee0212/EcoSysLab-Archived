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
			Plane m_modelSpaceProjectionPlane;
			std::vector<Element> m_elements;
			std::vector<InstancedElement> m_instancedElements;
		};

		struct BillboardProjectSettings
		{
		};
		static Cluster Project(const Cluster& cluster, const BillboardProjectSettings& settings);
	};
}