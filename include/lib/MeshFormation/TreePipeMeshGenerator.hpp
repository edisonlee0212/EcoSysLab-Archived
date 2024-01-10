#pragma once

#include "Vertex.hpp"
#include "PipeModelData.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	struct TreePipeMeshGeneratorSettings
	{
		glm::vec3 m_vertexColor = glm::vec3(1.0f);
		bool m_removeDuplicate = true;
		int m_voxelSmoothIteration = 10;
		bool m_autoLevel = true;
		int m_voxelSubdivisionLevel = 10;
		float m_marchingCubeRadius = 0.01f;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};
	class TreePipeMeshGenerator
	{
	public:
		static void Generate(const
			PipeModelPipeGroup& pipes, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreePipeMeshGeneratorSettings& settings);
	};
}