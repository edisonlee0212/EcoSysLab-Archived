#pragma once

#include "Vertex.hpp"
#include "PipeModelData.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	struct TreePipeMeshGeneratorSettings
	{
		//TODO: Add parameters here.
		glm::vec3 m_vertexColor = glm::vec3(1.0f);
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