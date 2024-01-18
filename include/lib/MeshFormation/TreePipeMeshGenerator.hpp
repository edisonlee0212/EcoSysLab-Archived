#pragma once

#include "Vertex.hpp"
#include "PipeModelData.hpp"
#include "TreeModel.hpp"

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

		// this is for debugging purposes only and should not be used to obtain a proper mesh
		bool m_limitProfileIterations = false;
		int m_maxProfileIterations = 20;
		float m_maxParam = 1.0f;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};
	class TreePipeMeshGenerator
	{
	public:
		static void Generate(
			const TreeModel& treeModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreePipeMeshGeneratorSettings& settings);
	};
}