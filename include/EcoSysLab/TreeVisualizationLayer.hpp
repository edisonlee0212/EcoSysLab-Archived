#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
#include "BranchMeshGenerator.hpp"
using namespace UniEngine;
namespace EcoSysLab {
	class TreeVisualizationLayer : public ILayer {
		bool m_displayBranches = true;
		bool m_displayRootFlows = true;
		bool m_displayBoundingBox = false;
		bool m_debugVisualization = true;
		bool m_rendering = true;
		std::vector<int> m_versions;
		std::vector<glm::vec3> m_randomColors;
		std::vector<glm::mat4> m_branchMatrices;
		std::vector<glm::vec4> m_branchColors;

		std::vector<glm::mat4> m_rootFlowMatrices;
		std::vector<glm::vec4> m_rootFlowColors;

		std::vector<glm::mat4> m_boundingBoxMatrices;
		std::vector<glm::vec4> m_boundingBoxColors;

		glm::ivec2 m_gridSize = { 32, 32 };
		glm::vec2 m_gridDistance = { 10, 10 };
		float m_lastUsedTime = 0.0f;
		float m_totalTime = 0.0f;
		int m_internodeSize = 0;
		int m_branchSize = 0;
		int m_rootNodeSize = 0;
		int m_rootFlowSize = 0;
		bool m_needFlowUpdate = false;
		bool m_lockTreeSelection = false;
		bool m_autoGrow = false;

		void FixedUpdate() override;

		void OnCreate() override;

		void OnDestroy() override;

		void LateUpdate() override;

		void OnInspect() override;

		std::shared_ptr<OpenGLUtils::GLShader> m_treeBranchComp;
		std::shared_ptr<OpenGLUtils::GLProgram> m_treeBranchComputeProgram;
		std::shared_ptr<OpenGLUtils::GLVBO> m_treeBranchBuffer;

	public:
		MeshGeneratorSettings m_meshGeneratorSettings;
		Entity m_selectedTree = {};
		TreeVisualizer m_treeVisualizer;
		void GrowAllTrees();
		void GenerateMeshes(const MeshGeneratorSettings& meshGeneratorSettings);
	};
}