#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
#include "BranchMeshGenerator.hpp"
using namespace UniEngine;
namespace EcoSysLab {
	class TreeVisualizationLayer : public ILayer {
		bool m_displayBranches = true;
		bool m_displayRootFlows = false;
		bool m_displayBoundingBox = false;
		bool m_debugVisualization = true;
		bool m_rendering = false;
		std::vector<int> m_versions;
		std::vector<glm::vec3> m_randomColors;
		
		std::vector<glm::uint> m_branchSegments;
		std::vector<StrandPoint> m_branchPoints;
		std::vector<glm::uint> m_rootSegments;
		std::vector<StrandPoint> m_rootPoints;

		AssetRef m_branchStrands;
		AssetRef m_rootStrands;

		std::vector<glm::mat4> m_rootFlowMatrices;
		std::vector<glm::vec4> m_rootFlowColors;

		std::vector<glm::mat4> m_boundingBoxMatrices;
		std::vector<glm::vec4> m_boundingBoxColors;

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
		std::unique_ptr<OpenGLUtils::GLBuffer> m_treeBranchBuffer;
		std::shared_ptr<Mesh> m_treeMesh;
	public:
		MeshGeneratorSettings m_meshGeneratorSettings;
		Entity m_selectedTree = {};
		TreeVisualizer m_treeVisualizer;
		void GrowAllTrees();
		void GenerateMeshes(const MeshGeneratorSettings& meshGeneratorSettings);

	};
}