#pragma once

#include "Vertex.hpp"
#include "PipeModelData.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreeModel.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	enum class TreePipeMeshGeneratorType
	{
		RecursiveSlicing,
		HybridMarchingCube
	};

	struct PipeModelMeshGeneratorSettings
	{
		unsigned m_generatorType = static_cast<unsigned>(TreePipeMeshGeneratorType::RecursiveSlicing);
#pragma region Recursive Slicing
		int m_stepsPerSegment = 4;
		// this is for debugging purposes only and should not be used to obtain a proper mesh
		//bool m_limitProfileIterations = false;
		//int m_maxProfileIterations = 20;
		//float m_maxParam = 1.0f;
		bool m_branchConnections = true;
		int m_uMultiplier = 1;
		float m_vMultiplier = 1.0;
#pragma endregion

#pragma region Hybrid MarchingCube
		bool m_removeDuplicate = true;
		
		bool m_autoLevel = true;
		int m_voxelSubdivisionLevel = 10;
		float m_marchingCubeRadius = 0.003f;
		float m_xSubdivision = 0.03f;
		float m_ySubdivision = 0.03f;
		glm::vec4 m_marchingCubeColor = glm::vec4(0.6, 0.3, 0.0f, 1.0f);
		glm::vec4 m_cylindricalColor = glm::vec4(0.1, 0.9, 0.0f, 1.0f);
		float m_texCoordsMultiplier = 10.0f;
#pragma endregion
		int m_smoothIteration = 0;
		int m_minCellCountForMajorBranches = 50;
		int m_maxCellCountForMinorBranches = 200;
		bool m_enableBranch = true;
		bool m_enableFoliage = true;

		FoliageParameters m_foliageSettings = {};
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};
	class TreePipeMeshGenerator
	{
		static void RecursiveSlicing(
			const TreeModel& treeModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const PipeModelMeshGeneratorSettings& settings);
		static void MarchingCube(
			const TreeModel& treeModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const PipeModelMeshGeneratorSettings& settings);

		static void CylindricalMeshing(const TreeModel& treeModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const PipeModelMeshGeneratorSettings& settings);

		static void MeshSmoothing(std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices);

		static void CalculateNormal(std::vector<Vertex>& vertices,
			const std::vector<unsigned int>& indices);

		static void CalculateUV(std::vector<Vertex>& vertices, float factor = 1.0f);
	public:
		static void Generate(
			const TreeModel& treeModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const PipeModelMeshGeneratorSettings& settings);
	};
}