#pragma once

#include "Vertex.hpp"
#include "StrandModelData.hpp"
#include "TreeMeshGenerator.hpp"
#include "StrandModel.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	enum class StrandModelMeshGeneratorType
	{
		RecursiveSlicing,
		MarchingCube
	};

	struct StrandModelMeshGeneratorSettings
	{
		unsigned m_generatorType = static_cast<unsigned>(StrandModelMeshGeneratorType::RecursiveSlicing);
#pragma region Recursive Slicing
		int m_stepsPerSegment = 4;
		// this is for debugging purposes only and should not be used to obtain a proper mesh
		//bool m_limitProfileIterations = false;
		//int m_maxProfileIterations = 20;
		float m_maxParam = std::numeric_limits<float>::infinity();
		bool m_branchConnections = true;
		int m_uMultiplier = 2;
		float m_vMultiplier = 0.25;
		float m_clusterDistance = 1.0f;
#pragma endregion

#pragma region Hybrid MarchingCube
		bool m_removeDuplicate = true;
		
		bool m_autoLevel = true;
		int m_voxelSubdivisionLevel = 10;
		float m_marchingCubeRadius = 0.002f;
		float m_xSubdivision = 0.03f;
		float m_ySubdivision = 0.03f;
		glm::vec4 m_marchingCubeColor = glm::vec4(0.6, 0.3, 0.0f, 1.0f);
		glm::vec4 m_cylindricalColor = glm::vec4(0.1, 0.9, 0.0f, 1.0f);

		int m_rootDistanceMultiplier = 10;
		float m_circleMultiplier = 1.f;
#pragma endregion

		bool m_recalculateUV = false;
		bool m_fastUV = true;
		int m_smoothIteration = 0;
		int m_minCellCountForMajorBranches = 100;
		int m_maxCellCountForMinorBranches = 150;
		bool m_enableBranch = true;
		bool m_enableFoliage = true;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};
	class StrandModelMeshGenerator
	{
		static void RecursiveSlicing(
			const StrandModel& strandModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

		static void RecursiveSlicing(
			const StrandModel& strandModel, std::vector<Vertex>& vertices,
			std::vector<glm::vec2>& texCoords,
			std::vector<std::pair<unsigned int, unsigned int>>& indices, const StrandModelMeshGeneratorSettings& settings);

		static void MarchingCube(
			const StrandModel& strandModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

		static void CylindricalMeshing(const StrandModel& strandModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

		static void MeshSmoothing(std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices);

		static void MeshSmoothing(std::vector<Vertex>& vertices,
			std::vector<std::pair<unsigned int, unsigned int>>& indices);

		static void CalculateNormal(std::vector<Vertex>& vertices,
			const std::vector<unsigned int>& indices);

		static void CalculateNormal(std::vector<Vertex>& vertices,
			const std::vector<std::pair<unsigned int, unsigned int>>& indices);

		static void CalculateUV(const StrandModel& strandModel, std::vector<Vertex>& vertices, const StrandModelMeshGeneratorSettings& settings);
	public:
		static void Generate(
			const StrandModel& strandModel, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);
		static void Generate(
			const StrandModel& strandModel, std::vector<Vertex>& vertices,
			std::vector<glm::vec2>& texCoords,
			std::vector<std::pair<unsigned int, unsigned int>>& indices, const StrandModelMeshGeneratorSettings& settings);
	};
}