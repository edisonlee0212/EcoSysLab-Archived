#pragma once
#include "TreeMeshGenerator.hpp"
#include "TreePointCloudScanner.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class DatasetGenerator {
	public:
		static void GenerateTreeTrunkMesh(const std::string& treeParametersPath,
			float deltaTime,
			int maxIterations,
			int maxTreeNodeCount,
			const TreeMeshGeneratorSettings& meshGeneratorSettings, 
			const std::string& treeMeshOutputPath,
			const std::string& treeTrunkOutputPath,
			const std::string& treeInfoPath);

		static void GenerateTreeMesh(const std::string& treeParametersPath,
			float deltaTime,
			int maxIterations,
			int maxTreeNodeCount,
			const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::string& treeMeshOutputPath);

		static void GeneratePointCloudForTree(
			const TreePointCloudPointSettings& pointSettings,
			const TreePointCloudCircularCaptureSettings& captureSettings,
			const std::string& treeParametersPath,
			float deltaTime,
			int maxIterations,
			int maxTreeNodeCount,
			const TreeMeshGeneratorSettings& meshGeneratorSettings,
			const std::string& pointCloudOutputPath,
			bool exportTreeMesh,
			const std::string& treeMeshOutputPath,
			bool exportJunction,
			const std::string& treeJunctionOutputPath
		);
		static void GeneratePointCloudForForestPatch(
			int gridSize, float gridDistance, float randomShift,
			const TreePointCloudPointSettings& pointSettings,
			const TreePointCloudGridCaptureSettings& captureSettings,
			const std::string& treeParametersFolderPath,
			const std::string& forestPatchPath,
			float deltaTime,
			int maxIterations,
			int maxTreeNodeCount,
			const TreeMeshGeneratorSettings& meshGeneratorSettings,
			const std::string& pointCloudOutputPath
		);
	};

}