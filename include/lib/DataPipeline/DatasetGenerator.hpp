#pragma once
#include "TreeMeshGenerator.hpp"
#include "TreePointCloudScanner.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class DatasetGenerator {
	public:
		static void GeneratePointCloudForTree(
			const PointCloudPointSettings& pointSettings,
			const PointCloudCircularCaptureSettings& captureSettings,
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
			const PointCloudPointSettings& pointSettings,
			const PointCloudGridCaptureSettings& captureSettings,
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