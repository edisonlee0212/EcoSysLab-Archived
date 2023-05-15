#pragma once

#include "ecosyslab_export.h"
#include "VoxelGrid.hpp"
#include "PlantStructure.hpp"
#include "TreeMeshGenerator.hpp"
#include "Curve.hpp"

using namespace UniEngine;
namespace EcoSysLab {
	typedef int PointHandle;
	typedef int BranchHandle;
	struct ScatteredPoint {
		PointHandle m_handle = -1;
		std::vector<PointHandle> m_neighbors;
		std::vector<BranchHandle> m_neighborBranchStarts;
		std::vector<BranchHandle> m_neighborBranchEnds;
		glm::vec3 m_position = glm::vec3(0.0f);
	};

	struct ScannedBranch {
		BranchHandle m_handle = -1;
		BezierCurve m_bezierCurve;
		float m_startThickness = 0.0f;
		float m_endThickness = 0.0f;
		std::vector<PointHandle> m_startNeighbors;
		std::vector<PointHandle> m_endNeighbors;

		std::vector<BranchHandle> m_neighborBranchStarts;
		std::vector<BranchHandle> m_neighborBranchEnds;
		BranchHandle m_parentHandle = -1;
		std::vector<BranchHandle> m_childHandles;

		std::vector<NodeHandle> m_chainNodeHandles;
	};

	struct TreePart {
		std::vector<glm::vec3> m_allocatedPoints;
		std::vector<BranchHandle> m_branchHandles;
	};

	struct ConnectivityGraphSettings {
		float m_finderStep = 0.05f;
		float m_edgeLength = 0.25f;
		int m_maxTimeout = 30;
		float m_junctionAngleLimit = 10.0f;
		float m_forceConnectionLength = 0.35f;
		float m_forceJunctionAngleLimit = 135.0f;
		void OnInspect();
	};

	enum class PointCloudVoxelType {
		ScatteredPoint,
		BranchStart,
		BranchEnd
	};

	struct PointCloudVoxel {
		PointCloudVoxelType m_type;
		glm::vec3 m_position;
		int m_handle;
	};

	struct ReconstructionSettings {
		float m_internodeLength = 0.03f;
		float m_minHeight = 0.1f;
		float m_maxTreeDistance = 0.05f;
	};

	class TreePointCloud : public IPrivateComponent {
		void FindPoints(const glm::vec3 &position, VoxelGrid<std::vector<PointCloudVoxel>> &pointVoxelGrid, float radius,
										const std::function<void(const PointCloudVoxel &voxel)> &func) const;

		public:
		void ImportGraph(const std::filesystem::path &path, float scaleFactor = 0.1f);

		glm::vec3 m_min;
		glm::vec3 m_max;
		std::vector<ScatteredPoint> m_points;
		std::vector<ScannedBranch> m_branches;
		std::vector<TreePart> m_treeParts;

		void OnInspect() override;

		std::vector<BaseSkeleton> m_skeletons;

		std::vector<std::pair<glm::vec3, glm::vec3>> m_scatterPointsConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_branchConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_filteredJunctionConnections;

		void EstablishConnectivityGraph(const ConnectivityGraphSettings &otherPointHandle);

		void BuildTreeStructure(const ReconstructionSettings &reconstructionSettings);

		void ClearMeshes() const;

		void GenerateMeshes(const TreeMeshGeneratorSettings &meshGeneratorSettings);
	};
}