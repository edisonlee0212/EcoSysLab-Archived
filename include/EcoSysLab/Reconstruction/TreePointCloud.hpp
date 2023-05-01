#pragma once

#include "ecosyslab_export.h"
#include "VoxelGrid.hpp"
#include "PlantStructure.hpp"
#include "TreeMeshGenerator.hpp"
using namespace UniEngine;
namespace EcoSysLab {
	typedef int PointHandle;
	typedef int JunctionHandle;
	struct ScannedPoint {
		PointHandle m_handle;
		std::vector<PointHandle> m_neighbors;
		JunctionHandle m_junctionHandle = -1;
		PointHandle m_prevHandle = -1;
		PointHandle m_nextHandle = -1;
		glm::vec3 m_position = glm::vec3(0.0f);
		float m_thickness = 0;
		NodeHandle m_parentNodeHandle = -1;
		NodeHandle m_nodeHandle = -1;
	};

	struct ScannedJunction{
		glm::vec3 m_start = glm::vec3(0.0f);
		glm::vec3 m_end = glm::vec3(0.0f);
		glm::vec3 m_center = glm::vec3(0.0f);
		glm::vec3 m_direction = glm::vec3(0.0f);

		PointHandle m_startHandle = -1;
		PointHandle m_endHandle = -1;
		bool m_endJunction = false;
		JunctionHandle m_parentHandle = -1;
		std::vector<JunctionHandle> m_childHandles;
	};

	struct ConnectivityGraphSettings {
		float m_finderStep = 0.05f;
		float m_edgeLength = 0.25f;
		int m_maxTimeout = 10;
		float m_junctionLimit = 10.0f;
	};

	class TreePointCloud : public IPrivateComponent {
		void FindPoints(PointHandle targetPoint, VoxelGrid<std::vector<PointHandle>> &pointVoxelGrid, float radius,
										const std::function<void(PointHandle handle)> &func) const;

		public:
		void ImportCsv(const std::filesystem::path &path);

		glm::vec3 m_min;
		glm::vec3 m_max;
		std::vector<ScannedPoint> m_points;
		std::vector<ScannedJunction> m_junctions;
		void OnInspect() override;

		std::vector<std::pair<PointHandle, PointHandle>> m_scatterPointsConnections;
		std::vector<std::pair<PointHandle, PointHandle>> m_junctionConnections;
		std::vector<std::pair<PointHandle, PointHandle>> m_filteredJunctionConnections;
		void EstablishConnectivityGraph(const ConnectivityGraphSettings &settings);
		BaseSkeleton BuildTreeStructure();

		void ClearMeshes() const;
		void GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings);
	};
}