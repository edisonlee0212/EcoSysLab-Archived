#pragma once

#include "VoxelGrid.hpp"
#include "Skeleton.hpp"
#include "TreeMeshGenerator.hpp"
#include "Curve.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	typedef int PointHandle;
	typedef int BranchHandle;
	typedef int TreePartHandle;
	struct ScatteredPoint {
		PointHandle m_handle = -1;
		std::vector<PointHandle> m_neighborScatterPoints;
		std::vector<std::pair<float, BranchHandle>> m_p3;

		//For reversed branch
		std::vector<std::pair<float, BranchHandle>> m_p0;
		glm::vec3 m_position = glm::vec3(0.0f);
	};
	struct AllocatedPoint {
		glm::vec3 m_color;
		glm::vec3 m_position;

		glm::vec2 m_planePosition;
		float m_planeCenterDistance;
		PointHandle m_handle = -1;
		TreePartHandle m_treePartHandle = -1;
		BranchHandle m_branchHandle = -1;
		NodeHandle m_nodeHandle = -1;
		int m_skeletonIndex = -1;
	};
	struct PredictedBranch {
		glm::vec3 m_color;

		TreePartHandle m_treePartHandle = -1;
		BranchHandle m_handle = -1;
		BezierCurve m_bezierCurve;
		float m_startThickness = 0.0f;
		float m_endThickness = 0.0f;

		float m_branchThickness = 0.0f;
		std::vector<PointHandle> m_allocatedPoints;

		std::vector<std::pair<float, PointHandle>> m_pointsToP3;
		std::unordered_map<BranchHandle, float> m_p3ToP0;

		//For reversed branch
		std::vector<std::pair<float, PointHandle>> m_pointsToP0;
		std::unordered_map<BranchHandle, float> m_p3ToP3;
		std::unordered_map<BranchHandle, float> m_p0ToP0;
		std::unordered_map<BranchHandle, float> m_p0ToP3;
	};

	struct OperatorBranch {
		glm::vec3 m_color;

		TreePartHandle m_treePartHandle = -1;
		BranchHandle m_handle = -1;

		BranchHandle m_reversedBranchHandle = -1;

		BezierCurve m_bezierCurve;
		float m_thickness = 0.0f;

		BranchHandle m_parentHandle = -1;
		std::vector<BranchHandle> m_childHandles;
		BranchHandle m_largestChildHandle = -1;
		int m_skeletonIndex = -1;
		std::vector<NodeHandle> m_chainNodeHandles;

		float m_bestDistance = FLT_MAX;

		std::vector<std::pair<BranchHandle, float>> m_parentCandidates;
		bool m_used = false;
		bool m_orphan = false;

		bool m_apical = false;

		float m_distanceToParentBranch = 0.0f;
		float m_rootDistance = 0.0f;

		int m_descendentSize = 0;
	};

	struct TreePart {
		glm::vec3 m_color;

		TreePartHandle m_handle = -1;
		std::vector<PointHandle> m_allocatedPoints;
		std::vector<BranchHandle> m_branchHandles;
	};

	struct ConnectivityGraphSettings {
		bool m_reverseConnection = true;

		float m_pointCheckRadius = 0.05f;
		bool m_zigzagCheck = true;
		float m_zigzagBranchShortening = 0.1f;
		bool m_parallelShiftCheck = true;
		float m_parallelShiftLimitRange = 2.0f;
		float m_pointPointConnectionDetectionRadius = 0.05f;
		float m_pointBranchConnectionDetectionRange = 0.5f;
		float m_branchBranchConnectionMaxLengthRange = 5.0f;
		float m_directionConnectionAngleLimit = 45.0f;
		float m_indirectConnectionAngleLimit = 45.0f;

		float m_connectionRangeLimit = 1.0f;
		void OnInspect();
	};

	struct PointData {
		glm::vec3 m_position = glm::vec3(0.0f);
		glm::vec3 m_direction = glm::vec3(0.0f);
		int m_handle = -1;
		int m_index = -1;
		float m_minDistance = FLT_MAX;
	};
	struct BranchEndData {
		bool m_isP0 = true;
		glm::vec3 m_position = glm::vec3(0.0f);
		int m_branchHandle = -1;
	};

	struct ReconstructionSettings {
		float m_internodeLength = 0.03f;
		float m_minHeight = 0.3f;
		float m_minimumTreeDistance = 0.1f;
		float m_branchShortening = 0.3f;

		float m_endNodeThickness = 0.002f;
		float m_thicknessSumFactor = 0.4f;
		float m_thicknessAccumulationFactor = 0.00005f;
		float m_overrideThicknessRootDistance = 0.0f;

		int m_spaceColonizationTimeout = 10;
		float m_spaceColonizationFactor = 0.0f;
		float m_spaceColonizationRemovalDistanceFactor = 2;
		float m_spaceColonizationDetectionDistanceFactor = 4;
		float m_spaceColonizationTheta = 20.0f;

		int m_minimumNodeCount = 1;
		bool m_limitParentThickness = true;
		float m_minimumRootThickness = 0.02f;

		int m_nodeBackTrackLimit = 30;
		int m_branchBackTrackLimit = 1;

		/*
		bool m_candidateSearch = true;
		int m_candidateSearchLimit = 1;
		bool m_forceConnectAllBranches = false;
		*/
		bool m_useRootDistance = false;
		int m_optimizationTimeout = 999;

		float m_directionSmoothing = 0.1f;
		float m_positionSmoothing = 0.1f;
		int m_smoothIteration = 10;
		void OnInspect();
	};

	struct ReconstructionSkeletonData {
		glm::vec3 m_rootPosition = glm::vec3(0.0f);
		float m_maxEndDistance = 0.0f;
	};
	struct ReconstructionFlowData {

	};
	struct ReconstructionNodeData {
		glm::vec3 m_globalEndPosition = glm::vec3(0.0f);

		float m_draftThickness = 0.0f;
		float m_allocatedPointThickness = 0.0f;
		std::vector<PointHandle> m_allocatedPoints;
		std::vector<PointHandle> m_filteredPoints;
		BranchHandle m_branchHandle;

		bool m_regrowth = false;
		int m_markerSize = 0;
		glm::vec3 m_regrowDirection = glm::vec3(0.0f);
		
	};
	typedef Skeleton<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> ReconstructionSkeleton;

	class TreePointCloud : public IPrivateComponent {
		bool DirectConnectionCheck(const BezierCurve& parentCurve, const BezierCurve& childCurve, bool reverse);

		static void FindPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid, float radius,
		                       const std::function<void(const PointData& voxel)>& func);
		static bool HasPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid, float radius);
		static void ForEachBranchEnd(const glm::vec3& position, VoxelGrid<std::vector<BranchEndData>>& branchEndsVoxelGrid, float radius,
		                             const std::function<void(const BranchEndData& voxel)>& func);

		void CalculateNodeTransforms(ReconstructionSkeleton& skeleton);

		void BuildConnectionBranch(BranchHandle processingBranchHandle, NodeHandle& prevNodeHandle);

		void Unlink(BranchHandle childHandle, BranchHandle parentHandle);
		void Link(BranchHandle childHandle, BranchHandle parentHandle);

		void GetSortedBranchList(BranchHandle branchHandle, std::vector<BranchHandle>& list);

		void ConnectBranches(BranchHandle branchHandle);

		void ApplyCurve(const OperatorBranch& branch);

		void BuildVoxelGrid();

		static void CloneOperatingBranch(OperatorBranch& operatorBranch, const PredictedBranch& target);

		void SpaceColonization();

		void CalculateBranchRootDistance(const std::vector<std::pair<glm::vec3, BranchHandle>>& rootBranchHandles);

		void CalculateSkeletonGraphs();
	public:
		VoxelGrid<std::vector<PointData>> m_scatterPointsVoxelGrid;
		VoxelGrid<std::vector<PointData>> m_allocatedPointsVoxelGrid;
		VoxelGrid<std::vector<PointData>> m_spaceColonizationVoxelGrid;
		VoxelGrid<std::vector<BranchEndData>> m_branchEndsVoxelGrid;

		TreeMeshGeneratorSettings m_treeMeshGeneratorSettings {};
		ReconstructionSettings m_reconstructionSettings{};
		ConnectivityGraphSettings m_connectivityGraphSettings{};
		void ImportGraph(const std::filesystem::path& path, float scaleFactor = 0.1f);
		void ExportForestOBJ(const std::filesystem::path& path) const;

		glm::vec3 m_min;
		glm::vec3 m_max;
		std::vector<ScatteredPoint> m_scatteredPoints;
		std::vector<AllocatedPoint> m_allocatedPoints;
		std::vector<PredictedBranch> m_predictedBranches;

		std::vector<OperatorBranch> m_operatingBranches;
		std::vector<TreePart> m_treeParts;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		std::vector<ReconstructionSkeleton> m_skeletons;

		std::vector<std::pair<glm::vec3, glm::vec3>> m_scatterPointToBranchEndConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_scatterPointToBranchStartConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_scatterPointsConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_candidateBranchConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_reversedCandidateBranchConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_filteredBranchConnections;
		std::vector<std::pair<glm::vec3, glm::vec3>> m_branchConnections;
		void EstablishConnectivityGraph();

		void BuildSkeletons();

		void ClearMeshes() const;

		void OnCreate() override;

		void FormGeometryEntity() const;

		std::vector<std::shared_ptr<Mesh>> GenerateForestBranchMeshes() const;
		std::vector<std::shared_ptr<Mesh>> GenerateFoliageMeshes() const;
	};
}