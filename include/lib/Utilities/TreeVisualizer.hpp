#pragma once

#include "TreeModel.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Jobs.hpp"
#include "StrandModel.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	
	struct SkeletalGraphSettings
	{
		float m_lineThickness = 0.0f;
		float m_fixedLineThickness = 0.001f;
		float m_branchPointSize = 1.0f;
		float m_junctionPointSize = 1.f;

		bool m_fixedPointSize = false;
		float m_fixedPointSizeFactor = 0.01f;
		glm::vec4 m_lineColor = glm::vec4(1.f, .5f, 0.5f, 1.0f);
		glm::vec4 m_branchPointColor = glm::vec4(1.f, 1.f, 0.f, 1.f);
		glm::vec4 m_junctionPointColor = glm::vec4(0.f, .7f, 1.f, 1.f);

		glm::vec4 m_lineFocusColor = glm::vec4(1.f, 0.f, 0.f, 1.f);
		glm::vec4 m_branchFocusColor = glm::vec4(1.f, 0.f, 0.f, 1.f);
		void OnInspect();
	};

	enum class ShootVisualizerMode {
		Default,
		Order,
		Level,
		LightIntensity,
		LightDirection,

		GrowthPotential,
		ApicalControl,
		DesiredGrowthRate,
		GrowthRate,

		IsMaxChild,
		AllocatedVigor,

	};

	enum class RootVisualizerMode {
		Default,
		AllocatedVigor,
	};

	struct TreeVisualizerColorSettings {
		int m_shootVisualizationMode = static_cast<int>(ShootVisualizerMode::DesiredGrowthRate);
		float m_shootColorMultiplier = 1.0f;
	};

	class TreeVisualizer {
		bool m_initialized = false;

		std::vector<glm::vec4> m_randomColors;

		std::shared_ptr<ParticleInfoList> m_internodeMatrices;

		
		
		bool DrawInternodeInspectionGui(TreeModel& treeModel, SkeletonNodeHandle internodeHandle, bool& deleted, const unsigned& hierarchyLevel);

		void PeekNodeInspectionGui(const ShootSkeleton& skeleton, SkeletonNodeHandle nodeHandle, const unsigned& hierarchyLevel);

		void PeekInternode(const ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle) const;

		bool InspectInternode(ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle);

	public:
		bool RayCastSelection(const std::shared_ptr<Camera>& cameraComponent, const glm::vec2& mousePosition, const ShootSkeleton& skeleton, const GlobalTransform& globalTransform);

		bool ScreenCurvePruning(const std::function<void(SkeletonNodeHandle)>& handler, std::vector<glm::vec2>& mousePositions, ShootSkeleton& skeleton, const GlobalTransform& globalTransform);

		std::vector<SkeletonNodeHandle> m_selectedInternodeHierarchyList;
		SkeletonNodeHandle m_selectedInternodeHandle = -1;
		bool m_visualization = true;
		TreeVisualizerColorSettings m_settings;
		SkeletalGraphSettings m_skeletalGraphSettings{};
		bool m_profileGui = true;
		bool m_treeHierarchyGui = false;
		float m_selectedInternodeLengthFactor = 0.0f;
		int m_checkpointIteration = 0;
		bool m_needUpdate = false;

		[[nodiscard]] bool Initialized() const;
		void ClearSelections();

		void Initialize();

		void SetSelectedNode(const ShootSkeleton& skeleton, SkeletonNodeHandle nodeHandle);

		void SyncMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particleInfoList, SkeletonNodeHandle selectedNodeHandle);

		bool OnInspect(TreeModel& treeModel);

		void Visualize(const TreeModel& treeModel, const GlobalTransform& globalTransform);
		void Visualize(StrandModel& strandModel);
		void Reset(TreeModel& treeModel);

		void Clear();
	};
}