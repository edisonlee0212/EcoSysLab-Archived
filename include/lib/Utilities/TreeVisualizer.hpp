#pragma once

#include "TreeModel.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Jobs.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	

	enum class ShootVisualizerMode {
		Order,
		Level,
		LightIntensity,
		LightDirection,

		GrowthPotential,
		ApicalControl,
		DesiredGrowthRate,

		IsMaxChild,
		AllocatedVigor,

	};

	enum class RootVisualizerMode {
		Default,
		AllocatedVigor,
	};

	struct TreeVisualizerColorSettings {
		int m_shootVisualizationMode = static_cast<int>(ShootVisualizerMode::Level);
		float m_shootColorMultiplier = 1.0f;
	};

	class TreeVisualizer {
		bool m_initialized = false;

		std::vector<glm::vec4> m_randomColors;

		std::shared_ptr<ParticleInfoList> m_internodeMatrices;

		
		
		bool DrawInternodeInspectionGui(TreeModel& treeModel, NodeHandle internodeHandle, bool& deleted, const unsigned& hierarchyLevel);

		void PeekNodeInspectionGui(const ShootSkeleton& skeleton, NodeHandle nodeHandle, const unsigned& hierarchyLevel);

		void PeekInternode(const ShootSkeleton& shootSkeleton, NodeHandle internodeHandle) const;

		bool InspectInternode(ShootSkeleton& shootSkeleton, NodeHandle internodeHandle);

	public:
		bool RayCastSelection(const std::shared_ptr<Camera>& cameraComponent, const glm::vec2& mousePosition, const ShootSkeleton& skeleton, const GlobalTransform& globalTransform);

		bool ScreenCurvePruning(const std::function<void(NodeHandle)>& handler, std::vector<glm::vec2>& mousePositions, ShootSkeleton& skeleton, const GlobalTransform& globalTransform);

		std::vector<NodeHandle> m_selectedInternodeHierarchyList;
		NodeHandle m_selectedInternodeHandle = -1;
		bool m_visualization = true;
		TreeVisualizerColorSettings m_settings;
		
		bool m_frontProfileGui = false;
		bool m_backProfileGui = true;
		bool m_treeHierarchyGui = false;
		float m_selectedInternodeLengthFactor = 0.0f;
		int m_checkpointIteration = 0;
		bool m_needUpdate = false;
		bool m_needShootColorUpdate = false;

		[[nodiscard]] bool Initialized() const;
		void ClearSelections();

		void Initialize();

		void SetSelectedNode(const ShootSkeleton& skeleton, NodeHandle nodeHandle);

		void SyncMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particleInfoList);

		void SyncColors(const ShootSkeleton& shootSkeleton, NodeHandle selectedNodeHandle);


		bool OnInspect(TreeModel& treeModel);

		void Visualize(TreeModel& treeModel, const GlobalTransform& globalTransform);

		void Reset(TreeModel& treeModel);

		void Clear();
	};
}