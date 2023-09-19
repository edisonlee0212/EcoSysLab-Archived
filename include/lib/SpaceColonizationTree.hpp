#pragma once

#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "SpaceColonizationTreeModel.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	struct SpaceColonizationTreeVoxelData {
		bool m_occupied = false;
	};
	class SpaceColonizationTree : public IPrivateComponent {
	public:
		SpaceColonizationTreeModel m_spaceColonizationTreeModel{};
		VoxelGrid<SpaceColonizationTreeVoxelData> m_voxelGrid{};
		void ParseVoxelData(const std::filesystem::path& filePath, VoxelGrid<SpaceColonizationTreeVoxelData>& voxelGrid);
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}