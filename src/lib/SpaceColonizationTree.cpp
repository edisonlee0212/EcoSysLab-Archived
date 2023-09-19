#include "SpaceColonizationTree.hpp"
using namespace EcoSysLab;
void SpaceColonizationTree::ParseVoxelData(const std::filesystem::path& filePath, VoxelGrid<SpaceColonizationTreeVoxelData>& voxelGrid)
{
	/*
	 * TODO: Initialize the grid with all voxels empty.
	 * One way to init (the maxBound will be automatically calculated):
	 * voxelGrid.Initialize(voxelSize, resolution, minBound, {});
	 * Another way to init (the resolution will be automatically calculated):
	 * voxelGrid.Initialize(voxelSize, minBound, maxBound, {});
	 */
	/*
	 * TODO: Parse the voxel data from the file path, and fill the voxelGrid above.
	 * E.g. You can fill a voxel with coordinate (0, 0, 0) by:
	 * voxelGrid.Ref(glm::ivec3(0, 0, 0)).m_occupied = true;
	 */

	// TODO: Please clear any unused resources before you return!
}

void SpaceColonizationTree::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	FileUtils::OpenFile("Load Voxel Data", "Binvox", { ".binvox" }, [&](const std::filesystem::path& path) {
		ParseVoxelData(path, m_voxelGrid);
		}, false);

}
