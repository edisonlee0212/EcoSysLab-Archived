#include "DatasetGenerator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "WindowLayer.hpp"
#include "RenderLayer.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "PlayerController.hpp"
#include "Prefab.hpp"
#include "Times.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "ClassRegistry.hpp"
#include "TreeModel.hpp"
#include "Tree.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "RadialBoundingVolume.hpp"
#include "HeightField.hpp"
#include "ObjectRotator.hpp"
#include "SorghumLayer.hpp"
#include "TreePointCloud.hpp"
#include "Scene.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>
#endif
#include <TreePointCloudScanner.hpp>

#include "DatasetGenerator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"
using namespace EcoSysLab;

void RegisterClasses() {
	ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
	ClassRegistry::RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
	ClassRegistry::RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
}

void RegisterLayers(bool enableWindowLayer, bool enableEditorLayer)
{
	if (enableWindowLayer) Application::PushLayer<WindowLayer>();
	if (enableWindowLayer && enableEditorLayer) Application::PushLayer<EditorLayer>();
	Application::PushLayer<RenderLayer>();
	Application::PushLayer<EcoSysLabLayer>();
	Application::PushLayer<SorghumLayer>();
#ifdef BUILD_WITH_RAYTRACER
	Application::PushLayer<RayTracerLayer>();
#endif
}

void StartProjectWindowless(const std::filesystem::path& projectPath)
{
	if (std::filesystem::path(projectPath).extension().string() != ".eveproj") {
		EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
		return;
	}
	RegisterClasses();
	RegisterLayers(false, false);
	ApplicationInfo applicationInfo{};
	applicationInfo.m_projectPath = projectPath;
	Application::Initialize(applicationInfo);
	Application::Start();
}
int main() {
	std::filesystem::path project_folder_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject";
	std::filesystem::path project_path = project_folder_path / "test.eveproj";
	std::filesystem::path output_root = "D:\\ForestPointCloudData\\";

	StartProjectWindowless(project_path);

	TreeMeshGeneratorSettings tmgs{};

	tmgs.m_enableFoliage = true;
	tmgs.m_enableTwig = true;

	PointCloudPointSettings pcps{};
	PointCloudCircularCaptureSettings pcccs{};
	PointCloudGridCaptureSettings pcgcs{};
	pcps.m_ballRandRadius = 0.01f;
	pcps.m_treePartIndex = false;
	pcps.m_branchIndex = false;
	pcps.m_instanceIndex = true;
	pcps.m_typeIndex = true;
	pcps.m_lineIndex = false;
	pcccs.m_distance = 4.0f;
	pcccs.m_height = 3.0f;

	int gridSize = 6;
	float gridDistance = 1.0f;
	float randomShift = 0.5f;
	pcgcs.m_gridSize = { gridSize + 1, gridSize  + 1};
	pcgcs.m_gridDistance = gridDistance;

	int index = 0;
	for (int i = 0; i < 40; i++) {
		std::filesystem::path target_descriptor_folder_path = project_folder_path / "TreeDescriptors";
		std::filesystem::path target_forest_patch_path = project_folder_path / "Digital Forestry" / "4x4.fp";
		std::string name = "Forest_" + std::to_string(i);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		//DatasetGenerator::GeneratePointCloudForTree(pcps, pcccs, target_descriptor_folder_path.string(), 0.08220, 200, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), true, target_tree_junction_path.string());
		
		DatasetGenerator::GeneratePointCloudForForestPatch(gridSize, gridDistance, randomShift, pcps, pcgcs, target_descriptor_folder_path.string(), target_forest_patch_path.string(), 0.08220, 120, 30000, tmgs, target_tree_pointcloud_path.string());
		index++;
	}
	

}