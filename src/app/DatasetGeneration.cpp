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
	std::filesystem::path project_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\test.eveproj";
	std::filesystem::path output_root = "D:\\TreePointCloudData\\";

	StartProjectWindowless(project_path);

	TreeMeshGeneratorSettings tmgs{};
	PointCloudPointSettings pcps{};
	PointCloudCaptureSettings pccs{};
	pcps.m_ballRandRadius = 0.01f;
	pcps.m_junctionIndex = false;
		
	pccs.m_distance = 4.0f;
	pccs.m_height = 3.0f;
	int index = 0;
	int numPerSpecie = 50;
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Elm";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Maple";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Acacia";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Apple";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Hazel";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Spruce";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Willow";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}
	for (int i = 0; i < numPerSpecie; i++) {
		std::string specieName = "Ash";
		std::filesystem::path target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\" + specieName + ".td";
		std::string name = specieName + "_" + std::to_string(index);
		std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
		std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
		std::filesystem::path target_tree_junction_path = output_root / (name + ".yml");
		DatasetGenerator::GeneratePointCloudForTree(pcps, pccs, target_descriptor_path.string(), 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path.string(), false, target_tree_mesh_path.string(), false, target_tree_junction_path.string());
		index++;
	}

}