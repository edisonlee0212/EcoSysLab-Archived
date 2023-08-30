#include "pybind11/pybind11.h"
#include "pybind11/stl/filesystem.h"
#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "WindowLayer.hpp"
#include "RenderLayer.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "PlayerController.hpp"
#include "Prefab.hpp"
#include "Time.hpp"
#include "PhysicsLayer.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "PhysicsLayer.hpp"
#include "ClassRegistry.hpp"
#include "TreeModel.hpp"
#include "Tree.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Trees.hpp"
#include "EcoSysLabLayer.hpp"
#include "RadialBoundingVolume.hpp"
#include "HeightField.hpp"
#include "ObjectRotator.hpp"
#include "SinglePipeProfile.hpp"
#include "SorghumLayer.hpp"
#include "TreePointCloud.hpp"
#include "PipeModelBase.hpp"
#include "Scene.hpp"
#ifdef RAYTRACERFACILITY

#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>

#endif

using namespace EvoEngine;

#ifdef RAYTRACERFACILITY
using namespace RayTracerFacility;
#endif
using namespace EcoSysLab;

namespace py = pybind11;

void RegisterClasses() {
	ClassRegistry::RegisterPrivateComponent<Tree>("Tree");
	ClassRegistry::RegisterPrivateComponent<TreePointCloud>("TreePointCloud");
	ClassRegistry::RegisterPrivateComponent<Soil>("Soil");
	ClassRegistry::RegisterPrivateComponent<Climate>("Climate");
	ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
	ClassRegistry::RegisterAsset<Trees>("Trees", { ".trees" });
	ClassRegistry::RegisterAsset<TreeDescriptor>("TreeDescriptor", { ".td" });
	ClassRegistry::RegisterAsset<SoilDescriptor>("SoilDescriptor", { ".sd" });
	ClassRegistry::RegisterAsset<ClimateDescriptor>("ClimateDescriptor", { ".cd" });
	ClassRegistry::RegisterAsset<RadialBoundingVolume>("RadialBoundingVolume", { ".rbv" });
	ClassRegistry::RegisterAsset<HeightField>("HeightField", { ".hf" });
	ClassRegistry::RegisterAsset<NoiseSoilLayerDescriptor>("NoiseSoilLayerDescriptor", { ".nsld" });
	ClassRegistry::RegisterPrivateComponent<SinglePipeProfile>("SinglePipeProfile");
	ClassRegistry::RegisterPrivateComponent<PipeModelBase>("PipeModelBase");
}

void PushWindowLayer() {
	Application::PushLayer<WindowLayer>();
}
void PushEditorLayer() {
	Application::PushLayer<EditorLayer>();
}
void PushRenderLayer() {
	Application::PushLayer<RenderLayer>();
}
void PushEcoSysLabLayer() {
	Application::PushLayer<EcoSysLabLayer>();
}
void PushSorghumLayer() {
	Application::PushLayer<SorghumLayer>();
}

void RegisterLayers(bool enableWindowLayer, bool enableEditorLayer)
{
	if (enableWindowLayer) Application::PushLayer<WindowLayer>();
	if (enableWindowLayer && enableEditorLayer) Application::PushLayer<EditorLayer>();
	Application::PushLayer<RenderLayer>();
	Application::PushLayer<EcoSysLabLayer>();
	Application::PushLayer<SorghumLayer>();
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

void StartProjectWithEditor(const std::filesystem::path& projectPath)
{
	if (!projectPath.empty()) {
		if (std::filesystem::path(projectPath).extension().string() != ".eveproj") {
			EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
			return;
		}
	}
	RegisterClasses();
	RegisterLayers(false, false);
	ApplicationInfo applicationInfo{};
	applicationInfo.m_projectPath = projectPath;
	Application::Initialize(applicationInfo);
	Application::Start();
	Application::Run();
}

void CaptureActiveScene(const int resolutionX, const int resolutionY, const std::string& outputPath)
{
	if (resolutionX <= 0 || resolutionY <= 0)
	{
		EVOENGINE_ERROR("Resolution error!");
		return;
	}
	
	const auto scene = Application::GetActiveScene();
	if(!scene)
	{
		EVOENGINE_ERROR("No active scene!");
		return;
	}
	const auto mainCamera = scene->m_mainCamera.Get<Camera>();
	if(!mainCamera)
	{
		EVOENGINE_ERROR("No main camera in scene!");
		return;
	}
	mainCamera->Resize({ resolutionX, resolutionY });
	Application::Loop();
	mainCamera->GetRenderTexture()->StoreToPng(outputPath);
	EVOENGINE_LOG("Exported image to " + outputPath);
}

void YamlToMesh(const std::string& yamlPath,
	const ConnectivityGraphSettings& connectivityGraphSettings,
	const ReconstructionSettings& reconstructionSettings,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const std::string& meshPath) {
	const auto scene = Application::GetActiveScene();
	const auto tempEntity = scene->CreateEntity("Temp");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreePointCloud>(tempEntity).lock();
	treePointCloud->ImportGraph(yamlPath);

	treePointCloud->EstablishConnectivityGraph(connectivityGraphSettings);
	treePointCloud->BuildTreeStructure(reconstructionSettings);
	const auto meshes = treePointCloud->GenerateMeshes(meshGeneratorSettings);
	EVOENGINE_LOG("Exporting " + std::to_string(meshes.size()) + " meshes");
	if (meshes.size() == 1) {
		meshes[0]->Export(meshPath);
	}
	else {
		int index = 0;
		for (const auto& i : meshes) {
			auto savePath = std::filesystem::path(meshPath);
			savePath.replace_filename(savePath.filename().string() + "_" + std::to_string(index));
			index++;
			i->Export(savePath);
		}
	}
	EVOENGINE_LOG("Export finished!");
	scene->DeleteEntity(tempEntity);
}

PYBIND11_MODULE(pyecosyslab, m) {
	py::class_<Entity>(m, "Entity")
		.def("get_index", &Entity::GetIndex)
		.def("get_version", &Entity::GetVersion);

	py::class_<ConnectivityGraphSettings>(m, "ConnectivityGraphSettings")
		.def(py::init<>())
		.def_readwrite("m_scatterPointsConnectionMaxLength", &ConnectivityGraphSettings::m_scatterPointsConnectionMaxLength)
		.def_readwrite("m_scatterPointBranchConnectionMaxLength", &ConnectivityGraphSettings::m_scatterPointBranchConnectionMaxLength)
		.def_readwrite("m_edgeExtendStep", &ConnectivityGraphSettings::m_edgeExtendStep)
		.def_readwrite("m_edgeLength", &ConnectivityGraphSettings::m_edgeLength)
		.def_readwrite("m_maxTimeout", &ConnectivityGraphSettings::m_maxTimeout)
		.def_readwrite("m_forceConnectionAngleLimit", &ConnectivityGraphSettings::m_forceConnectionAngleLimit)
		.def_readwrite("m_forceConnectionRatio", &ConnectivityGraphSettings::m_forceConnectionRatio)
		.def_readwrite("m_absoluteAngleLimit", &ConnectivityGraphSettings::m_absoluteAngleLimit)
		.def_readwrite("m_branchShortening", &ConnectivityGraphSettings::m_branchShortening);



	py::class_<ReconstructionSettings>(m, "ReconstructionSettings")
		.def(py::init<>())
		.def_readwrite("m_internodeLength", &ReconstructionSettings::m_internodeLength)
		.def_readwrite("m_minHeight", &ReconstructionSettings::m_minHeight)
		.def_readwrite("m_maxTreeDistance", &ReconstructionSettings::m_maxTreeDistance)
		.def_readwrite("m_branchShortening", &ReconstructionSettings::m_branchShortening)
		.def_readwrite("m_minThickness", &ReconstructionSettings::m_minThickness)
		.def_readwrite("m_overrideThickness", &ReconstructionSettings::m_overrideThickness)
		.def_readwrite("m_minimumNodeCount", &ReconstructionSettings::m_minimumNodeCount);

	py::class_<PresentationOverrideSettings>(m, "PresentationOverrideSettings")
		.def(py::init<>())
		.def_readwrite("m_leafCountPerInternode", &PresentationOverrideSettings::m_leafCountPerInternode)
		.def_readwrite("m_distanceToEndLimit", &PresentationOverrideSettings::m_distanceToEndLimit)
		.def_readwrite("m_positionVariance", &PresentationOverrideSettings::m_positionVariance)
		.def_readwrite("m_phototropism", &PresentationOverrideSettings::m_phototropism)
		.def_readwrite("m_limitMaxThickness", &PresentationOverrideSettings::m_limitMaxThickness);

	py::class_<TreeMeshGeneratorSettings>(m, "TreeMeshGeneratorSettings")
		.def(py::init<>())
		.def_readwrite("m_vertexColorOnly", &TreeMeshGeneratorSettings::m_vertexColorOnly)
		.def_readwrite("m_enableFoliage", &TreeMeshGeneratorSettings::m_enableFoliage)
		.def_readwrite("m_enableFruit", &TreeMeshGeneratorSettings::m_enableFruit)
		.def_readwrite("m_enableBranch", &TreeMeshGeneratorSettings::m_enableBranch)
		.def_readwrite("m_enableRoot", &TreeMeshGeneratorSettings::m_enableRoot)
		.def_readwrite("m_enableFineRoot", &TreeMeshGeneratorSettings::m_enableFineRoot)
		.def_readwrite("m_enableTwig", &TreeMeshGeneratorSettings::m_enableTwig)
		.def_readwrite("m_overridePresentation", &TreeMeshGeneratorSettings::m_overridePresentation)
		.def_readwrite("m_presentationOverrideSettings", &TreeMeshGeneratorSettings::m_presentationOverrideSettings)
		.def_readwrite("m_resolution", &TreeMeshGeneratorSettings::m_resolution)
		.def_readwrite("m_subdivision", &TreeMeshGeneratorSettings::m_subdivision)
		.def_readwrite("m_overrideRadius", &TreeMeshGeneratorSettings::m_overrideRadius)
		.def_readwrite("m_radius", &TreeMeshGeneratorSettings::m_radius)
		.def_readwrite("m_overrideVertexColor", &TreeMeshGeneratorSettings::m_overrideVertexColor)
		.def_readwrite("m_markJunctions", &TreeMeshGeneratorSettings::m_markJunctions)
		.def_readwrite("m_junctionLowerRatio", &TreeMeshGeneratorSettings::m_junctionLowerRatio)
		.def_readwrite("m_junctionUpperRatio", &TreeMeshGeneratorSettings::m_junctionUpperRatio)
		.def_readwrite("m_baseControlPointRatio", &TreeMeshGeneratorSettings::m_baseControlPointRatio)
		.def_readwrite("m_branchControlPointRatio", &TreeMeshGeneratorSettings::m_branchControlPointRatio)
		.def_readwrite("m_lineLengthFactor", &TreeMeshGeneratorSettings::m_lineLengthFactor)
		.def_readwrite("m_smoothness", &TreeMeshGeneratorSettings::m_smoothness)
		.def_readwrite("m_autoLevel", &TreeMeshGeneratorSettings::m_autoLevel)
		.def_readwrite("m_voxelSubdivisionLevel", &TreeMeshGeneratorSettings::m_voxelSubdivisionLevel)
		.def_readwrite("m_voxelSmoothIteration", &TreeMeshGeneratorSettings::m_voxelSmoothIteration)
		.def_readwrite("m_removeDuplicate", &TreeMeshGeneratorSettings::m_removeDuplicate)
		.def_readwrite("m_branchMeshType", &TreeMeshGeneratorSettings::m_branchMeshType)
		.def_readwrite("m_rootMeshType", &TreeMeshGeneratorSettings::m_rootMeshType)
		.def_readwrite("m_detailedFoliage", &TreeMeshGeneratorSettings::m_detailedFoliage);

	py::class_<Scene>(m, "Scene")
		.def("create_entity", static_cast<Entity(Scene::*)(const std::string&)>(&Scene::CreateEntity))
		.def("delete_entity", &Scene::DeleteEntity);

	py::class_<ApplicationInfo>(m, "ApplicationInfo")
		.def(py::init<>())
		.def_readwrite("m_projectPath", &ApplicationInfo::m_projectPath)
		.def_readwrite("m_applicationName", &ApplicationInfo::m_applicationName)
		.def_readwrite("m_enableDocking", &ApplicationInfo::m_enableDocking)
		.def_readwrite("m_enableViewport", &ApplicationInfo::m_enableViewport)
		.def_readwrite("m_fullScreen", &ApplicationInfo::m_fullScreen);

	py::class_<Application>(m, "Application")
		.def_static("initialize", &Application::Initialize)
		.def_static("start", &Application::Start)
		.def_static("run", &Application::Run)
		.def_static("loop", &Application::Loop)
		.def_static("terminate", &Application::Terminate)
		.def_static("get_active_scene", &Application::GetActiveScene);

	py::class_<ProjectManager>(m, "ProjectManager")
		.def("GetOrCreateProject", &ProjectManager::GetOrCreateProject);

	m.doc() = "EcoSysLab"; // optional module docstring
	m.def("register_classes", &RegisterClasses, "RegisterClasses");
	m.def("register_layers", &RegisterLayers, "RegisterLayers");
	m.def("start_project_windowless", &StartProjectWindowless, "StartProjectWindowless");
	m.def("start_project_with_editor", &StartProjectWithEditor, "StartProjectWithEditor");

	m.def("yaml_to_mesh", &YamlToMesh, "YamlToMesh");
	m.def("capture_active_scene", &CaptureActiveScene, "CaptureActiveScene");
}