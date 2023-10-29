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
#include "Times.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
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
#include "SorghumLayer.hpp"
#include "TreePointCloud.hpp"
#include "Scene.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>
#endif

using namespace EvoEngine;
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
	RegisterLayers(true, true);
	ApplicationInfo applicationInfo{};
	applicationInfo.m_projectPath = projectPath;
	Application::Initialize(applicationInfo);
	Application::Start();
}

void CaptureScene(
	const float posX, const float posY, const float posZ,
	const float angleX, const float angleY, const float angleZ,
	const int resolutionX, const int resolutionY, bool whiteBackground, const std::string& outputPath)
{
	if (resolutionX <= 0 || resolutionY <= 0)
	{
		EVOENGINE_ERROR("Resolution error!");
		return;
	}

	const auto scene = Application::GetActiveScene();
	if (!scene)
	{
		EVOENGINE_ERROR("No active scene!");
		return;
	}
	auto mainCamera = scene->m_mainCamera.Get<Camera>();
	Entity mainCameraEntity;
	bool tempCamera = false;
	if (!mainCamera)
	{
		mainCameraEntity = scene->CreateEntity("Main Camera");
		mainCamera = scene->GetOrSetPrivateComponent<Camera>(mainCameraEntity).lock();
		scene->m_mainCamera = mainCamera;
		tempCamera = true;
	}
	else
	{
		mainCameraEntity = mainCamera->GetOwner();
	}
	auto globalTransform = scene->GetDataComponent<GlobalTransform>(mainCameraEntity);
	const auto originalTransform = globalTransform;
	globalTransform.SetPosition({ posX, posY, posZ });
	globalTransform.SetEulerRotation(glm::radians(glm::vec3(angleX, angleY, angleZ)));
	scene->SetDataComponent(mainCameraEntity, globalTransform);
	mainCamera->Resize({ resolutionX, resolutionY });
	const auto useClearColor = mainCamera->m_useClearColor;
	const auto clearColor = mainCamera->m_clearColor;
	if (whiteBackground)
	{
		mainCamera->m_useClearColor = true;
		mainCamera->m_clearColor = glm::vec3(1, 1, 1);
	}
	Application::Loop();
	mainCamera->GetRenderTexture()->StoreToPng(outputPath);
	if (tempCamera)
	{
		scene->DeleteEntity(mainCameraEntity);
	}
	else
	{
		scene->SetDataComponent(mainCameraEntity, originalTransform);
		if (whiteBackground)
		{
			mainCamera->m_useClearColor = useClearColor;
			mainCamera->m_clearColor = clearColor;
		}
	}
	EVOENGINE_LOG("Exported image to " + outputPath);
}

Entity ImportTreePointCloud(const std::string& yamlPath)
{
	const auto scene = Application::GetActiveScene();
	const auto retVal = scene->CreateEntity("TreePointCloud");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreePointCloud>(retVal).lock();
	treePointCloud->ImportGraph(yamlPath);
	return retVal;
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
	treePointCloud->BuildSkeletons(reconstructionSettings);
	treePointCloud->ExportForestOBJ(meshPath, meshGeneratorSettings);
	EVOENGINE_LOG("Exported forest as OBJ");
	scene->DeleteEntity(tempEntity);
}
void VisualizeYaml(const std::string& yamlPath,
	const ConnectivityGraphSettings& connectivityGraphSettings,
	const ReconstructionSettings& reconstructionSettings,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const float posX, const float posY, const float posZ,
	const float angleX, const float angleY, const float angleZ,
	const int resolutionX, const int resolutionY, const std::string& outputPath)
{
	const auto scene = Application::GetActiveScene();
	const auto tempEntity = scene->CreateEntity("Temp");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreePointCloud>(tempEntity).lock();
	treePointCloud->ImportGraph(yamlPath);

	treePointCloud->EstablishConnectivityGraph(connectivityGraphSettings);
	treePointCloud->BuildSkeletons(reconstructionSettings);
	treePointCloud->FormGeometryEntity(meshGeneratorSettings);

	CaptureScene(posX, posY, posZ, angleX, angleY, angleZ, resolutionX, resolutionY, true, outputPath);
	scene->DeleteEntity(tempEntity);
}

void VoxelSpaceColonizationTreeData(
	const float radius,
	const std::string& binvoxPath,
	const std::string& treeParametersPath,
	const float deltaTime,
	const int iterations,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	bool exportTreeMesh,
	const std::string& treeMeshOutputPath,
	bool exportTreeIO,
	const std::string& treeIOOutputPath,
	bool exportRadialBoundingVolume,
	const std::string& radialBoundingVolumeOutputPath,
	bool exportRadialBoundingVolumeMesh,
	const std::string& radialBoundingVolumeMeshOutputPath
)
{
	const auto applicationStatus = Application::GetApplicationStatus();
	if (applicationStatus == ApplicationStatus::NoProject)
	{
		EVOENGINE_ERROR("No project!");
		return;
	}
	if (applicationStatus == ApplicationStatus::OnDestroy)
	{
		EVOENGINE_ERROR("Application is destroyed!");
		return;
	}
	if (applicationStatus == ApplicationStatus::Uninitialized)
	{
		EVOENGINE_ERROR("Application not uninitialized!");
		return;
	}
	const auto scene = Application::GetActiveScene();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	if (!ecoSysLabLayer)
	{
		EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
		return;
	}
	std::shared_ptr<Soil> soil;
	std::shared_ptr<Climate> climate;

	const std::vector<Entity>* soilEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Soil>();
	if (soilEntities && !soilEntities->empty()) {
		soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
	}
	if (!soil)
	{
		EVOENGINE_ERROR("No soil in scene!");
		return;
	}
	const std::vector<Entity>* climateEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Climate>();
	if (climateEntities && !climateEntities->empty()) {
		climate = scene->GetOrSetPrivateComponent<Climate>(climateEntities->at(0)).lock();
	}
	if (!climate)
	{
		EVOENGINE_ERROR("No climate in scene!");
		return;
	}

	const auto tempEntity = scene->CreateEntity("Temp");
	const auto tree = scene->GetOrSetPrivateComponent<Tree>(tempEntity).lock();
	tree->m_soil = soil;
	tree->m_climate = climate;
	std::shared_ptr<TreeDescriptor> treeDescriptor;
	if (ProjectManager::IsInProjectFolder(treeParametersPath))
	{
		treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
	}
	else {
		treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
	}
	tree->m_treeDescriptor = treeDescriptor;
	auto& occupancyGrid = tree->m_treeModel.m_treeOccupancyGrid;
	VoxelGrid<TreeOccupancyGridBasicData> inputGrid{};
	if (tree->ParseBinvox(binvoxPath, inputGrid, 1.f))
	{
		occupancyGrid.Initialize(inputGrid,
			glm::vec3(-radius, 0, -radius),
			glm::vec3(radius, 2.0f * radius, radius),
			treeDescriptor->m_shootGrowthParameters.m_internodeLength,
			tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor,
			tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationTheta,
			tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor);
	}
	tree->m_treeModel.m_treeGrowthSettings.m_enableShoot = true;
	tree->m_treeModel.m_treeGrowthSettings.m_enableRoot = false;
	tree->m_treeModel.m_treeGrowthSettings.m_useSpaceColonization = true;
	tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationAutoResize = false;
	for (int i = 0; i < iterations; i++)
	{
		tree->TryGrow(deltaTime);
	}

	if (exportTreeMesh) {
		tree->GenerateMeshes(meshGeneratorSettings);
		const auto children = scene->GetChildren(tempEntity);
		for (const auto& child : children) {
			auto name = scene->GetEntityName(child);
			if (name == "Branch Mesh") {
				auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
				mmr->m_mesh.Get<Mesh>()->Export(treeMeshOutputPath);
			}

		}
	}
	if (exportTreeIO)
	{
		bool succeed = tree->ExportIOTree(treeIOOutputPath);
	}
	if (exportRadialBoundingVolume || exportRadialBoundingVolumeMesh)
	{
		const auto rbv = ProjectManager::CreateTemporaryAsset<RadialBoundingVolume>();
		tree->ExportRadialBoundingVolume(rbv);
		if (exportRadialBoundingVolume)
		{
			rbv->Export(radialBoundingVolumeOutputPath);
		}
		if (exportRadialBoundingVolumeMesh)
		{
			rbv->ExportAsObj(radialBoundingVolumeMeshOutputPath);
		}
	}
	scene->DeleteEntity(tempEntity);
}


void RBVSpaceColonizationTreeData(
	const std::string& rbvPath,
	const std::string& treeParametersPath,
	const float deltaTime,
	const int iterations,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	bool exportTreeMesh,
	const std::string& treeMeshOutputPath,
	bool exportTreeIO,
	const std::string& treeIOOutputPath,
	bool exportRadialBoundingVolumeMesh,
	const std::string& radialBoundingVolumeMeshOutputPath
)
{
	const auto applicationStatus = Application::GetApplicationStatus();
	if (applicationStatus == ApplicationStatus::NoProject)
	{
		EVOENGINE_ERROR("No project!");
		return;
	}
	if (applicationStatus == ApplicationStatus::OnDestroy)
	{
		EVOENGINE_ERROR("Application is destroyed!");
		return;
	}
	if (applicationStatus == ApplicationStatus::Uninitialized)
	{
		EVOENGINE_ERROR("Application not uninitialized!");
		return;
	}
	const auto scene = Application::GetActiveScene();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	if (!ecoSysLabLayer)
	{
		EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
		return;
	}
	std::shared_ptr<Soil> soil;
	std::shared_ptr<Climate> climate;

	const std::vector<Entity>* soilEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Soil>();
	if (soilEntities && !soilEntities->empty()) {
		soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
	}
	if (!soil)
	{
		EVOENGINE_ERROR("No soil in scene!");
		return;
	}
	const std::vector<Entity>* climateEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Climate>();
	if (climateEntities && !climateEntities->empty()) {
		climate = scene->GetOrSetPrivateComponent<Climate>(climateEntities->at(0)).lock();
	}
	if (!climate)
	{
		EVOENGINE_ERROR("No climate in scene!");
		return;
	}

	const auto tempEntity = scene->CreateEntity("Temp");
	const auto tree = scene->GetOrSetPrivateComponent<Tree>(tempEntity).lock();
	tree->m_soil = soil;
	tree->m_climate = climate;
	std::shared_ptr<TreeDescriptor> treeDescriptor;
	if (ProjectManager::IsInProjectFolder(treeParametersPath))
	{
		treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
	}
	else {
		treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
	}
	tree->m_treeDescriptor = treeDescriptor;
	auto& occupancyGrid = tree->m_treeModel.m_treeOccupancyGrid;
	const auto rbv = ProjectManager::CreateTemporaryAsset<RadialBoundingVolume>();
	rbv->Import(rbvPath);

	occupancyGrid.Initialize(rbv,
		glm::vec3(-rbv->m_maxRadius, 0, -rbv->m_maxRadius),
		glm::vec3(rbv->m_maxRadius, 2.0f * rbv->m_maxRadius, rbv->m_maxRadius),
		treeDescriptor->m_shootGrowthParameters.m_internodeLength,
		tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor,
		tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationTheta,
		tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor);

	tree->m_treeModel.m_treeGrowthSettings.m_enableShoot = true;
	tree->m_treeModel.m_treeGrowthSettings.m_enableRoot = false;
	tree->m_treeModel.m_treeGrowthSettings.m_useSpaceColonization = true;
	tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationAutoResize = false;
	for (int i = 0; i < iterations; i++)
	{
		tree->TryGrow(deltaTime);
	}

	if (exportTreeMesh) {
		tree->GenerateMeshes(meshGeneratorSettings);
		const auto children = scene->GetChildren(tempEntity);
		for (const auto& child : children) {
			auto name = scene->GetEntityName(child);
			if (name == "Branch Mesh") {
				auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
				mmr->m_mesh.Get<Mesh>()->Export(treeMeshOutputPath);
			}

		}
	}
	if (exportTreeIO)
	{
		bool succeed = tree->ExportIOTree(treeIOOutputPath);
	}
	if (exportRadialBoundingVolumeMesh)
	{
		rbv->ExportAsObj(radialBoundingVolumeMeshOutputPath);
	}
	scene->DeleteEntity(tempEntity);
}

PYBIND11_MODULE(pyecosyslab, m) {
	py::class_<Entity>(m, "Entity")
		.def("GetIndex", &Entity::GetIndex)
		.def("GetVersion", &Entity::GetVersion);

	py::class_<ConnectivityGraphSettings>(m, "ConnectivityGraphSettings")
		.def(py::init<>())
		.def_readwrite("m_scatterPointsConnectionMaxLength", &ConnectivityGraphSettings::m_scatterPointsConnectionMaxLength)
		.def_readwrite("m_scatterPointBranchConnectionMaxLength", &ConnectivityGraphSettings::m_scatterPointBranchConnectionMaxLength)
		.def_readwrite("m_edgeExtendStep", &ConnectivityGraphSettings::m_edgeExtendStep)
		.def_readwrite("m_edgeLength", &ConnectivityGraphSettings::m_edgeLength)
		.def_readwrite("m_maxTimeout", &ConnectivityGraphSettings::m_maxTimeout)
		.def_readwrite("m_forceConnectionAngleLimit", &ConnectivityGraphSettings::m_forceConnectionAngleLimit)
		.def_readwrite("m_forceConnectionRatio", &ConnectivityGraphSettings::m_forceConnectionRatio)
		.def_readwrite("m_angleLimit", &ConnectivityGraphSettings::m_angleLimit)
		.def_readwrite("m_branchShortening", &ConnectivityGraphSettings::m_branchShortening);



	py::class_<ReconstructionSettings>(m, "ReconstructionSettings")
		.def(py::init<>())
		.def_readwrite("m_internodeLength", &ReconstructionSettings::m_internodeLength)
		.def_readwrite("m_minHeight", &ReconstructionSettings::m_minHeight)
		.def_readwrite("m_maxTreeDistance", &ReconstructionSettings::m_maxTreeDistance)
		.def_readwrite("m_branchShortening", &ReconstructionSettings::m_branchShortening)
		.def_readwrite("m_endNodeThickness", &ReconstructionSettings::m_endNodeThickness)
		.def_readwrite("m_overrideThickness", &ReconstructionSettings::m_overrideThickness)
		.def_readwrite("m_minimumNodeCount", &ReconstructionSettings::m_minimumNodeCount);

	py::class_<PresentationOverrideSettings>(m, "PresentationOverrideSettings")
		.def(py::init<>())
		.def_readwrite("m_rootOverrideColor", &PresentationOverrideSettings::m_rootOverrideColor)
		.def_readwrite("m_branchOverrideColor", &PresentationOverrideSettings::m_branchOverrideColor)
		.def_readwrite("m_foliageOverrideColor", &PresentationOverrideSettings::m_foliageOverrideColor)
		.def_readwrite("m_limitMaxThickness", &PresentationOverrideSettings::m_limitMaxThickness);

	py::class_<FoliageOverrideSettings>(m, "FoliageOverrideSettings")
		.def(py::init<>())
		.def_readwrite("m_leafSize", &FoliageOverrideSettings::m_leafSize)
		.def_readwrite("m_leafCountPerInternode", &FoliageOverrideSettings::m_leafCountPerInternode)
		.def_readwrite("m_positionVariance", &FoliageOverrideSettings::m_positionVariance)
		.def_readwrite("m_maxNodeThickness", &FoliageOverrideSettings::m_maxNodeThickness)
		.def_readwrite("m_minRootDistance", &FoliageOverrideSettings::m_minRootDistance)
		.def_readwrite("m_maxEndDistance", &FoliageOverrideSettings::m_maxEndDistance);

	py::class_<TreeMeshGeneratorSettings>(m, "TreeMeshGeneratorSettings")
		.def(py::init<>())
		.def_readwrite("m_vertexColorOnly", &TreeMeshGeneratorSettings::m_vertexColorOnly)
		.def_readwrite("m_enableFoliage", &TreeMeshGeneratorSettings::m_enableFoliage)
		.def_readwrite("m_enableFruit", &TreeMeshGeneratorSettings::m_enableFruit)
		.def_readwrite("m_enableBranch", &TreeMeshGeneratorSettings::m_enableBranch)
		.def_readwrite("m_enableRoot", &TreeMeshGeneratorSettings::m_enableRoot)
		.def_readwrite("m_enableFineRoot", &TreeMeshGeneratorSettings::m_enableFineRoot)
		.def_readwrite("m_enableTwig", &TreeMeshGeneratorSettings::m_enableTwig)
		.def_readwrite("m_foliageOverride", &TreeMeshGeneratorSettings::m_foliageOverride)
		.def_readwrite("m_foliageOverrideSettings", &TreeMeshGeneratorSettings::m_foliageOverrideSettings)
		.def_readwrite("m_presentationOverrideSettings", &TreeMeshGeneratorSettings::m_presentationOverrideSettings)
		.def_readwrite("m_ringXSubdivision", &TreeMeshGeneratorSettings::m_ringXSubdivision)
		.def_readwrite("m_ringYSubdivision", &TreeMeshGeneratorSettings::m_ringYSubdivision)
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
		.def_readwrite("m_rootMeshType", &TreeMeshGeneratorSettings::m_rootMeshType);

	py::class_<Scene>(m, "Scene")
		.def("CreateEntity", static_cast<Entity(Scene::*)(const std::string&)>(&Scene::CreateEntity))
		.def("DeleteEntity", static_cast<void(Scene::*)(const Entity&)>(&Scene::DeleteEntity));

	py::class_<ApplicationInfo>(m, "ApplicationInfo")
		.def(py::init<>())
		.def_readwrite("m_projectPath", &ApplicationInfo::m_projectPath)
		.def_readwrite("m_applicationName", &ApplicationInfo::m_applicationName)
		.def_readwrite("m_enableDocking", &ApplicationInfo::m_enableDocking)
		.def_readwrite("m_enableViewport", &ApplicationInfo::m_enableViewport)
		.def_readwrite("m_fullScreen", &ApplicationInfo::m_fullScreen);

	py::class_<Application>(m, "Application")
		.def_static("Initialize", &Application::Initialize)
		.def_static("Start", &Application::Start)
		.def_static("Run", &Application::Run)
		.def_static("Loop", &Application::Loop)
		.def_static("Terminate", &Application::Terminate)
		.def_static("GetActiveScene", &Application::GetActiveScene);

	py::class_<ProjectManager>(m, "ProjectManager")
		.def_static("GetOrCreateProject", &ProjectManager::GetOrCreateProject);

	m.doc() = "EcoSysLab"; // optional module docstring
	m.def("register_classes", &RegisterClasses, "RegisterClasses");
	m.def("register_layers", &RegisterLayers, "RegisterLayers");
	m.def("start_project_windowless", &StartProjectWindowless, "StartProjectWindowless");
	m.def("start_project_with_editor", &StartProjectWithEditor, "StartProjectWithEditor");

	m.def("yaml_to_mesh", &YamlToMesh, "YamlToMesh");
	m.def("capture_scene", &CaptureScene, "CaptureScene");
	m.def("visualize_yaml", &VisualizeYaml, "VisualizeYaml");
	m.def("voxel_space_colonization_tree_data", &VoxelSpaceColonizationTreeData, "VoxelSpaceColonizationTreeData");
	m.def("rbv_space_colonization_tree_data", &RBVSpaceColonizationTreeData, "RBVSpaceColonizationTreeData");
}