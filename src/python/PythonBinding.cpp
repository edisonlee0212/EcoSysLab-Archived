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

using namespace EvoEngine;
using namespace EcoSysLab;

namespace py = pybind11;

void RegisterClasses() {
	ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
	ClassRegistry::RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
	ClassRegistry::RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
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
	
	treePointCloud->m_connectivityGraphSettings = connectivityGraphSettings;
	treePointCloud->m_reconstructionSettings = reconstructionSettings;
	treePointCloud->m_treeMeshGeneratorSettings = meshGeneratorSettings;
	treePointCloud->ImportGraph(yamlPath);
	treePointCloud->EstablishConnectivityGraph();
	treePointCloud->BuildSkeletons();
	treePointCloud->ExportForestOBJ(meshPath);
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
	treePointCloud->m_connectivityGraphSettings = connectivityGraphSettings;
	treePointCloud->m_reconstructionSettings = reconstructionSettings;
	treePointCloud->m_treeMeshGeneratorSettings = meshGeneratorSettings;
	treePointCloud->ImportGraph(yamlPath);
	treePointCloud->EstablishConnectivityGraph();
	treePointCloud->BuildSkeletons();
	treePointCloud->FormGeometryEntity();

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
	Application::Loop();
	for (int i = 0; i < iterations; i++)
	{
		ecoSysLabLayer->Simulate(deltaTime);
	}

	if (exportTreeMesh) {
		tree->ExportOBJ(treeMeshOutputPath, meshGeneratorSettings);
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

void RBVToObj(
	const std::string& rbvPath,
	const std::string& radialBoundingVolumeMeshOutputPath
)
{
	const auto rbv = ProjectManager::CreateTemporaryAsset<RadialBoundingVolume>();
	rbv->Import(rbvPath);
	rbv->ExportAsObj(radialBoundingVolumeMeshOutputPath);
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
	Application::Loop();
	for (int i = 0; i < iterations; i++)
	{
		ecoSysLabLayer->Simulate(deltaTime);
	}

	if (exportTreeMesh) {
		tree->ExportOBJ(treeMeshOutputPath, meshGeneratorSettings);
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
		.def_readwrite("m_pointCheckRadius", &ConnectivityGraphSettings::m_pointCheckRadius)
		.def_readwrite("m_pointPointConnectionDetectionRadius", &ConnectivityGraphSettings::m_pointPointConnectionDetectionRadius)
		.def_readwrite("m_pointBranchConnectionDetectionRange", &ConnectivityGraphSettings::m_pointBranchConnectionDetectionRange)
		.def_readwrite("m_branchBranchConnectionMaxLengthRange", &ConnectivityGraphSettings::m_branchBranchConnectionMaxLengthRange)
		.def_readwrite("m_directionConnectionAngleLimit", &ConnectivityGraphSettings::m_directionConnectionAngleLimit)
		.def_readwrite("m_indirectConnectionAngleLimit", &ConnectivityGraphSettings::m_indirectConnectionAngleLimit)
		;

	py::class_<PointCloudPointSettings>(m, "PointCloudPointSettings")
		.def(py::init<>())
		.def_readwrite("m_variance", &PointCloudPointSettings::m_variance)
		.def_readwrite("m_ballRandRadius", &PointCloudPointSettings::m_ballRandRadius)
		.def_readwrite("m_typeIndex", &PointCloudPointSettings::m_typeIndex)
		.def_readwrite("m_instanceIndex", &PointCloudPointSettings::m_instanceIndex)
		.def_readwrite("m_junctionIndex", &PointCloudPointSettings::m_junctionIndex)
		.def_readwrite("m_branchIndex", &PointCloudPointSettings::m_branchIndex)
		.def_readwrite("m_internodeIndex", &PointCloudPointSettings::m_internodeIndex)
		.def_readwrite("m_boundingBoxLimit", &PointCloudPointSettings::m_boundingBoxLimit);


	py::class_<PointCloudCaptureSettings>(m, "PointCloudCaptureSettings")
		.def(py::init<>())
		.def_readwrite("m_pitchAngleStart", &PointCloudCaptureSettings::m_pitchAngleStart)
		.def_readwrite("m_pitchAngleStep", &PointCloudCaptureSettings::m_pitchAngleStep)
		.def_readwrite("m_pitchAngleEnd", &PointCloudCaptureSettings::m_pitchAngleEnd)
		.def_readwrite("m_turnAngleStart", &PointCloudCaptureSettings::m_turnAngleStart)
		.def_readwrite("m_turnAngleStep", &PointCloudCaptureSettings::m_turnAngleStep)
		.def_readwrite("m_turnAngleEnd", &PointCloudCaptureSettings::m_turnAngleEnd)
		.def_readwrite("m_distance", &PointCloudCaptureSettings::m_distance)
		.def_readwrite("m_height", &PointCloudCaptureSettings::m_height)
		.def_readwrite("m_fov", &PointCloudCaptureSettings::m_fov)
		.def_readwrite("m_resolution", &PointCloudCaptureSettings::m_resolution)
		.def_readwrite("m_cameraDepthMax", &PointCloudCaptureSettings::m_cameraDepthMax);

	py::class_<ReconstructionSettings>(m, "ReconstructionSettings")
		.def(py::init<>())
		.def_readwrite("m_internodeLength", &ReconstructionSettings::m_internodeLength)
		.def_readwrite("m_minHeight", &ReconstructionSettings::m_minHeight)
		.def_readwrite("m_minimumTreeDistance", &ReconstructionSettings::m_minimumTreeDistance)
		.def_readwrite("m_branchShortening", &ReconstructionSettings::m_branchShortening)
		.def_readwrite("m_endNodeThickness", &ReconstructionSettings::m_endNodeThickness)
		.def_readwrite("m_minimumNodeCount", &ReconstructionSettings::m_minimumNodeCount);

	py::class_<PresentationOverrideSettings>(m, "PresentationOverrideSettings")
		.def(py::init<>())
		.def_readwrite("m_rootOverrideColor", &PresentationOverrideSettings::m_rootOverrideColor)
		.def_readwrite("m_branchOverrideColor", &PresentationOverrideSettings::m_branchOverrideColor)
		.def_readwrite("m_maxThickness", &PresentationOverrideSettings::m_maxThickness);

	py::class_<FoliageParameters>(m, "FoliageParameters")
		.def(py::init<>())
		.def_readwrite("m_leafSize", &FoliageParameters::m_leafSize)
		.def_readwrite("m_leafCountPerInternode", &FoliageParameters::m_leafCountPerInternode)
		.def_readwrite("m_positionVariance", &FoliageParameters::m_positionVariance)
		.def_readwrite("m_rotationVariance", &FoliageParameters::m_rotationVariance)
		.def_readwrite("m_branchingAngle", &FoliageParameters::m_branchingAngle)
		.def_readwrite("m_maxNodeThickness", &FoliageParameters::m_maxNodeThickness)
		.def_readwrite("m_minRootDistance", &FoliageParameters::m_minRootDistance)
		.def_readwrite("m_maxEndDistance", &FoliageParameters::m_maxEndDistance);

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
		.def_readwrite("m_xSubdivision", &TreeMeshGeneratorSettings::m_xSubdivision)
		.def_readwrite("m_trunkYSubdivision", &TreeMeshGeneratorSettings::m_trunkYSubdivision)
		.def_readwrite("m_trunkThickness", &TreeMeshGeneratorSettings::m_trunkThickness)
		.def_readwrite("m_branchYSubdivision", &TreeMeshGeneratorSettings::m_branchYSubdivision)


		.def_readwrite("m_overrideRadius", &TreeMeshGeneratorSettings::m_overrideRadius)
		.def_readwrite("m_radius", &TreeMeshGeneratorSettings::m_radius)
		.def_readwrite("m_overrideVertexColor", &TreeMeshGeneratorSettings::m_overrideVertexColor)
		.def_readwrite("m_junctionStartDistance", &TreeMeshGeneratorSettings::m_junctionStartDistance)
		.def_readwrite("m_junctionEndDistance", &TreeMeshGeneratorSettings::m_junctionEndDistance)
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
	m.def("rbv_to_obj", &RBVToObj, "RBVToObj");
	

	py::class_<DatasetGenerator>(m, "DatasetGenerator")
		.def(py::init<>())
		.def_static("GeneratePointCloudForTree", &DatasetGenerator::GeneratePointCloudForTree);

}