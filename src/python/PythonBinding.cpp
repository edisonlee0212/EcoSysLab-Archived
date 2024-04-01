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
#include "TreeStructor.hpp"
#include "Scene.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>
#endif
#include <TreePointCloudScanner.hpp>

#include "DatasetGenerator.hpp"
#include "FoliageDescriptor.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"

using namespace EvoEngine;
using namespace EcoSysLab;

namespace py = pybind11;

void register_classes() {
	ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
	ClassRegistry::RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
	ClassRegistry::RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
}

void push_window_layer() {
	Application::PushLayer<WindowLayer>();
}
void push_editor_layer() {
	Application::PushLayer<EditorLayer>();
}
void push_render_layer() {
	Application::PushLayer<RenderLayer>();
}
void push_ecosyslab_layer() {
	Application::PushLayer<EcoSysLabLayer>();
}
void push_sorghum_layer() {
	Application::PushLayer<SorghumLayer>();
}

void register_layers(bool enableWindowLayer, bool enableEditorLayer)
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

void start_project_windowless(const std::filesystem::path& projectPath)
{
	if (std::filesystem::path(projectPath).extension().string() != ".eveproj") {
		EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
		return;
	}
	register_classes();
	register_layers(false, false);
	ApplicationInfo applicationInfo{};
	applicationInfo.m_projectPath = projectPath;
	Application::Initialize(applicationInfo);
	Application::Start();
}

void start_project(const std::filesystem::path& projectPath)
{
	if (!projectPath.empty()) {
		if (std::filesystem::path(projectPath).extension().string() != ".eveproj") {
			EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
			return;
		}
	}
	register_classes();
	register_layers(true, true);
	ApplicationInfo applicationInfo{};
	applicationInfo.m_projectPath = projectPath;
	Application::Initialize(applicationInfo);
	Application::Start();
}

void scene_capture(
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

Entity import_tree_pointcloud(const std::string& yamlPath)
{
	const auto scene = Application::GetActiveScene();
	const auto retVal = scene->CreateEntity("TreeStructor");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreeStructor>(retVal).lock();
	treePointCloud->ImportGraph(yamlPath);
	return retVal;
}

void tree_structor(const std::string& yamlPath,
	const ConnectivityGraphSettings& connectivityGraphSettings,
	const ReconstructionSettings& reconstructionSettings,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const std::string& meshPath) {
	const auto scene = Application::GetActiveScene();
	const auto tempEntity = scene->CreateEntity("Temp");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreeStructor>(tempEntity).lock();
	
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
void yaml_visualization(const std::string& yamlPath,
	const ConnectivityGraphSettings& connectivityGraphSettings,
	const ReconstructionSettings& reconstructionSettings,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const float posX, const float posY, const float posZ,
	const float angleX, const float angleY, const float angleZ,
	const int resolutionX, const int resolutionY, const std::string& outputPath)
{
	const auto scene = Application::GetActiveScene();
	const auto tempEntity = scene->CreateEntity("Temp");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreeStructor>(tempEntity).lock();
	treePointCloud->m_connectivityGraphSettings = connectivityGraphSettings;
	treePointCloud->m_reconstructionSettings = reconstructionSettings;
	treePointCloud->m_treeMeshGeneratorSettings = meshGeneratorSettings;
	treePointCloud->ImportGraph(yamlPath);
	treePointCloud->EstablishConnectivityGraph();
	treePointCloud->BuildSkeletons();
	treePointCloud->FormGeometryEntity();
	scene_capture(posX, posY, posZ, angleX, angleY, angleZ, resolutionX, resolutionY, true, outputPath);
	scene->DeleteEntity(tempEntity);
}

void voxel_space_colonization_tree_data(
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
			treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>()->m_internodeLength,
			tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor,
			tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationTheta,
			tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor);
	}
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

void rbv_to_obj(
	const std::string& rbvPath,
	const std::string& radialBoundingVolumeMeshOutputPath
)
{
	const auto rbv = ProjectManager::CreateTemporaryAsset<RadialBoundingVolume>();
	rbv->Import(rbvPath);
	rbv->ExportAsObj(radialBoundingVolumeMeshOutputPath);
}

void rbv_space_colonization_tree_data(
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
		treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>()->m_internodeLength,
		tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor,
		tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationTheta,
		tree->m_treeModel.m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor);

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
		.def_readwrite("m_pointBranchConnectionDetectionRadius", &ConnectivityGraphSettings::m_pointBranchConnectionDetectionRadius)
		.def_readwrite("m_branchBranchConnectionMaxLengthRange", &ConnectivityGraphSettings::m_branchBranchConnectionMaxLengthRange)
		.def_readwrite("m_directionConnectionAngleLimit", &ConnectivityGraphSettings::m_directionConnectionAngleLimit)
		.def_readwrite("m_indirectConnectionAngleLimit", &ConnectivityGraphSettings::m_indirectConnectionAngleLimit)
		;

	py::class_<TreePointCloudPointSettings>(m, "TreePointCloudPointSettings")
		.def(py::init<>())
		.def_readwrite("m_variance", &TreePointCloudPointSettings::m_variance)
		.def_readwrite("m_ballRandRadius", &TreePointCloudPointSettings::m_ballRandRadius)
		.def_readwrite("m_typeIndex", &TreePointCloudPointSettings::m_typeIndex)
		.def_readwrite("m_instanceIndex", &TreePointCloudPointSettings::m_instanceIndex)
		.def_readwrite("m_treePartIndex", &TreePointCloudPointSettings::m_treePartIndex)
		.def_readwrite("m_lineIndex", &TreePointCloudPointSettings::m_lineIndex)
		.def_readwrite("m_branchIndex", &TreePointCloudPointSettings::m_branchIndex)
		.def_readwrite("m_internodeIndex", &TreePointCloudPointSettings::m_internodeIndex)
		.def_readwrite("m_boundingBoxLimit", &TreePointCloudPointSettings::m_boundingBoxLimit);


	py::class_<TreePointCloudCircularCaptureSettings>(m, "PointCloudCircularCaptureSettings")
		.def(py::init<>())
		.def_readwrite("m_pitchAngleStart", &TreePointCloudCircularCaptureSettings::m_pitchAngleStart)
		.def_readwrite("m_pitchAngleStep", &TreePointCloudCircularCaptureSettings::m_pitchAngleStep)
		.def_readwrite("m_pitchAngleEnd", &TreePointCloudCircularCaptureSettings::m_pitchAngleEnd)
		.def_readwrite("m_turnAngleStart", &TreePointCloudCircularCaptureSettings::m_turnAngleStart)
		.def_readwrite("m_turnAngleStep", &TreePointCloudCircularCaptureSettings::m_turnAngleStep)
		.def_readwrite("m_turnAngleEnd", &TreePointCloudCircularCaptureSettings::m_turnAngleEnd)
		.def_readwrite("m_gridDistance", &TreePointCloudCircularCaptureSettings::m_distance)
		.def_readwrite("m_height", &TreePointCloudCircularCaptureSettings::m_height)
		.def_readwrite("m_fov", &TreePointCloudCircularCaptureSettings::m_fov)
		.def_readwrite("m_resolution", &TreePointCloudCircularCaptureSettings::m_resolution)
		.def_readwrite("m_cameraDepthMax", &TreePointCloudCircularCaptureSettingsWWWWWWWWWWW::m_cameraDepthMax);

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
		.def_readwrite("m_maxThickness", &PresentationOverrideSettings::m_maxThickness);

	py::class_<ShootDescriptor>(m, "ShootDescriptor")
		.def(py::init<>())
		.def_readwrite("m_growthRate", &ShootDescriptor::m_growthRate)
		.def_readwrite("m_branchingAngleMeanVariance", &ShootDescriptor::m_branchingAngleMeanVariance)
		.def_readwrite("m_rollAngleMeanVariance", &ShootDescriptor::m_rollAngleMeanVariance)
		.def_readwrite("m_apicalAngleMeanVariance", &ShootDescriptor::m_apicalAngleMeanVariance)
		.def_readwrite("m_gravitropism", &ShootDescriptor::m_gravitropism)
		.def_readwrite("m_phototropism", &ShootDescriptor::m_phototropism)
		.def_readwrite("m_horizontalTropism", &ShootDescriptor::m_horizontalTropism)
		.def_readwrite("m_internodeLength", &ShootDescriptor::m_internodeLength)
		.def_readwrite("m_internodeLengthThicknessFactor", &ShootDescriptor::m_internodeLengthThicknessFactor)
		.def_readwrite("m_endNodeThickness", &ShootDescriptor::m_endNodeThickness)
		.def_readwrite("m_thicknessAccumulationFactor", &ShootDescriptor::m_thicknessAccumulationFactor)
		.def_readwrite("m_thicknessAgeFactor", &ShootDescriptor::m_thicknessAgeFactor)
		.def_readwrite("m_internodeShadowFactor", &ShootDescriptor::m_internodeShadowFactor)
		.def_readwrite("m_lateralBudCount", &ShootDescriptor::m_lateralBudCount)
		.def_readwrite("m_apicalBudExtinctionRate", &ShootDescriptor::m_apicalBudExtinctionRate)
		.def_readwrite("m_lateralBudFlushingRate", &ShootDescriptor::m_lateralBudFlushingRate)
		.def_readwrite("m_apicalControl", &ShootDescriptor::m_apicalControl)
		.def_readwrite("m_apicalDominance", &ShootDescriptor::m_apicalDominance)
		.def_readwrite("m_apicalDominanceLoss", &ShootDescriptor::m_apicalDominanceLoss)
		.def_readwrite("m_lowBranchPruning", &ShootDescriptor::m_lowBranchPruning)
		.def_readwrite("m_lowBranchPruningThicknessFactor", &ShootDescriptor::m_lowBranchPruningThicknessFactor)
		.def_readwrite("m_lightPruningFactor", &ShootDescriptor::m_lightPruningFactor)
		.def_readwrite("m_thicknessPruningFactor", &ShootDescriptor::m_thicknessPruningFactor)
		.def_readwrite("m_saggingFactorThicknessReductionMax", &ShootDescriptor::m_saggingFactorThicknessReductionMax);


	py::class_<FoliageDescriptor>(m, "FoliageDescriptor")
		.def(py::init<>())
		.def_readwrite("m_leafSize", &FoliageDescriptor::m_leafSize)
		.def_readwrite("m_leafCountPerInternode", &FoliageDescriptor::m_leafCountPerInternode)
		.def_readwrite("m_positionVariance", &FoliageDescriptor::m_positionVariance)
		.def_readwrite("m_rotationVariance", &FoliageDescriptor::m_rotationVariance)
		.def_readwrite("m_branchingAngle", &FoliageDescriptor::m_branchingAngle)
		.def_readwrite("m_maxNodeThickness", &FoliageDescriptor::m_maxNodeThickness)
		.def_readwrite("m_minRootDistance", &FoliageDescriptor::m_minRootDistance)
		.def_readwrite("m_maxEndDistance", &FoliageDescriptor::m_maxEndDistance);

	py::class_<TreeMeshGeneratorSettings>(m, "TreeMeshGeneratorSettings")
		.def(py::init<>())
		.def_readwrite("m_vertexColorOnly", &TreeMeshGeneratorSettings::m_vertexColorOnly)
		.def_readwrite("m_enableFoliage", &TreeMeshGeneratorSettings::m_enableFoliage)
		.def_readwrite("m_enableFruit", &TreeMeshGeneratorSettings::m_enableFruit)
		.def_readwrite("m_enableBranch", &TreeMeshGeneratorSettings::m_enableBranch)
		.def_readwrite("m_enableTwig", &TreeMeshGeneratorSettings::m_enableTwig)
		.def_readwrite("m_presentationOverrideSettings", &TreeMeshGeneratorSettings::m_presentationOverrideSettings)
		.def_readwrite("m_xSubdivision", &TreeMeshGeneratorSettings::m_xSubdivision)
		.def_readwrite("m_trunkYSubdivision", &TreeMeshGeneratorSettings::m_trunkYSubdivision)
		.def_readwrite("m_trunkThickness", &TreeMeshGeneratorSettings::m_trunkThickness)
		.def_readwrite("m_branchYSubdivision", &TreeMeshGeneratorSettings::m_branchYSubdivision)


		.def_readwrite("m_overrideRadius", &TreeMeshGeneratorSettings::m_overrideRadius)
		.def_readwrite("m_thickness", &TreeMeshGeneratorSettings::m_radius)
		.def_readwrite("m_treePartBaseDistance", &TreeMeshGeneratorSettings::m_treePartBaseDistance)
		.def_readwrite("m_treePartEndDistance", &TreeMeshGeneratorSettings::m_treePartEndDistance)
		.def_readwrite("m_baseControlPointRatio", &TreeMeshGeneratorSettings::m_baseControlPointRatio)
		.def_readwrite("m_branchControlPointRatio", &TreeMeshGeneratorSettings::m_branchControlPointRatio)
		.def_readwrite("m_smoothness", &TreeMeshGeneratorSettings::m_smoothness)
		.def_readwrite("m_autoLevel", &TreeMeshGeneratorSettings::m_autoLevel)
		.def_readwrite("m_voxelSubdivisionLevel", &TreeMeshGeneratorSettings::m_voxelSubdivisionLevel)
		.def_readwrite("m_voxelSmoothIteration", &TreeMeshGeneratorSettings::m_voxelSmoothIteration)
		.def_readwrite("m_removeDuplicate", &TreeMeshGeneratorSettings::m_removeDuplicate)
		.def_readwrite("m_branchMeshType", &TreeMeshGeneratorSettings::m_branchMeshType);

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
	m.def("register_classes", &register_classes, "register_classes");
	m.def("register_layers", &register_layers, "register_layers");
	m.def("start_project_windowless", &start_project_windowless, "StartProjectWindowless");
	m.def("start_project_with_editor", &start_project, "start_project");

	m.def("tree_structor", &tree_structor, "TreeStructor");
	m.def("scene_capture", &scene_capture, "CaptureScene");
	m.def("yaml_visualization", &yaml_visualization, "yaml_visualization");
	m.def("voxel_space_colonization_tree_data", &voxel_space_colonization_tree_data, "voxel_space_colonization_tree_data");
	m.def("rbv_space_colonization_tree_data", &rbv_space_colonization_tree_data, "rbv_space_colonization_tree_data");
	m.def("rbv_to_obj", &rbv_to_obj, "rbv_to_obj");
	

	py::class_<DatasetGenerator>(m, "DatasetGenerator")
		.def(py::init<>())
		.def_static("GeneratePointCloudForTree", &DatasetGenerator::GeneratePointCloudForTree);

}