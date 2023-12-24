#include "DatasetGenerator.hpp"
#include "Climate.hpp"
#include "Soil.hpp"
#include "EcoSysLabLayer.hpp"
#include "ForestPatch.hpp"
using namespace EcoSysLab;

void DatasetGenerator::GeneratePointCloudForTree(const PointCloudPointSettings& pointSettings,
	const PointCloudCircularCaptureSettings& captureSettings, const std::string& treeParametersPath, const float deltaTime,
	const int maxIterations, const int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const std::string& pointCloudOutputPath, bool exportTreeMesh, const std::string& treeMeshOutputPath,
	bool exportJunction, const std::string& treeJunctionOutputPath)
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
	std::shared_ptr<TreeDescriptor> treeDescriptor;
	if (ProjectManager::IsInProjectFolder(treeParametersPath))
	{
		treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
	}
	else {
		EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
		return;
	}
	if (const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>(); treeEntities && !treeEntities->empty())
	{
		for (const auto& treeEntity : *treeEntities)
		{
			scene->DeleteEntity(treeEntity);
		}
	}

	const auto treeEntity = scene->CreateEntity("Tree");
	const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
	
	tree->m_treeDescriptor = treeDescriptor;
	tree->m_treeModel.m_treeGrowthSettings.m_enableShoot = true;
	tree->m_treeModel.m_treeGrowthSettings.m_enableRoot = false;
	tree->m_treeModel.m_treeGrowthSettings.m_useSpaceColonization = false;
	Application::Loop();
	for (int i = 0; i < maxIterations; i++)
	{
		ecoSysLabLayer->Simulate(deltaTime);
		if (tree->m_treeModel.RefShootSkeleton().RefSortedNodeList().size() >= maxTreeNodeCount)
		{
			break;
		}
	}
	tree->GenerateGeometry(meshGeneratorSettings);
	Application::Loop();
	if (exportTreeMesh) {
		tree->ExportOBJ(treeMeshOutputPath, meshGeneratorSettings);
	}
	Application::Loop();
	const auto scannerEntity = scene->CreateEntity("Scanner");
	const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
	std::shared_ptr<PointCloudCircularCaptureSettings> pointCloudCircularCaptureSettings = std::make_shared<PointCloudCircularCaptureSettings>(captureSettings);
	scanner->m_pointSettings = pointSettings;
	Application::Loop();
	scanner->Capture(pointCloudOutputPath, pointCloudCircularCaptureSettings);
	Application::Loop();
	scene->DeleteEntity(treeEntity);
	scene->DeleteEntity(scannerEntity);
	Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatch(
	const int gridSize, const float gridDistance, const float randomShift, 
	const PointCloudPointSettings& pointSettings,
	const PointCloudGridCaptureSettings& captureSettings, const std::string& treeParametersPath,
	const std::string& forestPatchPath, float deltaTime, int maxIterations, int maxTreeNodeCount,
	const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::string& pointCloudOutputPath)
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
	std::shared_ptr<ForestPatch> forestPatch;
	if (ProjectManager::IsInProjectFolder(forestPatchPath))
	{
		forestPatch = std::dynamic_pointer_cast<ForestPatch>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(forestPatchPath)));
	}
	else {
		EVOENGINE_ERROR("Forest patch doesn't exist!");
		return;
	}
	std::shared_ptr<TreeDescriptor> treeDescriptor;
	if (ProjectManager::IsInProjectFolder(treeParametersPath))
	{
		treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
	}
	else {
		EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
		return;
	}

	std::shared_ptr<Soil> soil;

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
	soil->GenerateMesh(0.0f, 0.0f);
	if (const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>(); treeEntities && !treeEntities->empty())
	{
		for (const auto& treeEntity : *treeEntities)
		{
			scene->DeleteEntity(treeEntity);
		}
	}
	forestPatch->m_treeGrowthSettings.m_enableShoot = true;
	forestPatch->m_treeGrowthSettings.m_enableRoot = false;
	forestPatch->m_treeGrowthSettings.m_useSpaceColonization = false;

	Application::Loop();
	forestPatch->SetupGrid({ gridSize, gridSize }, gridDistance, randomShift);
	forestPatch->ApplyTreeDescriptors({ treeDescriptor });
	forestPatch->InstantiatePatch(false);
	
	ecoSysLabLayer->m_simulationSettings.m_maxNodeCount = maxTreeNodeCount;
	
	for (int i = 0; i < maxIterations; i++)
	{
		ecoSysLabLayer->Simulate(deltaTime);
	}
	ecoSysLabLayer->GenerateMeshes(meshGeneratorSettings);
	Application::Loop();
	const auto scannerEntity = scene->CreateEntity("Scanner");
	const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
	const std::shared_ptr<PointCloudGridCaptureSettings> pointCloudGridCaptureSettings = std::make_shared<PointCloudGridCaptureSettings>(captureSettings);
	scanner->m_pointSettings = pointSettings;
	Application::Loop();
	scanner->Capture(pointCloudOutputPath, pointCloudGridCaptureSettings);
	if (const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>(); treeEntities && !treeEntities->empty())
	{
		for(const auto& treeEntity : *treeEntities)
		{
			scene->DeleteEntity(treeEntity);
		}
	}
	scene->DeleteEntity(scannerEntity);
	Application::Loop();
}
