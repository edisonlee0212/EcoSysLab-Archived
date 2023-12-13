#include "DatasetGenerator.hpp"
#include "Climate.hpp"
#include "Soil.hpp"
#include "EcoSysLabLayer.hpp"
using namespace EcoSysLab;

void DatasetGenerator::GeneratePointCloudForTree(const PointCloudPointSettings& pointSettings,
	const PointCloudCaptureSettings& captureSettings, const std::string& treeParametersPath, const float deltaTime,
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
	const auto treeEntity = scene->CreateEntity("Tree");
	const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
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
	soil->GenerateMesh();
	Application::Loop();
	if (exportTreeMesh) {
		tree->ExportOBJ(treeMeshOutputPath, meshGeneratorSettings);
	}
	Application::Loop();
	const auto scannerEntity = scene->CreateEntity("Scanner");
	const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
	scanner->m_captureSettings = captureSettings;
	scanner->m_pointSettings = pointSettings;
	Application::Loop();
	scanner->GeneratePointCloud(pointCloudOutputPath);
	if (exportJunction) tree->ExportJunction(meshGeneratorSettings, treeJunctionOutputPath);
	Application::Loop();
	scene->DeleteEntity(treeEntity);
	scene->DeleteEntity(scannerEntity);
	Application::Loop();
}
