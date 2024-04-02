#include "DatasetGenerator.hpp"
#include "Climate.hpp"
#include "Soil.hpp"
#include "EcoSysLabLayer.hpp"
#include "ForestPatch.hpp"
#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
using namespace EcoSysLab;

void DatasetGenerator::GenerateTreeTrunkMesh(const std::string& treeParametersPath, float deltaTime,
	int maxIterations, int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const std::string& treeMeshOutputPath,
	const std::string& treeTrunkOutputPath,
	const std::string& treeInfoPath)
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
	tree->m_treeModel.m_treeGrowthSettings.m_useSpaceColonization = false;
	Application::Loop();
	for (int i = 0; i < maxIterations; i++)
	{
		ecoSysLabLayer->Simulate(deltaTime);
		if (tree->m_treeModel.RefShootSkeleton().PeekSortedNodeList().size() >= maxTreeNodeCount)
		{
			break;
		}
	}
	tree->GenerateGeometryEntities(meshGeneratorSettings);
	Application::Loop();
	tree->ExportTrunkOBJ(treeTrunkOutputPath, meshGeneratorSettings);
	tree->ExportOBJ(treeMeshOutputPath, meshGeneratorSettings);
	Application::Loop();
	float trunkHeight = 0.0f;
	float baseDiameter = 0.0f;
	float topDiameter = 0.0f;
	const auto& skeleton = tree->m_treeModel.RefShootSkeleton();
	const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
	if (sortedInternodeList.size() > 1) {
		std::unordered_set<NodeHandle> trunkHandles{};
		for (const auto& nodeHandle : sortedInternodeList)
		{
			const auto& node = skeleton.PeekNode(nodeHandle);
			trunkHandles.insert(nodeHandle);
			if (node.RefChildHandles().size() > 1) {
				trunkHeight = node.m_info.GetGlobalEndPosition().y;
				topDiameter = node.m_info.m_thickness;
				break;
			}
		}
		baseDiameter = skeleton.PeekNode(0).m_info.m_thickness;
		std::ofstream of;
		of.open(treeInfoPath, std::ofstream::out | std::ofstream::trunc);
		std::stringstream data;
		data << "TrunkHeight " << std::to_string(trunkHeight) << "\n";
		data << "TrunkBaseDiameter " << std::to_string(baseDiameter) << "\n";
		data << "TrunkTopDiameter " << std::to_string(topDiameter) << "\n";

		data << "TreeBoundingBoxMinX " << std::to_string(skeleton.m_min.x) << "\n";
		data << "TreeBoundingBoxMinY " << std::to_string(skeleton.m_min.y) << "\n";
		data << "TreeBoundingBoxMinZ " << std::to_string(skeleton.m_min.z) << "\n";
		data << "TreeBoundingBoxMaxX " << std::to_string(skeleton.m_max.x) << "\n";
		data << "TreeBoundingBoxMaxY " << std::to_string(skeleton.m_max.y) << "\n";
		data << "TreeBoundingBoxMaxZ " << std::to_string(skeleton.m_max.z) << "\n";
		const auto result = data.str();
		of.write(result.c_str(), result.size());
		of.flush();
	}
	scene->DeleteEntity(treeEntity);
	Application::Loop();
}

void DatasetGenerator::GenerateTreeMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
	int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::string& treeMeshOutputPath)
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
	tree->m_treeModel.m_treeGrowthSettings.m_useSpaceColonization = false;
	Application::Loop();
	for (int i = 0; i < maxIterations; i++)
	{
		ecoSysLabLayer->Simulate(deltaTime);
		if (tree->m_treeModel.RefShootSkeleton().PeekSortedNodeList().size() >= maxTreeNodeCount)
		{
			break;
		}
	}
	tree->GenerateGeometryEntities(meshGeneratorSettings);
	Application::Loop();
	tree->ExportOBJ(treeMeshOutputPath, meshGeneratorSettings);
	Application::Loop();
	scene->DeleteEntity(treeEntity);
	Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForTree(const TreePointCloudPointSettings& pointSettings,
	const std::shared_ptr<PointCloudCaptureSettings>& captureSettings, const std::string& treeParametersPath, const float deltaTime,
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
	tree->m_treeModel.m_treeGrowthSettings.m_useSpaceColonization = false;
	Application::Loop();
	for (int i = 0; i < maxIterations; i++)
	{
		ecoSysLabLayer->Simulate(deltaTime);
		if (tree->m_treeModel.RefShootSkeleton().PeekSortedNodeList().size() >= maxTreeNodeCount)
		{
			break;
		}
	}
	tree->GenerateGeometryEntities(meshGeneratorSettings);
	Application::Loop();
	if (exportTreeMesh) {
		tree->ExportOBJ(treeMeshOutputPath, meshGeneratorSettings);
	}
	Application::Loop();
	const auto scannerEntity = scene->CreateEntity("Scanner");
	const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
	scanner->m_pointSettings = pointSettings;
	Application::Loop();
	scanner->Capture(meshGeneratorSettings, pointCloudOutputPath, captureSettings);
	if (exportJunction) tree->ExportJunction(meshGeneratorSettings, treeJunctionOutputPath);
	Application::Loop();
	scene->DeleteEntity(treeEntity);
	scene->DeleteEntity(scannerEntity);
	Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatch(
	const int gridSize, const float gridDistance, const float randomShift,
	const TreePointCloudPointSettings& pointSettings,
	const std::shared_ptr<PointCloudCaptureSettings>& captureSettings, const std::string& treeParametersFolderPath,
	float deltaTime, int maxIterations, int maxTreeNodeCount,
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
	std::shared_ptr<ForestPatch> forestPatch = ProjectManager::CreateTemporaryAsset<ForestPatch>();
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

	soil->RandomOffset(0, 99999);
	soil->GenerateMesh(0.0f, 0.0f);
	if (const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>(); treeEntities && !treeEntities->empty())
	{
		for (const auto& treeEntity : *treeEntities)
		{
			scene->DeleteEntity(treeEntity);
		}
	}
	forestPatch->m_treeGrowthSettings.m_useSpaceColonization = false;

	Application::Loop();
	
	forestPatch->SetupGrid({ gridSize, gridSize }, gridDistance, randomShift);
	forestPatch->ApplyTreeDescriptors(treeParametersFolderPath, {1.f});
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
	scanner->m_pointSettings = pointSettings;
	Application::Loop();
	scanner->Capture(meshGeneratorSettings, pointCloudOutputPath, captureSettings);

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

void DatasetGenerator::GeneratePointCloudForSorghumPatch(
	const RectangularSorghumFieldPattern& pattern,
	const std::shared_ptr<SorghumDescriptor>& sorghumDescriptor,
	const SorghumPointCloudPointSettings& pointSettings, 
	const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
	const SorghumMeshGeneratorSettings& sorghumMeshGeneratorSettings, const std::string& pointCloudOutputPath)
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
	Application::Loop();
	const auto sorghumField = ProjectManager::CreateTemporaryAsset<SorghumField>();
	std::vector<glm::mat4> matricesList;
	pattern.GenerateField(matricesList);
	sorghumField->m_matrices.resize(matricesList.size());
	for(int i = 0; i < matricesList.size(); i++)
	{
		sorghumField->m_matrices[i] = { sorghumDescriptor, matricesList[i] };
	}

	const auto field = sorghumField->InstantiateField();
	Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(sorghumMeshGeneratorSettings);
	Application::Loop();
	const auto scannerEntity = scene->CreateEntity("Scanner");
	const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scannerEntity).lock();
	scanner->m_sorghumPointCloudPointSettings = pointSettings;
	Application::Loop();
	scanner->Capture(pointCloudOutputPath, captureSettings);
	scene->DeleteEntity(field);
	scene->DeleteEntity(scannerEntity);
	Application::Loop();
}
