#include "Climate.hpp"


#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;

void ClimateDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	if (ImGui::Button("Instantiate")) {
		const auto scene = Application::GetActiveScene();
		const auto climateEntity = scene->CreateEntity(GetTitle());
		const auto climate = scene->GetOrSetPrivateComponent<Climate>(climateEntity).lock();
		climate->m_climateDescriptor = ProjectManager::GetAsset(GetHandle());
	}
}

void ClimateDescriptor::Serialize(YAML::Emitter& out)
{
	
}

void ClimateDescriptor::Deserialize(const YAML::Node& in)
{
	
}

void Climate::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	if(editorLayer->DragAndDropButton<ClimateDescriptor>(m_climateDescriptor, "ClimateDescriptor", true))
	{
		InitializeClimateModel();
	}

	if (m_climateDescriptor.Get<ClimateDescriptor>())
	{

	}
		
}

void Climate::Serialize(YAML::Emitter& out)
{
	m_climateDescriptor.Save("m_climateDescriptor", out);
}

void Climate::CollectAssetRef(std::vector<AssetRef>& list)
{
	list.push_back(m_climateDescriptor);
}

void Climate::InitializeClimateModel()
{
	auto climateDescriptor = m_climateDescriptor.Get<ClimateDescriptor>();
	if (climateDescriptor)
	{
		auto params = climateDescriptor->m_climateParameters;
		m_climateModel.Initialize(params);
	}
}

void Climate::PrepareForGrowth()
{
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	if (!treeEntities || treeEntities->empty()) return;

	auto& estimator = m_climateModel.m_environmentGrid;
	estimator.m_settings = ecoSysLabLayer->m_simulationSettings.m_shadowEstimationSettings;
	auto minBound = estimator.m_voxel.GetMinBound();
	auto maxBound = estimator.m_voxel.GetMaxBound();
	bool boundChanged = false;
	for (const auto& treeEntity : *treeEntities)
	{
		const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
		const auto globalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity).m_value;
		const glm::vec3 currentMinBound = globalTransform * glm::vec4(tree->m_treeModel.RefShootSkeleton().m_min, 1.0f);
		const glm::vec3 currentMaxBound = globalTransform * glm::vec4(tree->m_treeModel.RefShootSkeleton().m_max, 1.0f);

		if (currentMinBound.x <= minBound.x || currentMinBound.y <= minBound.y || currentMinBound.z <= minBound.z
			|| currentMaxBound.x >= maxBound.x || currentMaxBound.y >= maxBound.y || currentMaxBound.z >= maxBound.z) {
			minBound = glm::min(currentMinBound - glm::vec3(1.0f, 0.1f, 1.0f), minBound);
			maxBound = glm::max(currentMaxBound + glm::vec3(1.0f), maxBound);
			boundChanged = true;
			//EVOENGINE_LOG("Shadow grid resized!");
		}
		tree->m_treeModel.m_crownShynessDistance = ecoSysLabLayer->m_simulationSettings.m_crownShynessDistance;
	}
	if (boundChanged) estimator.m_voxel.Initialize(estimator.m_voxelSize, minBound, maxBound);
	estimator.m_voxel.Reset();
	for (const auto& treeEntity : *treeEntities)
	{
		const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
		tree->RegisterVoxel();
	}

	estimator.ShadowPropagation();
}

void Climate::Deserialize(const YAML::Node& in)
{
	m_climateDescriptor.Load("m_climateDescriptor", in);
}
