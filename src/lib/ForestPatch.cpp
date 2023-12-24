//
// Created by lllll on 10/25/2022.
//

#include "ForestPatch.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Climate.hpp"
#include "Tree.hpp"
#include "EcoSysLabLayer.hpp"
using namespace EcoSysLab;

void TreeInfo::Serialize(YAML::Emitter& out) const
{
	out << YAML::Key << "m_globalTransform" << YAML::Value << m_globalTransform.m_value;
	m_treeDescriptor.Save("m_treeDescriptor", out);
}

void TreeInfo::Deserialize(const YAML::Node& in)
{
	if (in["m_globalTransform"]) m_globalTransform.m_value = in["m_globalTransform"].as<glm::mat4>();
	m_treeDescriptor.Load("m_treeDescriptor", in);
}

void TreeInfo::CollectAssetRef(std::vector<AssetRef>& list) const
{
	list.push_back(m_treeDescriptor);
}

void ForestPatch::ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor)
{
	if (treeDescriptor) {
		for (auto& i : m_treeInfos)
		{
			i.m_treeDescriptor = treeDescriptor;
		}
	}
}

void ForestPatch::ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors)
{
	if(treeDescriptors.empty()) return;
	for(auto& i : m_treeInfos)
	{
		i.m_treeDescriptor = treeDescriptors.at(glm::linearRand(0, static_cast<int>(treeDescriptors.size()) - 1));
	}
}

void ForestPatch::ApplyTreeDescriptors(const std::filesystem::path& folderPath)
{
	std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
	for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath))
	{
		if (i.is_regular_file() && i.path().extension().string() == ".fp")
		{
			const auto treeDescriptor =
				std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
			collectedTreeDescriptors.emplace_back(treeDescriptor);
		}
	}
	ApplyTreeDescriptors(collectedTreeDescriptors);
}

void ForestPatch::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	static glm::ivec2 gridSize = { 5, 5 };
	static float gridDistance = 1.5f;
	static float randomShift = 0.25f;
	static bool setParent = true;
	static bool enableHistory = false;
	static int historyIteration = 30;
	ImGui::Checkbox("Enable history", &enableHistory);
	if (enableHistory) ImGui::DragInt("History iteration", &historyIteration, 1, 1, 999);
	if (ImGui::TreeNode("Grid...")) {
		ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
		ImGui::DragFloat("Grid distance", &gridDistance, 0.1f, 0.0f, 100.0f);
		ImGui::DragFloat("Random shift", &randomShift, 0.01f, 0.0f, 0.5f);
		if (ImGui::Button("Reset Grid")) {
			SetupGrid(gridSize, gridDistance, randomShift);
		}
		ImGui::TreePop();
	}
	/*
	FileUtils::OpenFolder("Parameters sample", [&](const std::filesystem::path& path)
		{
			int index = 0;
			const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
			const auto soil = ecoSysLabLayer->m_soilHolder.Get<Soil>();
			const auto soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
			std::shared_ptr<HeightField> heightField{};
			if (soilDescriptor)
			{
				heightField = soilDescriptor->m_heightField.Get<HeightField>();
			}
			for(const auto& i : std::filesystem::recursive_directory_iterator(path))
			{
				if(i.is_regular_file() && i.path().extension().string() == ".fp")
				{
					const auto treeDescriptor = 
						std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
					m_treeInfos.emplace_back();
					glm::vec3 position = glm::vec3(5.0f * (index ), 0.0f, index * 5.0f);
					if (heightField) position.y = heightField->GetValue({ position.x, position.z }) - 0.05f;
					m_treeInfos.back().m_globalTransform.SetPosition(position);
					m_treeInfos.back().m_treeDescriptor = treeDescriptor;
					index++;
				}
			}
		}, false);
	*/
	FileUtils::OpenFolder("Assign Tree Descriptors", [&](const std::filesystem::path& path)
		{
			ApplyTreeDescriptors(path);
		}, false);
	static AssetRef treeDescriptorRef;
	if (editorLayer->DragAndDropButton<TreeDescriptor>(treeDescriptorRef, "Apply all with treeDescriptor...", true))
	{
		if (const auto treeDescriptor = treeDescriptorRef.Get<TreeDescriptor>()) {
			ApplyTreeDescriptor(treeDescriptor);
		}
		treeDescriptorRef.Clear();
	}

	if (ImGui::TreeNode("Tree Instances"))
	{
		int index = 1;
		for (auto& i : m_treeInfos)
		{
			editorLayer->DragAndDropButton<TreeDescriptor>(i.m_treeDescriptor, "Tree No." + std::to_string(index), true);
			index++;
		}
		ImGui::TreePop();
	}

	if (ImGui::Button("Instantiate patch")) {
		InstantiatePatch(setParent);
	}


	if (!m_treeInfos.empty() && ImGui::Button("Clear")) {
		m_treeInfos.clear();
	}
}

void ForestPatch::OnCreate() {

}

void ForestPatch::CollectAssetRef(std::vector<AssetRef>& list) {
	for (const auto& i : m_treeInfos)
	{
		i.CollectAssetRef(list);
	}
}

void ForestPatch::Serialize(YAML::Emitter& out) {
	out << YAML::Key << "m_treeInfos" << YAML::BeginSeq;
	for (const auto& i : m_treeInfos)
	{
		i.Serialize(out);
	}
	out << YAML::EndSeq;

	out << YAML::Key << "m_treeGrowthSettings" << YAML::Value << YAML::BeginMap;
	Tree::SerializeTreeGrowthSettings(m_treeGrowthSettings, out);
	out << YAML::EndMap;
}

void ForestPatch::Deserialize(const YAML::Node& in) {
	if (in["m_treeInfos"])
	{
		m_treeInfos.clear();
		for (const auto& i : in["m_treeInfos"]) {
			m_treeInfos.emplace_back();
			auto& back = m_treeInfos.back();
			back.Deserialize(i);
		}
	}

	if (in["m_treeGrowthSettings"]) {
		Tree::DeserializeTreeGrowthSettings(m_treeGrowthSettings, in["m_treeGrowthSettings"]);
	}
}

void ForestPatch::SetupGrid(const glm::ivec2& gridSize, float gridDistance, float randomShift)
{
	m_treeInfos.clear();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto soil = ecoSysLabLayer->m_soilHolder.Get<Soil>();
	const auto soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
	std::shared_ptr<HeightField> heightField{};
	if (soilDescriptor)
	{
		heightField = soilDescriptor->m_heightField.Get<HeightField>();
	}
	const glm::vec2 startPoint = glm::vec2((gridSize.x - 1) * gridDistance, (gridSize.y - 1) * gridDistance) * 0.5f;
	for (int i = 0; i < gridSize.x; i++) {
		for (int j = 0; j < gridSize.y; j++) {
			m_treeInfos.emplace_back();
			glm::vec3 position = glm::vec3(-startPoint.x + i * gridDistance, 0.0f, -startPoint.y + j * gridDistance);
			position.x += glm::linearRand(-gridDistance * randomShift, gridDistance * randomShift);
			position.z += glm::linearRand(-gridDistance * randomShift, gridDistance * randomShift);
			if (heightField) position.y = heightField->GetValue({ position.x, position.z }) - 0.05f;
			m_treeInfos.back().m_globalTransform.SetPosition(position);
		}
	}
}

void ForestPatch::InstantiatePatch(const bool setParent)
{
	const auto scene = Application::GetActiveScene();
	Entity parent;
	if (setParent) {
		parent = scene->CreateEntity("Forest (" + std::to_string(m_treeInfos.size()) + ") - " + GetTitle());
	}
	int i = 0;
	for (const auto& gt : m_treeInfos) {
		auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(i));
		i++;
		scene->SetDataComponent(treeEntity, gt.m_globalTransform);
		const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
		tree->m_treeModel.m_treeGrowthSettings = m_treeGrowthSettings;
		tree->m_treeDescriptor = gt.m_treeDescriptor;
		if (setParent) scene->SetParent(treeEntity, parent);
	}
}
