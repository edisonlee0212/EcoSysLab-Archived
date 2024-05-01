//
// Created by lllll on 10/25/2022.
//

#include "ForestDescriptor.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Climate.hpp"
#include "Tree.hpp"
#include "EcoSysLabLayer.hpp"
using namespace EcoSysLab;

void ForestPatch::InstantiatePatch(const bool setParent)
{
	const auto scene = Application::GetActiveScene();
	std::vector<GlobalTransform> treeMatrices;
	std::shared_ptr<Soil> soil;
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) soil = soilCandidate.lock();
	std::shared_ptr<SoilDescriptor> soilDescriptor;
	if (soil) {
		soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
	}
	std::shared_ptr<HeightField> heightField{};
	if (soilDescriptor)
	{
		heightField = soilDescriptor->m_heightField.Get<HeightField>();
	}
	treeMatrices.resize(m_gridSize.x * m_gridSize.y);
	const glm::vec2 startPoint = glm::vec2((m_gridSize.x - 1) * m_gridDistance.x, (m_gridSize.y - 1) * m_gridDistance.y) * 0.5f;
	for (int i = 0; i < m_gridSize.x; i++) {
		for (int j = 0; j < m_gridSize.y; j++) {
			glm::vec3 position = glm::vec3(-startPoint.x + i * m_gridDistance.x, 0.0f, -startPoint.y + j * m_gridDistance.y);
			position.x += glm::linearRand(-m_gridDistance.x * m_positionOffsetMean.x, m_gridDistance.x * m_positionOffsetMean.x);
			position.z += glm::linearRand(-m_gridDistance.y * m_positionOffsetMean.y, m_gridDistance.y * m_positionOffsetMean.y);
			position += glm::gaussRand(glm::vec3(0.0f), glm::vec3(m_positionOffsetVariance.x, 0.0f, m_positionOffsetVariance.y));
			if (heightField) position.y = heightField->GetValue({ position.x, position.z }) - 0.01f;
			GlobalTransform transform{};
			transform.SetPosition(position);
			auto rotation = glm::quat(glm::radians(
				glm::vec3(glm::gaussRand(glm::vec3(0.0f), m_rotationOffsetVariance))));
			transform.SetRotation(rotation);
			transform.SetScale(glm::vec3(1.f));
			treeMatrices[i * m_gridSize.y + j] = transform;
		}
	}

	Entity parent;
	if (setParent) {
		parent = scene->CreateEntity("Forest (" + std::to_string(treeMatrices.size()) + ") - " + GetTitle());
	}
	int i = 0;
	for (const auto& gt : treeMatrices) {
		auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(i));
		i++;

		scene->SetDataComponent(treeEntity, gt);
		const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
		tree->m_treeModel.m_treeGrowthSettings = m_treeGrowthSettings;
		tree->m_treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		if (setParent) scene->SetParent(treeEntity, parent);
	}
}

void ForestPatch::CollectAssetRef(std::vector<AssetRef>& list)
{
	if (m_treeDescriptor.Get<TreeDescriptor>()) list.push_back(m_treeDescriptor);
}

void ForestPatch::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_gridDistance" << YAML::Value << m_gridDistance;
	out << YAML::Key << "m_positionOffsetMean" << YAML::Value << m_positionOffsetMean;
	out << YAML::Key << "m_positionOffsetVariance" << YAML::Value << m_positionOffsetVariance;
	out << YAML::Key << "m_rotationOffsetVariance" << YAML::Value << m_rotationOffsetVariance;
	out << YAML::Key << "m_gridSize" << YAML::Value << m_gridSize;
	m_treeDescriptor.Save("m_treeDescriptor", out);

}

void ForestPatch::Deserialize(const YAML::Node& in)
{
	if (in["m_gridDistance"]) m_gridDistance = in["m_gridDistance"].as<glm::vec2>();
	if (in["m_positionOffsetMean"]) m_positionOffsetMean = in["m_positionOffsetMean"].as<glm::vec2>();
	if (in["m_positionOffsetVariance"]) m_positionOffsetVariance = in["m_positionOffsetVariance"].as<glm::vec2>();
	if (in["m_rotationOffsetVariance"]) m_rotationOffsetVariance = in["m_rotationOffsetVariance"].as<glm::vec3>();
	if (in["m_gridSize"]) m_gridSize = in["m_gridSize"].as<glm::ivec2>();
	m_treeDescriptor.Load("m_treeDescriptor", in);

}

void ForestPatch::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	editorLayer->DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor");
	ImGui::DragInt2("Grid size", &m_gridSize.x, 1, 0, 100);
	ImGui::DragFloat2("Grid distance", &m_gridDistance.x, 0.1f, 0.0f, 100.0f);
	ImGui::Separator();
	ImGui::DragFloat2("Position offset mean", &m_positionOffsetMean.x, 0.01f, 0.0f, 5.f);
	ImGui::DragFloat2("Position offset variance", &m_positionOffsetVariance.x, 0.01f, 0.0f, 5.f);
	ImGui::DragFloat2("Rotation offset variance", &m_rotationOffsetVariance.x, 0.01f, 0.0f, 5.f);
	static bool setParent = true;
	ImGui::Checkbox("Set Parent", &setParent);
	if (ImGui::Button("Instantiate")) {
		InstantiatePatch(setParent);
	}
}

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

void ForestDescriptor::ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor)
{
	if (treeDescriptor) {
		for (auto& i : m_treeInfos)
		{
			i.m_treeDescriptor = treeDescriptor;
		}
	}
}

void ForestDescriptor::ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors)
{
	if (treeDescriptors.empty()) return;
	for (auto& i : m_treeInfos)
	{
		i.m_treeDescriptor = treeDescriptors.at(glm::linearRand(0, static_cast<int>(treeDescriptors.size()) - 1));
	}
}

void ForestDescriptor::ApplyTreeDescriptors(const std::filesystem::path& folderPath)
{
	std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
	for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath))
	{
		if (i.is_regular_file() && i.path().extension().string() == ".td")
		{
			const auto treeDescriptor =
				std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
			collectedTreeDescriptors.emplace_back(treeDescriptor);
		}
	}
	ApplyTreeDescriptors(collectedTreeDescriptors);
}

void ForestDescriptor::ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors,
	const std::vector<float>& ratios)
{
	if (treeDescriptors.empty()) return;
	for (auto& i : m_treeInfos)
	{
		i.m_treeDescriptor = treeDescriptors.at(glm::linearRand(0, static_cast<int>(treeDescriptors.size()) - 1));
	}

	std::random_device rd;
	std::mt19937 g(rd());
	auto copiedDescriptors = treeDescriptors;
	std::shuffle(copiedDescriptors.begin(), copiedDescriptors.end(), g);
	std::vector<std::shared_ptr<TreeDescriptor>> appliedTreeDescriptors;
	int count = 0;
	for (int i = 0; i < ratios.size(); i++)
	{
		if (count >= m_treeInfos.size()) break;
		const int localSize = m_treeInfos.size() * ratios[i];
		for (int j = 0; j < localSize; j++)
		{
			if (count >= m_treeInfos.size()) break;
			m_treeInfos[count].m_treeDescriptor = copiedDescriptors[i];
			count++;
		}
	}
}

void ForestDescriptor::ApplyTreeDescriptors(const std::filesystem::path& folderPath, const std::vector<float>& ratios)
{
	std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
	for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath))
	{
		if (i.is_regular_file() && i.path().extension().string() == ".tree")
		{
			const auto treeDescriptor =
				std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
			collectedTreeDescriptors.emplace_back(treeDescriptor);
		}
	}
	ApplyTreeDescriptors(collectedTreeDescriptors, ratios);
}

void ForestDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	static glm::ivec2 gridSize = { 4, 4 };
	static float gridDistance = 1.5f;
	static float randomShift = 0.5f;
	static bool setParent = true;
	static bool enableHistory = false;
	static int historyIteration = 30;
	ImGui::Checkbox("Enable history", &enableHistory);
	if (enableHistory) ImGui::DragInt("History iteration", &historyIteration, 1, 1, 999);
	if (ImGui::TreeNodeEx("Grid...", ImGuiTreeNodeFlags_DefaultOpen)) {
		ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
		ImGui::DragFloat("Grid distance", &gridDistance, 0.1f, 0.0f, 100.0f);
		ImGui::DragFloat("Random shift", &randomShift, 0.01f, 0.0f, 0.5f);
		if (ImGui::Button("Reset Grid")) {
			SetupGrid(gridSize, gridDistance, randomShift);
		}
		ImGui::TreePop();
	}

	FileUtils::OpenFolder("Parameters sample", [&](const std::filesystem::path& path)
		{
			int index = 0;
			const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
			std::shared_ptr<Soil> soil;
			const auto soilCandidate = EcoSysLabLayer::FindSoil();
			if (!soilCandidate.expired()) soil = soilCandidate.lock();
			std::shared_ptr<SoilDescriptor> soilDescriptor;
			if (soil) {
				soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
			}
			std::shared_ptr<HeightField> heightField{};
			if (soilDescriptor)
			{
				heightField = soilDescriptor->m_heightField.Get<HeightField>();
			}
			for (const auto& i : std::filesystem::recursive_directory_iterator(path))
			{
				if (i.is_regular_file() && i.path().extension().string() == ".td")
				{
					const auto treeDescriptor =
						std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
					m_treeInfos.emplace_back();
					glm::vec3 position = glm::vec3(5.f * index, 0.0f, 0.0f);
					if (heightField) position.y = heightField->GetValue({ position.x, position.z }) - 0.05f;
					m_treeInfos.back().m_globalTransform.SetPosition(position);
					m_treeInfos.back().m_treeDescriptor = treeDescriptor;
					index++;
				}
			}
		}, false);

	FileUtils::OpenFolder("Randomly assign tree descriptors", [&](const std::filesystem::path& path)
		{
			ApplyTreeDescriptors(path);
		}, false);
	static AssetRef treeDescriptorRef;
	if (editorLayer->DragAndDropButton<TreeDescriptor>(treeDescriptorRef, "Apply all with tree descriptor...", true))
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

void ForestDescriptor::OnCreate() {

}

void ForestDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
	for (const auto& i : m_treeInfos)
	{
		i.CollectAssetRef(list);
	}
}

void ForestDescriptor::Serialize(YAML::Emitter& out) {
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

void ForestDescriptor::Deserialize(const YAML::Node& in) {
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

void ForestDescriptor::SetupGrid(const glm::ivec2& gridSize, float gridDistance, float randomShift)
{
	m_treeInfos.clear();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	std::shared_ptr<Soil> soil;
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) soil = soilCandidate.lock();
	std::shared_ptr<SoilDescriptor> soilDescriptor;
	if (soil) {
		soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
	}
	std::shared_ptr<HeightField> heightField{};
	if (soilDescriptor)
	{
		heightField = soilDescriptor->m_heightField.Get<HeightField>();
	}
	const glm::vec2 startPoint = glm::vec2((gridSize.x - 0.5f) * gridDistance, (gridSize.y - 0.5f) * gridDistance) * 0.5f;
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

void ForestDescriptor::InstantiatePatch(const bool setParent)
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
