//
// Created by lllll on 10/17/2022.
//

#include "CBTFGroup.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include "DoubleCBTF.hpp"
#endif

using namespace EcoSysLab;
bool CBTFGroup::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	bool changed = false;
#ifdef BUILD_WITH_RAYTRACER
	AssetRef temp;
	if (editorLayer->DragAndDropButton<DoubleCBTF>(temp, ("Drop to add..."))) {
		m_doubleCBTFs.emplace_back(temp);
		temp.Clear();
	}

	if (ImGui::TreeNodeEx("List", ImGuiTreeNodeFlags_DefaultOpen)) {
		for (int i = 0; i < m_doubleCBTFs.size(); i++) {
			if (editorLayer->DragAndDropButton<DoubleCBTF>(
				m_doubleCBTFs[i], ("No." + std::to_string(i + 1))) &&
				!m_doubleCBTFs[i].Get<DoubleCBTF>()) {
				m_doubleCBTFs.erase(m_doubleCBTFs.begin() + i);
				i--;
			}
		}
		ImGui::TreePop();
	}
#endif
	return changed;
}

void CBTFGroup::CollectAssetRef(std::vector<AssetRef>& list) {
	for (const auto& i : m_doubleCBTFs)
		list.push_back(i);
}
void CBTFGroup::Serialize(YAML::Emitter& out) const {
	if (!m_doubleCBTFs.empty()) {
		out << YAML::Key << "m_doubleCBTFs" << YAML::Value << YAML::BeginSeq;
		for (auto& cBTF : m_doubleCBTFs) {
			out << YAML::BeginMap;
			cBTF.Serialize(out);
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
	}
}
void CBTFGroup::Deserialize(const YAML::Node& in) {
	auto inCBTFs = in["m_doubleCBTFs"];
	if (inCBTFs) {
		for (const auto& i : inCBTFs) {
			AssetRef ref;
			ref.Deserialize(i);
			m_doubleCBTFs.emplace_back(ref);
		}
	}
}
AssetRef CBTFGroup::GetRandom() const {
	if (!m_doubleCBTFs.empty()) {
		return m_doubleCBTFs[glm::linearRand(0, (int)m_doubleCBTFs.size() - 1)];
	}
	return {};
}
