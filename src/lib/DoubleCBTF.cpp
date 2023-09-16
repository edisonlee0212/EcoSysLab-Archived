//
// Created by lllll on 11/16/2022.
//

#include "DoubleCBTF.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include "CompressedBTF.hpp"
#endif
using namespace EcoSysLab;
void DoubleCBTF::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
#ifdef BUILD_WITH_RAYTRACER
	editorLayer->DragAndDropButton<CompressedBTF>(m_top, "Top");
	editorLayer->DragAndDropButton<CompressedBTF>(m_bottom, "Bottom");
#endif
}
void DoubleCBTF::CollectAssetRef(std::vector<AssetRef>& list) {
	list.push_back(m_top);
	list.push_back(m_bottom);
}
void DoubleCBTF::Serialize(YAML::Emitter& out) {
	m_top.Save("m_top", out);
	m_bottom.Save("m_bottom", out);
}
void DoubleCBTF::Deserialize(const YAML::Node& in) {
	m_top.Load("m_top", in);
	m_bottom.Load("m_bottom", in);
}
