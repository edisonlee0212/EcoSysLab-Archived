//
// Created by lllll on 10/25/2022.
//

#include "Trees.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Tree.hpp"
using namespace EcoSysLab;

void Trees::OnInspect() {
    static glm::ivec2 gridSize = {8, 8};
    static glm::vec2 gridDistance = {20, 20};
    static bool setParent = true;
    static bool enableHistory = true;
    ImGui::Checkbox("Enable history", &enableHistory);
    ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
    ImGui::DragFloat2("Grid distance", &gridDistance.x, 0.1f, 0.0f, 100.0f);
    if (ImGui::Button("Reset Grid")) {
        m_globalTransforms.clear();
        for (int i = 0; i < gridSize.x; i++) {
            for (int j = 0; j < gridSize.y; j++) {
                m_globalTransforms.emplace_back();
                m_globalTransforms.back().SetPosition(glm::vec3(i * gridDistance.x, 0.0f, j * gridDistance.y));
            }
        }
    }
    Editor::DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true);
    if (!m_globalTransforms.empty() && m_treeDescriptor.Get<TreeDescriptor>()) {
        auto &parameters = m_treeDescriptor.Get<TreeDescriptor>()->m_treeStructuralGrowthParameters;
        if (ImGui::Button("Instantiate trees")) {
            auto scene = Application::GetActiveScene();
            Entity parent;
            if(setParent){
                parent = scene->CreateEntity("Trees (" + std::to_string(m_globalTransforms.size()) + ") - " + GetTitle());
            }
            int i = 0;
            for(const auto& gt : m_globalTransforms){
                auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(i));
                i++;
                scene->SetDataComponent(treeEntity, gt);
                auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                tree->m_treeDescriptor = m_treeDescriptor;
                tree->m_enableHistory = enableHistory;
                if(setParent) scene->SetParent(treeEntity, parent);
            }
        }
    }

    if (!m_globalTransforms.empty() && ImGui::Button("Clear")) {
        m_globalTransforms.clear();
    }
}

void Trees::OnCreate() {

}

void Trees::CollectAssetRef(std::vector<AssetRef> &list) {
    list.push_back(m_treeDescriptor);
}

void Trees::Serialize(YAML::Emitter &out) {
    m_treeDescriptor.Save("m_treeDescriptor", out);
}

void Trees::Deserialize(const YAML::Node &in) {
    m_treeDescriptor.Load("m_treeDescriptor", in);
}
