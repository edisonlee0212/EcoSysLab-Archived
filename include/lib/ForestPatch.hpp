#pragma once
#include "Tree.hpp"
using namespace EvoEngine;
namespace EcoSysLab {

    struct TreeInfo
    {
        GlobalTransform m_globalTransform{};
        AssetRef m_treeDescriptor{};
        void Serialize(YAML::Emitter& out) const;
        void Deserialize(const YAML::Node& in);
        void CollectAssetRef(std::vector<AssetRef>& list) const;
    };

    class ForestPatch : public IAsset{
    public:
        std::vector<TreeInfo> m_treeInfos;
        TreeGrowthSettings m_treeGrowthSettings;
        
        void ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor);
        void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors);
        void ApplyTreeDescriptors(const std::filesystem::path& folderPath);
        void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors, const std::vector<float>& ratios);
        void ApplyTreeDescriptors(const std::filesystem::path& folderPath, const std::vector<float>& ratios);
        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void OnCreate() override;

        void CollectAssetRef(std::vector<AssetRef> &list) override;

        void Serialize(YAML::Emitter &out) override;

        void Deserialize(const YAML::Node &in) override;

        void SetupGrid(const glm::ivec2& gridSize, float gridDistance, float randomShift);

        void InstantiatePatch(bool setParent);
    };
}