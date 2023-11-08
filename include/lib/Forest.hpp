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

    class ForestDescriptor : public IAsset{
    public:
        std::vector<TreeInfo> m_treeInfos;
        TreeGrowthSettings m_treeGrowthSettings;

        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void OnCreate() override;

        void CollectAssetRef(std::vector<AssetRef> &list) override;

        void Serialize(YAML::Emitter &out) override;

        void Deserialize(const YAML::Node &in) override;
    };
}