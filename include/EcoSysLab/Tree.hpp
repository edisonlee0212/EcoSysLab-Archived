#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
using namespace UniEngine;
namespace EcoSysLab {
    class TreeDescriptor : public IAsset {
    public:
        TreeStructuralGrowthParameters m_treeStructuralGrowthParameters;

        void OnCreate() override;

        void OnInspect() override;

        void CollectAssetRef(std::vector<AssetRef> &list) override;

        void Serialize(YAML::Emitter &out) override;

        void Deserialize(const YAML::Node &in) override;
    };

    class Tree : public IPrivateComponent {
    public:
        AssetRef m_treeDescriptor;

        TreeModel m_treeModel;

        void OnInspect() override;

        void OnDestroy() override;

        void OnCreate() override;
    };
}