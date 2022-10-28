#pragma once
#include "Tree.hpp"
using namespace UniEngine;
namespace EcoSysLab {
    struct TreeInstance{
        Transform m_transform;
        TreeModel m_treeModel;
    };

    class Trees : public IPrivateComponent{
    public:
        AssetRef m_treeDescriptor;
        std::vector<TreeInstance> m_trees;

        void OnInspect() override;

        void OnCreate() override;

        void OnDestroy() override;
    };
}