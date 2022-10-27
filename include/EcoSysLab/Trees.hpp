#pragma once
#include "Tree.hpp"
using namespace UniEngine;
namespace EcoSysLab {
    struct TreeInstance{
        AssetRef m_treeDescriptor;
        Transform m_transform;
        TreeModel m_treeModel;
    };

    class Trees : public IPrivateComponent{
    public:
        std::vector<TreeInstance> m_trees;

        void OnInspect() override;

        void OnCreate() override;

        void OnDestroy() override;
    };
}