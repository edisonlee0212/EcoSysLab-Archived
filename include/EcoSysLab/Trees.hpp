#pragma once
#include "Tree.hpp"
using namespace UniEngine;
namespace EcoSysLab {

    class Trees : public IPrivateComponent{
    public:
        AssetRef m_treeDescriptor;
        TreeModelGroup m_treeModelGroup;

        void OnInspect() override;

        void OnCreate() override;

        void OnDestroy() override;
    };
}