#pragma once
#include "Tree.hpp"
using namespace UniEngine;
namespace EcoSysLab {

    class Trees : public IPrivateComponent{
    public:
        AssetRef m_treeDescriptor;
        TreeModelGroup m_treeModelGroup;
        bool m_enableHistory = true;
        int m_iteration = 0;
        void OnInspect() override;

        void OnCreate() override;

        void OnDestroy() override;
    };
}