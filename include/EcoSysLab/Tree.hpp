#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
using namespace UniEngine;
namespace EcoSysLab {
    class TreeDescriptor : public IAsset{
    public:
        TreeStructuralGrowthParameters m_treeStructuralGrowthParameters;

        void OnCreate() override;

        void OnInspect() override;

        void CollectAssetRef(std::vector<AssetRef> &list) override;

        void Serialize(YAML::Emitter &out) override;

        void Deserialize(const YAML::Node &in) override;
    };

    class TreeVisualizer{
        std::vector<InternodeHandle> m_sortedInternodeList;
        std::vector<BranchHandle> m_sortedBranchList;
        std::vector<glm::mat4> m_matrices;
        std::vector<glm::vec4> m_colors;
        bool m_visualization = true;
        bool m_treeHierarchyGui = true;
        InternodeHandle m_selectedInternodeHandle = -1;
        std::vector<InternodeHandle> m_selectedInternodeHierarchyList;
        int m_version = -1;
        bool RayCastSelection(TreeModel& treeModel, const GlobalTransform& globalTransform);
        bool DrawInternodeMenu(TreeModel& treeModel, InternodeHandle internodeHandle);
        void SetSelectedInternode(TreeModel& treeModel, InternodeHandle internodeHandle);
        bool DrawInternodeInspectionGui(TreeModel& treeModel, InternodeHandle internodeHandle, bool& deleted, const unsigned &hierarchyLevel);
        void InspectInternode(TreeModel& treeModel, InternodeHandle internodeHandle);
    public:
        bool OnInspect(TreeModel& treeModel, const GlobalTransform& globalTransform);
        void Reset();
    };

    class Tree : public IPrivateComponent{
    public:
        AssetRef m_treeDescriptor;

        TreeModel m_treeModel;

        void OnInspect() override;

        void OnDestroy() override;

        void OnCreate() override;
    };
}