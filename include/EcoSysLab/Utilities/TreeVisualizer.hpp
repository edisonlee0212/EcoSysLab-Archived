#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"

using namespace UniEngine;
namespace EcoSysLab {
    enum class PruningMode{

        Line
    };


    class TreeVisualizer {
        std::vector<glm::mat4> m_matrices;
        std::vector<glm::vec4> m_colors;
        bool m_visualization = true;
        bool m_treeHierarchyGui = true;
        InternodeHandle m_selectedInternodeHandle = -1;
        float m_selectedInternodeLengthFactor = 0.0f;
        std::vector<InternodeHandle> m_selectedInternodeHierarchyList;
        int m_version = -1;

        bool RayCastSelection(const TreeSkeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                              const GlobalTransform &globalTransform);

        void SetSelectedInternode(const TreeSkeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                                  InternodeHandle internodeHandle);

        bool DrawInternodeInspectionGui(TreeStructure <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
                                        InternodeHandle internodeHandle, bool &deleted,
                                        const unsigned &hierarchyLevel);

        void PeekInternodeInspectionGui(const TreeSkeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                                        InternodeHandle internodeHandle,
                                        const unsigned &hierarchyLevel);

        void
        InspectInternode(const TreeSkeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton, InternodeHandle internodeHandle);

    public:
        void
        SyncMatrices(const TreeSkeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton);

        int m_iteration = 0;

        bool
        OnInspect(TreeStructure <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
                  const GlobalTransform &globalTransform);

        void Reset(TreeStructure <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure);
    };


}