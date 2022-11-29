#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"

using namespace UniEngine;
namespace EcoSysLab {
    enum class PruningMode {
        None,
        Stroke
    };


    class TreeVisualizer {
        std::vector<glm::mat4> m_internodeMatrices;
        std::vector<glm::vec4> m_internodeColors;
        std::vector<glm::mat4> m_rootNodeMatrices;
        std::vector<glm::vec4> m_rootNodeColors;

        std::vector<glm::vec2> m_storedMousePositions;
        bool m_visualization = true;
        bool m_treeHierarchyGui = true;

        NodeHandle m_selectedInternodeHandle = -1;
        float m_selectedInternodeLengthFactor = 0.0f;
        std::vector<NodeHandle> m_selectedInternodeHierarchyList;

        NodeHandle m_selectedRootNodeHandle = -1;
        float m_selectedRootNodeLengthFactor = 0.0f;
        std::vector<NodeHandle> m_selectedRootNodeHierarchyList;

        PruningMode m_mode = PruningMode::None;

        bool
        RayCastSelection(const Skeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                         const GlobalTransform &globalTransform);

        bool ScreenCurvePruning(Skeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                                const GlobalTransform &globalTransform);

        void SetSelectedInternode(
                const Skeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                NodeHandle internodeHandle);

        bool DrawInternodeInspectionGui(
                PlantStructure <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
                NodeHandle internodeHandle, bool &deleted,
                const unsigned &hierarchyLevel);

        void PeekInternodeInspectionGui(
                const Skeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                NodeHandle internodeHandle,
                const unsigned &hierarchyLevel);

        void
        PeekInternode(const Skeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                         NodeHandle internodeHandle);
        bool
        InspectInternode(Skeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
                      NodeHandle internodeHandle);

    public:

        void
        SyncMatrices(const Skeleton <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton);

        int m_iteration = 0;
        bool m_needUpdate = false;

        bool
        OnInspect(PlantStructure <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
                  const GlobalTransform &globalTransform);

        void Reset(PlantStructure <SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure);
    };


}