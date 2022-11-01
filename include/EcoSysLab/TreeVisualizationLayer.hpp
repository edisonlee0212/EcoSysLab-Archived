#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
using namespace UniEngine;
namespace EcoSysLab {
    class TreeVisualizationLayer : public ILayer{
        bool m_displayInternodes = true;
        bool m_displayBoundingBox = true;
        bool m_visualization = false;
        std::vector<int> m_versions;
        std::vector<glm::vec4> m_randomColors;
        std::vector<glm::mat4> m_matrices;
        std::vector<glm::vec4> m_colors;
        std::vector<glm::mat4> m_boundingBoxMatrices;
        std::vector<glm::vec4> m_boundingBoxColors;

        glm::ivec2 m_gridSize = {32, 32};
        glm::vec2 m_gridDistance = {10, 10};
        float m_lastUsedTime = 0.0f;
        float m_totalTime = 0.0f;
        int m_internodeSize = 0;
        int m_iteration = 0;
        bool m_needUpdate = false;
        bool m_enableHistory = true;
    public:
        void OnCreate() override;

        void OnDestroy() override;

        void LateUpdate() override;

        void OnInspect() override;
    };
}