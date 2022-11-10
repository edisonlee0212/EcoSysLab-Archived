#pragma once

#include "TreeModel.hpp"

using namespace UniEngine;
namespace EcoSysLab {
    struct RingSegment {
        glm::vec3 m_startPosition, m_endPosition;
        glm::vec3 m_startAxis, m_endAxis;
        float m_startRadius, m_endRadius;

        RingSegment() {}

        RingSegment(glm::vec3 startPosition, glm::vec3 endPosition,
                    glm::vec3 startAxis, glm::vec3 endAxis,
                    float startRadius, float endRadius);

        void AppendPoints(std::vector<Vertex> &vertices, glm::vec3 &normalDir,
                          int step);

        [[nodiscard]] glm::vec3 GetPoint(glm::vec3 &normalDir, float angle, bool isStart);
    };

    struct MeshGeneratorSettings {
        float m_resolution = 0.02f;
        float m_subdivision = 16.0f;
        bool m_vertexColorOnly = false;
        bool m_enableFoliage = true;
        bool m_enableBranch = true;

        bool m_overrideRadius = false;
        float m_radius = 0.01f;
        bool m_overrideVertexColor = false;
        bool m_markJunctions = true;
        float m_junctionLowerRatio = 0.4f;
        float m_junctionUpperRatio = 0.0f;
        glm::vec3 m_branchVertexColor = glm::vec3(1.0f);
        glm::vec3 m_foliageVertexColor = glm::vec3(1.0f);

        bool m_smoothness = true;
        float m_internodeLengthFactor = 1.0f;

        void OnInspect();

        void Save(const std::string &name, YAML::Emitter &out);

        void Load(const std::string &name, const YAML::Node &in);
    };

    class BranchMeshGenerator {
    public:
        static void Generate(TreeSkeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton, std::vector<Vertex> &vertices,
                             std::vector<unsigned int> &indices,
                             const MeshGeneratorSettings &settings);
    };
}