#pragma once

#include "ecosyslab_export.h"
#include "PlantStructure.hpp"
#include "IVolume.hpp"

using namespace UniEngine;
namespace EcoSysLab {
    struct RadialBoundingVolumeSlice {
        float m_maxDistance;
    };

    class RadialBoundingVolume : public IVolume {
        std::vector<std::shared_ptr<Mesh>> m_boundMeshes;
        bool m_meshGenerated = false;

        void CalculateSizes();
    public:
        glm::vec3 m_center;
        glm::vec4 m_displayColor = glm::vec4(0.0f, 0.0f, 1.0f, 0.5f);
        float m_offset = 0.1f;
        [[nodiscard]] glm::vec3 GetRandomPoint() override;

        [[nodiscard]] glm::ivec2 SelectSlice(glm::vec3 position) const;

        float m_maxHeight = 0.0f;
        float m_maxRadius = 0.0f;

        void GenerateMesh();

        void FormEntity();

        std::string Save();

        void ExportAsObj(const std::string& filename);

        void Load(const std::string& path);

        float m_displayScale = 0.2f;
        int m_layerAmount = 8;
        int m_sectorAmount = 8;
        std::vector<std::vector<RadialBoundingVolumeSlice>> m_layers;
        std::vector<std::pair<float, std::vector<float>>> m_sizes;
        float m_totalSize = 0;
        void CalculateVolume(const std::vector<glm::vec3>& points);
        template<typename SkeletonData, typename FlowData, typename NodeData>
        void CalculateVolume(const Skeleton <SkeletonData, FlowData, NodeData>& skeleton);
        void OnInspect() override;

        void ResizeVolumes();

        bool InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) override;

        bool InVolume(const glm::vec3& position) override;

        void Serialize(YAML::Emitter& out) override;

        void Deserialize(const YAML::Node& in) override;

        void Augmentation(float value);
    };

    template <typename SkeletonData, typename FlowData, typename NodeData>
    void RadialBoundingVolume::CalculateVolume(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton)
    {
        ResizeVolumes();
        m_maxHeight = 0;
        m_maxRadius = 0;
        for(const auto& node : skeleton.RefRawNodes())
        {
            const auto& info = node.m_info;
            const auto& point1 = info.m_globalPosition;
            {
                if (point1.y > m_maxHeight)
                    m_maxHeight = point1.y;
                const float radius = glm::length(glm::vec2(point1.x, point1.z));
                if (radius > m_maxRadius)
                    m_maxRadius = radius;
            }
            {
                auto point2 = point1 + info.m_length + info.m_globalRotation * glm::vec3(0, 0, -1);
                if (point2.y > m_maxHeight)
                    m_maxHeight = point2.y;
                const float radius = glm::length(glm::vec2(point2.x, point2.z));
                if (radius > m_maxRadius)
                    m_maxRadius = radius;
            }
        }

        for (const auto& node : skeleton.RefRawNodes()) {
            const auto& info = node.m_info;
            const auto& point1 = info.m_globalPosition;
            {
                const auto sliceIndex = SelectSlice(point1);
                const float currentDistance =
                    glm::length(glm::vec2(point1.x, point1.z));
                if (currentDistance <= m_offset) {
                    for (auto& slice : m_layers[sliceIndex.x]) {
                        if (slice.m_maxDistance <
                            currentDistance + m_offset)
                            slice.m_maxDistance = currentDistance + m_offset;
                    }
                }
                else if (m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance <
                    currentDistance)
                    m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance = currentDistance + m_offset;
            }
            {
	            auto point2 = point1 + info.m_length + info.m_globalRotation * glm::vec3(0, 0, -1);
	            const auto sliceIndex = SelectSlice(point2);
                const float currentDistance =
                    glm::length(glm::vec2(point2.x, point2.z));
                if (currentDistance <= m_offset) {
                    for (auto& slice : m_layers[sliceIndex.x]) {
                        if (slice.m_maxDistance <
                            currentDistance + m_offset)
                            slice.m_maxDistance = currentDistance + m_offset;
                    }
                }
                else if (m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance <
                    currentDistance)
                    m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance = currentDistance + m_offset;
            }
        }
        GenerateMesh();
        CalculateSizes();
    }
} // namespace EcoSysLab
