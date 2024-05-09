#pragma once
#include "Tree.hpp"
#include "SimulationSettings.hpp"
using namespace EvoEngine;
namespace EcoSysLab {

    class ForestPatch : public IAsset
    {
    public:
        glm::vec2 m_gridDistance = glm::vec2(1.5f);
        glm::vec2 m_positionOffsetMean = glm::vec2(0.f);
        glm::vec2 m_positionOffsetVariance = glm::vec2(0.0f);
        glm::vec3 m_rotationOffsetVariance = glm::vec3(0.0f);
        glm::ivec2 m_gridSize = glm::ivec2(4, 4);
        AssetRef m_treeDescriptor;
        TreeGrowthSettings m_treeGrowthSettings{};
        SimulationSettings m_simulationSettings;

        float m_minLowBranchPruning = 0.f;
        float m_maxLowBranchPruning = 0.f;

        float m_simulationTime = 0.f;
        float m_startTimeMax = 0.0f;
        Entity InstantiatePatch(bool setSimulationSettings = true);
        void CollectAssetRef(std::vector<AssetRef>& list) override;
        void Serialize(YAML::Emitter& out) override;
        void Deserialize(const YAML::Node& in) override;
        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
    };

    struct TreeInfo
    {
        GlobalTransform m_globalTransform{};
        AssetRef m_treeDescriptor{};
        void Serialize(YAML::Emitter& out) const;
        void Deserialize(const YAML::Node& in);
        void CollectAssetRef(std::vector<AssetRef>& list) const;
    };

    class ForestDescriptor : public IAsset{
    public:
        std::vector<TreeInfo> m_treeInfos;
        TreeGrowthSettings m_treeGrowthSettings;
        
        void ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor);
        void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors);
        void ApplyTreeDescriptors(const std::filesystem::path& folderPath);
        void ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors, const std::vector<float>& ratios);
        void ApplyTreeDescriptors(const std::filesystem::path& folderPath, const std::vector<float>& ratios);
        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void OnCreate() override;

        void CollectAssetRef(std::vector<AssetRef> &list) override;

        void Serialize(YAML::Emitter &out) override;

        void Deserialize(const YAML::Node &in) override;

        void SetupGrid(const glm::ivec2& gridSize, float gridDistance, float randomShift);

        void InstantiatePatch(bool setParent);
    };
}