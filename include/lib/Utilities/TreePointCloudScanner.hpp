#pragma once
#include "Tree.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
    struct PointCloudPointSettings {
        float m_variance = 0.0f;
        float m_ballRandRadius = 0.0f;
        bool m_typeIndex = true;
        bool m_instanceIndex = false;
        bool m_branchIndex = true;
        bool m_internodeIndex = false;
        float m_boundingBoxLimit = 1.f;

        void OnInspect();

        void Serialize(const std::string& name, YAML::Emitter& out) const;

        void Deserialize(const std::string& name, const YAML::Node& in);
    };

    struct PointCloudCaptureSettings {
        int m_pitchAngleStart = -20;
        int m_pitchAngleStep = 20;
        int m_pitchAngleEnd = 60;
        int m_turnAngleStart = 0;
        int m_turnAngleStep = 10;
        int m_turnAngleEnd = 360;
        float m_distance = 5.0f;
        float m_height = 1.5f;
        float m_fov = 60;
        int m_resolution = 128;
        float m_cameraDepthMax = 10;

        void OnInspect();

        void Serialize(const std::string& name, YAML::Emitter& out) const;

        void Deserialize(const std::string& name, const YAML::Node& in);

        GlobalTransform GetTransform(const Bound& bound, float turnAngle, float pitchAngle) const;
    };
	class TreePointCloudScanner : public IPrivateComponent{
	public:
        PointCloudPointSettings m_pointSettings;
        PointCloudCaptureSettings m_captureSettings;

        PrivateComponentRef m_tree;
        PrivateComponentRef m_soil;
        void GeneratePointCloud(const std::filesystem::path& savePath);
        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void OnDestroy() override;

        void Serialize(YAML::Emitter& out) override;
        void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
        void Deserialize(const YAML::Node& in) override;
	};
}