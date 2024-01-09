#pragma once

namespace EvoEngine
{
    struct PointCloudSample;
    struct HitInfo;

#ifndef BUILD_WITH_RAYTRACER
    struct HitInfo {
        glm::vec3 m_position = glm::vec3(0.0f);
        glm::vec3 m_normal = glm::vec3(0.0f);
        glm::vec3 m_tangent = glm::vec3(0.0f);
        glm::vec4 m_color = glm::vec4(1.0f);
        glm::vec2 m_texCoord = glm::vec2(0.0f);
        glm::vec3 m_data = glm::vec4(0.0f);
        glm::vec2 m_data2 = glm::vec4(0.0f);
    };
    struct PointCloudSample {
        // Input
        glm::vec3 m_direction = glm::vec3(0.0f);
        glm::vec3 m_start = glm::vec3(0.0f);

        // Output
        uint64_t m_handle = 0;
        bool m_hit = false;

        HitInfo m_hitInfo;
    };
#endif

}

using namespace EvoEngine;
namespace EcoSysLab {
    struct PointCloudPointSettings {
        float m_variance = 0.015f;
        float m_ballRandRadius = 0.005f;
        bool m_typeIndex = true;
        bool m_instanceIndex = true;
        bool m_branchIndex = false;
        bool m_internodeIndex = false;
        bool m_lineIndex = true;
        bool m_treePartIndex = true;

        float m_boundingBoxLimit = 1.f;

        void OnInspect();

        void Serialize(const std::string& name, YAML::Emitter& out) const;

        void Deserialize(const std::string& name, const YAML::Node& in);
    };

    class PointCloudCaptureSettings
    {
    public:
        virtual void OnInspect() = 0;
        virtual void Serialize(const std::string& name, YAML::Emitter& out) const {};
        virtual void Deserialize(const std::string& name, const YAML::Node& in) {};
        virtual void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) = 0;
        virtual bool SampleFilter(const PointCloudSample& sample) { return true; };
    };

    class PointCloudCircularCaptureSettings : public PointCloudCaptureSettings {
    public:
        int m_pitchAngleStart = -20;
        int m_pitchAngleStep = 10;
        int m_pitchAngleEnd = 60;
        int m_turnAngleStart = 0;
        int m_turnAngleStep = 10;
        int m_turnAngleEnd = 360;
        float m_distance = 5.0f;
        float m_height = 1.5f;
        float m_fov = 60;
        glm::vec2 m_focusPoint = { 0, 0 };
        int m_resolution = 128;
        float m_cameraDepthMax = 10;

        void OnInspect() override;

        void Serialize(const std::string& name, YAML::Emitter& out) const override;

        void Deserialize(const std::string& name, const YAML::Node& in) override;

        GlobalTransform GetTransform(const glm::vec2& focusPoint, float turnAngle, float pitchAngle) const;
        void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) override;
    };

    class PointCloudGridCaptureSettings : public PointCloudCaptureSettings
    {
    public:
        float m_boundingBoxSize = 3.;

        glm::ivec2 m_gridSize = { 5, 5 };
        float m_gridDistance = 1.25f;
        float m_step = 0.01f;
        int m_backpackSample = 512;
        float m_backpackHeight = 1.0f;
        int m_droneSample = 128;
        float m_droneHeight = 5.0f;
        void OnInspect() override;
        void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) override;
        bool SampleFilter(const PointCloudSample& sample) override;
    };

    class PointCloudScanner : public IPrivateComponent {
    public:
        PointCloudPointSettings m_pointSettings;
        void Capture(const std::filesystem::path& savePath, const std::shared_ptr<PointCloudCaptureSettings>& captureSettings) const;
        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void OnDestroy() override;

        void Serialize(YAML::Emitter& out) override;
        void Deserialize(const YAML::Node& in) override;
    };
}