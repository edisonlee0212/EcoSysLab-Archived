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
    class PointCloudCaptureSettings
    {
    public:
        virtual bool OnInspect() = 0;
        virtual void Save(const std::string& name, YAML::Emitter& out) const {}
        virtual void Load(const std::string& name, const YAML::Node& in) {}
        virtual void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) = 0;
        virtual bool SampleFilter(const PointCloudSample& sample) { return true; }
    };
}