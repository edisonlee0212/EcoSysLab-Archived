#pragma once

#include "PointCloudScannerUtils.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
    struct SorghumPointCloudPointSettings {
        float m_variance = 0.015f;
        float m_ballRandRadius = 0.005f;

        bool m_typeIndex = true;
        bool m_instanceIndex = true;
        bool m_leafIndex = true;

        float m_boundingBoxLimit = 1.f;

        void OnInspect();
        void Save(const std::string& name, YAML::Emitter& out) const;
        void Load(const std::string& name, const YAML::Node& in);
    };

    class SorghumPointCloudCaptureSettings : public PointCloudCaptureSettings
    {
    public:
        void OnInspect() override;
        void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) override;
        bool SampleFilter(const PointCloudSample& sample) override;
    };

	class SorghumPointCloudScanner : public IPrivateComponent
	{
	public:
        SorghumPointCloudPointSettings m_sorghumPointCloudPointSettings {};
        void Capture(const std::filesystem::path& savePath, const std::shared_ptr<PointCloudCaptureSettings>& captureSettings) const;
        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void OnDestroy() override;

        void Serialize(YAML::Emitter& out) override;
        void Deserialize(const YAML::Node& in) override;
	};
}