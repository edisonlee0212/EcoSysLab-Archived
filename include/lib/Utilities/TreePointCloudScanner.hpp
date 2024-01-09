#pragma once
#include "Tree.hpp"
#include "PointCloudScanner.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	class TreePointCloudScanner : public IPrivateComponent{
	public:
        PointCloudPointSettings m_pointSettings;
        void Capture(const std::filesystem::path& savePath, const std::shared_ptr<PointCloudCaptureSettings>& captureSettings) const;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void OnDestroy() override;

        void Serialize(YAML::Emitter& out) override;
        void Deserialize(const YAML::Node& in) override;
	};
}