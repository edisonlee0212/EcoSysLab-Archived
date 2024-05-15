#pragma once
#include <SorghumLayer.hpp>

using namespace EcoSysLab;

namespace EcoSysLab {
	class CBTFImporter : public IPrivateComponent {
	public:
		bool m_processing = false;
		std::filesystem::path m_currentExportFolder;
		std::vector<std::filesystem::path> m_importFolders;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Update() override;
	};
}