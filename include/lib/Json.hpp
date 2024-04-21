#pragma once

using namespace EvoEngine;
namespace EcoSysLab
{
	class Json : public IAsset
	{
	protected:
		bool SaveInternal(const std::filesystem::path& path) override;
		bool LoadInternal(const std::filesystem::path& path) override;

	public:
		std::string m_content;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}