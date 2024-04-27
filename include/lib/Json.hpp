#pragma once
#include "json/json.h"
using namespace EvoEngine;
namespace EcoSysLab
{
	class Json : public IAsset
	{
	protected:
		bool SaveInternal(const std::filesystem::path& path) override;
		bool LoadInternal(const std::filesystem::path& path) override;

	public:
		treeio::json m_json;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}