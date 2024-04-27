#include "Json.hpp"

using namespace EcoSysLab;

bool Json::SaveInternal(const std::filesystem::path& path)
{
	std::ofstream o("pretty.json");
	o << std::setw(4) << m_json << std::endl;
	return true;
}

bool Json::LoadInternal(const std::filesystem::path& path)
{
	std::ifstream ifs(path);
	m_json = treeio::json::parse(ifs);
	return true;
}

void Json::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
}
