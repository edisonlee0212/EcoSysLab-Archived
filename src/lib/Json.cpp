#include "Json.hpp"

using namespace EcoSysLab;

bool Json::SaveInternal(const std::filesystem::path& path)
{
	return true;
}

bool Json::LoadInternal(const std::filesystem::path& path)
{
	const std::ifstream input(path, std::ios::in | std::ios::binary);
	if (!input.is_open()) {
		std::cout << "Error: could not open file " << path << std::endl;
		return false;
	}
	std::ostringstream ss;
	ss << input.rdbuf();
	m_content = ss.str();
	return true;
}

void Json::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	ImGui::Text(m_content.c_str());
}
