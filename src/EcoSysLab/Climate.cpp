#include "Climate.hpp"

using namespace EcoSysLab;

void ClimateDescriptor::OnInspect()
{
	if (ImGui::Button("Instantiate")) {
		auto scene = Application::GetActiveScene();
		auto climateEntity = scene->CreateEntity(GetTitle());
		auto climate = scene->GetOrSetPrivateComponent<Climate>(climateEntity).lock();
		climate->m_climateDescriptor = ProjectManager::GetAsset(GetHandle());
	}
}

void ClimateDescriptor::Serialize(YAML::Emitter& out)
{
	
}

void ClimateDescriptor::Deserialize(const YAML::Node& in)
{
	
}

void Climate::OnInspect()
{
	if (m_climateDescriptor.Get<ClimateDescriptor>())
	{

	}
}
