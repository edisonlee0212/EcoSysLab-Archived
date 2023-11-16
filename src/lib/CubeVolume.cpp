#include "CubeVolume.hpp"

using namespace EcoSysLab;

void CubeVolume::ApplyMeshBounds(const std::shared_ptr<Mesh>& mesh) {
	if (!mesh) return;
	m_minMaxBound = mesh->GetBound();
}

void CubeVolume::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	IVolume::OnInspect(editorLayer);
	ImGui::DragFloat3("Min", &m_minMaxBound.m_min.x, 0.1);
	ImGui::DragFloat3("Max", &m_minMaxBound.m_max.x, 0.1);
	static PrivateComponentRef privateComponentRef{};

	if (editorLayer->DragAndDropButton<MeshRenderer>(privateComponentRef, "Target MeshRenderer"))
	{
		if (const auto mmr = privateComponentRef.Get<MeshRenderer>())
		{
			ApplyMeshBounds(mmr->m_mesh.Get<Mesh>());
			privateComponentRef.Clear();
		}
	}
}

bool CubeVolume::InVolume(const glm::vec3& position) {
	return m_minMaxBound.InBound(position);
}

glm::vec3 CubeVolume::GetRandomPoint() {
	return glm::linearRand(m_minMaxBound.m_min, m_minMaxBound.m_max);
}

bool CubeVolume::InVolume(const GlobalTransform& globalTransform,
	const glm::vec3& position) {
	const auto finalPos = glm::vec3(
		(glm::inverse(globalTransform.m_value) * glm::translate(position))[3]);
	return m_minMaxBound.InBound(finalPos);
}

void CubeVolume::Serialize(YAML::Emitter& out) {
	IVolume::Serialize(out);
	out << YAML::Key << "m_minMaxBound.m_min" << YAML::Value
		<< m_minMaxBound.m_min;
	out << YAML::Key << "m_minMaxBound.m_max" << YAML::Value
		<< m_minMaxBound.m_max;
}

void CubeVolume::Deserialize(const YAML::Node& in) {
	IVolume::Deserialize(in);
	m_minMaxBound.m_min = in["m_minMaxBound.m_min"].as<glm::vec3>();
	m_minMaxBound.m_max = in["m_minMaxBound.m_max"].as<glm::vec3>();
}

