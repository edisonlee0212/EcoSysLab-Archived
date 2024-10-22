//
// Created by lllll on 8/16/2021.
//

#include "ObjectRotator.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "Transform.hpp"
using namespace EcoSysLab;
void ObjectRotator::FixedUpdate() {
	auto scene = GetScene();
	auto transform = scene->GetDataComponent<Transform>(GetOwner());
	m_rotation.y += Times::FixedDeltaTime() * m_rotateSpeed;
	transform.SetEulerRotation(glm::radians(m_rotation));
	scene->SetDataComponent(GetOwner(), transform);
}

bool ObjectRotator::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	ImGui::DragFloat("Speed", &m_rotateSpeed);
	ImGui::DragFloat3("Rotation", &m_rotation.x);
	return false;
}

void ObjectRotator::Serialize(YAML::Emitter& out) const {
	out << YAML::Key << "m_rotateSpeed" << YAML::Value << m_rotateSpeed;
	out << YAML::Key << "m_rotation" << YAML::Value << m_rotation;
}

void ObjectRotator::Deserialize(const YAML::Node& in) {
	m_rotateSpeed = in["m_rotateSpeed"].as<float>();
	m_rotation = in["m_rotation"].as<glm::vec3>();
}
