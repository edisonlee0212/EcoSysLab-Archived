//
// Created by lllll on 9/16/2021.
//

#include "SorghumField.hpp"

#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
#include "SorghumStateGenerator.hpp"
#include "Scene.hpp"
#include "EditorLayer.hpp"
#include "TransformGraph.hpp"
#include "Soil.hpp"
#include "EcoSysLabLayer.hpp"
using namespace EcoSysLab;
void RectangularSorghumFieldPattern::GenerateField(
	std::vector<glm::mat4>& matricesList) const
{
	std::shared_ptr<Soil> soil;
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) soil = soilCandidate.lock();
	std::shared_ptr<SoilDescriptor> soilDescriptor;
	if (soil) {
		soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
	}
	std::shared_ptr<HeightField> heightField{};
	if (soilDescriptor)
	{
		heightField = soilDescriptor->m_heightField.Get<HeightField>();
	}
	matricesList.resize(m_gridSize.x * m_gridSize.y);
	const glm::vec2 startPoint = glm::vec2((m_gridSize.x - 1) * m_gridDistance.x, (m_gridSize.y - 1) * m_gridDistance.y) * 0.5f;
	for (int i = 0; i < m_gridSize.x; i++) {
		for (int j = 0; j < m_gridSize.y; j++) {
			glm::vec3 position = glm::vec3(-startPoint.x + i * m_gridDistance.x, 0.0f, -startPoint.y + j * m_gridDistance.y);
			position.x += glm::linearRand(-m_gridDistance.x * m_randomShiftMean.x, m_gridDistance.x * m_randomShiftMean.x);
			position.z += glm::linearRand(-m_gridDistance.y * m_randomShiftMean.y, m_gridDistance.y * m_randomShiftMean.y);
			position += glm::gaussRand(glm::vec3(0.0f), glm::vec3(m_distanceVariance.x, 0.0f, m_distanceVariance.y));
			if (heightField) position.y = heightField->GetValue({ position.x, position.z }) - 0.01f;
			Transform transform{};
			transform.SetPosition(position);
			auto rotation = glm::quat(glm::radians(
				glm::vec3(glm::gaussRand(glm::vec3(0.0f), m_rotationVariance))));
			transform.SetRotation(rotation);
			transform.SetScale(glm::vec3(1.f));
			matricesList[i * m_gridSize.y + j] = transform.m_value;
		}
	}
}

void SorghumField::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {

	ImGui::DragInt("Size limit", &m_sizeLimit, 1, 0, 10000);
	ImGui::DragFloat("Sorghum size", &m_sorghumSize, 0.01f, 0, 10);
	if (ImGui::Button("Instantiate")) {
		InstantiateField();
	}

	ImGui::Text("Matrices count: %d", (int)m_matrices.size());
}
void SorghumField::Serialize(YAML::Emitter& out) {
	out << YAML::Key << "m_sizeLimit" << YAML::Value << m_sizeLimit;
	out << YAML::Key << "m_sorghumSize" << YAML::Value << m_sorghumSize;

	out << YAML::Key << "m_matrices" << YAML::Value << YAML::BeginSeq;
	for (auto& i : m_matrices) {
		out << YAML::BeginMap;
		i.first.Save("SPD", out);
		out << YAML::Key << "Transform" << YAML::Value << i.second;
		out << YAML::EndMap;
	}
	out << YAML::EndSeq;
}
void SorghumField::Deserialize(const YAML::Node& in) {
	if (in["m_sizeLimit"])
		m_sizeLimit = in["m_sizeLimit"].as<int>();
	if (in["m_sorghumSize"])
		m_sorghumSize = in["m_sorghumSize"].as<float>();

	m_matrices.clear();
	if (in["m_matrices"]) {
		for (const auto& i : in["m_matrices"]) {
			AssetRef spd;
			spd.Load("SPD", i);
			m_matrices.emplace_back(spd, i["Transform"].as<glm::mat4>());
		}
	}
}
void SorghumField::CollectAssetRef(std::vector<AssetRef>& list) {
	for (auto& i : m_matrices) {
		list.push_back(i.first);
	}
}
Entity SorghumField::InstantiateField() const
{
	if (m_matrices.empty()) {
		EVOENGINE_ERROR("No matrices generated!");
		return {};
	}

	const auto sorghumLayer = Application::GetLayer<SorghumLayer>();
	const auto scene = sorghumLayer->GetScene();
	if (sorghumLayer) {
		const auto fieldAsset = std::dynamic_pointer_cast<SorghumField>(GetSelf());
		const auto field = scene->CreateEntity("Field");
		// Create sorghums here.
		int size = 0;
		for (auto& newSorghum : fieldAsset->m_matrices) {
			const auto sorghumDescriptor = newSorghum.first.Get<SorghumStateGenerator>();
			if (!sorghumDescriptor) continue;
			Entity sorghumEntity = sorghumDescriptor->CreateEntity(size);
			auto sorghumTransform = scene->GetDataComponent<Transform>(sorghumEntity);
			sorghumTransform.m_value = newSorghum.second;
			sorghumTransform.SetScale(glm::vec3(m_sorghumSize));
			scene->SetDataComponent(sorghumEntity, sorghumTransform);
			scene->SetParent(sorghumEntity, field);

			const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghumEntity).lock();
			sorghum->m_sorghumDescriptor = sorghumDescriptor;
			const auto sorghumState = ProjectManager::CreateTemporaryAsset<SorghumState>();
			sorghumDescriptor->Apply(sorghumState, glm::linearRand(0, INT_MAX));
			sorghum->m_sorghumState = sorghumState;
			size++;
			if (size >= m_sizeLimit)
				break;
		}

		
		TransformGraph::CalculateTransformGraphForDescendents(scene,
			field);
		return field;
	}
	else {
		EVOENGINE_ERROR("No sorghum layer!");
		return {};
	}
}

