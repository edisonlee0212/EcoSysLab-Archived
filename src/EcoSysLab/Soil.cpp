#include "Soil.hpp"

#include <cassert>

#include "EditorLayer.hpp"
#include "Graphics.hpp"
#include "HeightField.hpp"
using namespace EcoSysLab;
bool OnInspectSoilParameters(SoilParameters& soilParameters)
{
	bool changed = false;
	if (ImGui::TreeNodeEx("Soil Parameters", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::DragFloat("Diffusion Force", &soilParameters.m_diffusionForce, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		if (ImGui::DragFloat3("Gravity Factor", &soilParameters.m_gravityForce.x, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		if (ImGui::DragFloat("Delta X", &soilParameters.m_deltaX, 0.01f, 0.01f, 1.0f))
		{
			changed = true;
		}
		if (ImGui::DragFloat("Delta time", &soilParameters.m_deltaTime, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		ImGui::TreePop();
	}
	return changed;
}
void SoilDescriptor::OnInspect()
{
	bool changed = false;
	if (Editor::DragAndDropButton<HeightField>(m_heightField, "Height Field", true))
	{
		changed = true;
	}

	glm::ivec3 resolution = m_voxelResolution;
	if (ImGui::DragInt3("Voxel Resolution", &resolution.x, 1, 1, 100))
	{
		m_voxelResolution = resolution;
		changed = true;
	}
	if (ImGui::DragFloat3("Voxel Bounding box min", &m_boundingBoxMin.x, 0.01f))
	{
		changed = true;
	}

	if (ImGui::Button("Instantiate")) {
		auto scene = Application::GetActiveScene();
		auto soilEntity = scene->CreateEntity(GetTitle());
		auto soil = scene->GetOrSetPrivateComponent<Soil>(soilEntity).lock();
		soil->m_soilDescriptor = ProjectManager::GetAsset(GetHandle());
		soil->InitializeSoilModel();
	}


	if (OnInspectSoilParameters(m_soilParameters))
	{
		changed = true;
	}

	if (changed) m_saved = false;
}



void Soil::OnInspect()
{
	if (Editor::DragAndDropButton<SoilDescriptor>(m_soilDescriptor, "SoilDescriptor", true)) {
		InitializeSoilModel();
	}
	auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (soilDescriptor)
	{
		if (ImGui::Button("Generate surface mesh")) {
			GenerateMesh();
		}
		//auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
		//if (!m_soilModel.m_initialized) m_soilModel.Initialize(soilDescriptor->m_soilParameters);
		assert(m_soilModel.m_initialized);
		static bool autoStep = false;
		if (ImGui::Button("Initialize"))
		{
			InitializeSoilModel();
		}
		

		ImGui::Checkbox("Auto step", &autoStep);
		if (autoStep)
		{
			m_soilModel.Step();
		}
		else if (ImGui::Button("Step"))
		{
			m_soilModel.Step();
		}
	}
}

void Soil::Serialize(YAML::Emitter& out)
{
	m_soilDescriptor.Save("m_soilDescriptor", out);
}

void Soil::Deserialize(const YAML::Node& in)
{
	m_soilDescriptor.Load("m_soilDescriptor", in);
	InitializeSoilModel();
}

void Soil::CollectAssetRef(std::vector<AssetRef>& list)
{
	list.push_back(m_soilDescriptor);
}

void Soil::GenerateMesh()
{
	const auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (!soilDescriptor)
	{
		UNIENGINE_ERROR("No soil descriptor!");
		return;
	}
	const auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
	if (!heightField)
	{
		UNIENGINE_ERROR("No height field!");
		return;
	}
	std::vector<Vertex> vertices;
	std::vector<glm::uvec3> triangles;
	heightField->GenerateMesh(glm::vec2(soilDescriptor->m_boundingBoxMin.x, soilDescriptor->m_boundingBoxMin.z),
		glm::uvec2(soilDescriptor->m_voxelResolution.x, soilDescriptor->m_voxelResolution.z), soilDescriptor->m_soilParameters.m_deltaX, vertices, triangles);

	const auto scene = Application::GetActiveScene();
	const auto self = GetOwner();
	Entity groundSurfaceEntity;
	const auto children = scene->GetChildren(self);

	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Ground surface") {
			groundSurfaceEntity = child;
			break;
		}
	}
	if (groundSurfaceEntity.GetIndex() == 0)
	{
		groundSurfaceEntity = scene->CreateEntity("Ground surface");
		scene->SetParent(groundSurfaceEntity, self);
	}

	const auto meshRenderer =
		scene->GetOrSetPrivateComponent<MeshRenderer>(groundSurfaceEntity).lock();
	const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	const auto material = ProjectManager::CreateTemporaryAsset<Material>();
	mesh->SetVertices(17, vertices, triangles);
	meshRenderer->m_mesh = mesh;
	meshRenderer->m_material = material;
}

void Soil::InitializeSoilModel()
{
	auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (soilDescriptor)
	{
		auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
		if (heightField)
		{
			m_soilModel.Initialize(soilDescriptor->m_soilParameters, soilDescriptor->m_voxelResolution, soilDescriptor->m_boundingBoxMin, [&](const glm::vec3& position)
				{
					auto height = heightField->GetValue(glm::vec2(position.x, position.z));
			return position.y > height - 1 ? glm::clamp(height - soilDescriptor->m_soilParameters.m_deltaX / 2.0f - position.y, 0.0f, 1.0f) : 1;
				});
		}
		else {
			m_soilModel.Initialize(soilDescriptor->m_soilParameters, soilDescriptor->m_voxelResolution, soilDescriptor->m_boundingBoxMin, [](const glm::vec3& position)
				{
					return position.y > 0 ? 0 : 1;
				});
		}
	}
}

void SerializeSoilParameters(const std::string& name, const SoilParameters& soilParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_deltaTime" << YAML::Value << soilParameters.m_deltaTime;
	out << YAML::Key << "m_diffusionForce" << YAML::Value << soilParameters.m_diffusionForce;
	out << YAML::Key << "m_gravityForce" << YAML::Value << soilParameters.m_gravityForce;
	out << YAML::Key << "m_deltaX" << YAML::Value << soilParameters.m_deltaX;
	out << YAML::EndMap;
}

void DeserializeSoilParameters(const std::string& name, SoilParameters& soilParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_deltaTime"]) soilParameters.m_deltaTime = param["m_deltaTime"].as<float>();
		if (param["m_diffusionForce"]) soilParameters.m_diffusionForce = param["m_diffusionForce"].as<float>();
		if (param["m_deltaX"]) soilParameters.m_deltaX = param["m_deltaX"].as<float>();
		if (param["m_gravityForce"]) soilParameters.m_gravityForce = param["m_gravityForce"].as<glm::vec3>();
	}
}

void SoilDescriptor::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_voxelResolution" << YAML::Value << m_voxelResolution;
	out << YAML::Key << "m_boundingBoxMin" << YAML::Value << m_boundingBoxMin;
	m_heightField.Save("m_heightField", out);
	SerializeSoilParameters("m_soilParameters", m_soilParameters, out);
}

void SoilDescriptor::Deserialize(const YAML::Node& in)
{
	if (in["m_voxelResolution"]) m_voxelResolution = in["m_voxelResolution"].as<glm::uvec3>();
	if (in["m_boundingBoxMin"]) m_boundingBoxMin = in["m_boundingBoxMin"].as<glm::vec3>();
	m_heightField.Load("m_heightField", in);
	DeserializeSoilParameters("m_soilParameters", m_soilParameters, in);
}

void SoilDescriptor::CollectAssetRef(std::vector<AssetRef>& list)
{
	list.push_back(m_heightField);
}

