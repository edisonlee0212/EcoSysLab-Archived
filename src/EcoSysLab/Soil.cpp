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
		if (ImGui::InputInt3("Voxel Resolution", (int*)&soilParameters.m_voxelResolution))
		{
			changed = true;
		}
		if (ImGui::DragFloat("Delta X", &soilParameters.m_deltaX, 0.01f, 0.01f, 1.0f))
		{
			changed = true;
		}
		if (ImGui::DragFloat("Delta time", &soilParameters.m_deltaTime, 0.01f, 0.0f, 10.0f))
		{
			changed = true;
		}
		if (ImGui::InputFloat3("Bounding Box Min", (float*)&soilParameters.m_boundingBoxMin))
		{
			changed = true;
		}
		// TODO: boundaries
		if (ImGui::DragFloat("Diffusion Force", &soilParameters.m_diffusionForce, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		if (ImGui::DragFloat3("Gravity Force", &soilParameters.m_gravityForce.x, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		ImGui::TreePop();
	}
	return changed;
}

void SoilLayerDescriptor::OnInspect()
{
	bool changed = false;

	static unsigned soilTypePreset = 0;
	ImGui::Combo({ "Select soil type preset" }, { "Clay", "Silty Clay", "Loam", "Sand", "Loamy Sand" }, soilTypePreset);

	if(ImGui::Button("Apply soil type preset"))
	{
		switch (static_cast<SoilMaterialType>(soilTypePreset))
		{
		case SoilMaterialType::Clay:
			m_sandRatio = 0.1f;
			m_siltRatio = 0.1f;
			m_clayRatio = 0.8f;
			m_compactness = 1.f;
			break;
		case SoilMaterialType::SiltyClay:
			m_sandRatio = 0.1f;
			m_siltRatio = 0.4f;
			m_clayRatio = 0.5f;
			m_compactness = 1.f;
			break;
		case SoilMaterialType::Loam:
			m_sandRatio = 0.4f;
			m_siltRatio = 0.4f;
			m_clayRatio = 0.2f;
			m_compactness = 1.f;
			break;
		case SoilMaterialType::Sand:
			m_sandRatio = 1.f;
			m_siltRatio = 0.f;
			m_clayRatio = 0.f;
			m_compactness = 1.f;
			break;
		case SoilMaterialType::LoamySand:
			m_sandRatio = 0.8f;
			m_siltRatio = 0.1f;
			m_clayRatio = 0.1f;
			m_compactness = 1.f;
			break;
		}
		changed = true;
	}

	changed = ImGui::DragFloat("Sand ratio", &m_sandRatio, 0.01f, 0.0f, 1.0f) || changed;
	changed = ImGui::DragFloat("Silt ratio", &m_siltRatio, 0.01f, 0.0f, 1.0f) || changed;
	changed = ImGui::DragFloat("Clay ratio", &m_clayRatio, 0.01f, 0.0f, 1.0f) || changed;
	changed = ImGui::DragFloat("Compactness", &m_compactness, 0.01f, 0.0f, 1.0f) || changed;

	if (ImGui::TreeNode("Thickness")) {
		changed = m_thickness.OnInspect() || changed;
		ImGui::TreePop();
	}
	if (changed) m_saved = false;
}

void SoilLayerDescriptor::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_sandRatio" << YAML::Value << m_sandRatio;
	out << YAML::Key << "m_siltRatio" << YAML::Value << m_siltRatio;
	out << YAML::Key << "m_clayRatio" << YAML::Value << m_clayRatio;
	out << YAML::Key << "m_compactness" << YAML::Value << m_compactness;

	m_thickness.Save("m_thickness", out);
}

void SoilLayerDescriptor::Deserialize(const YAML::Node& in)
{
	if (in["m_sandRatio"]) m_sandRatio = in["m_sandRatio"].as<float>();
	if (in["m_siltRatio"]) m_siltRatio = in["m_siltRatio"].as<float>();
	if (in["m_clayRatio"]) m_clayRatio = in["m_clayRatio"].as<float>();
	if (in["m_compactness"]) m_compactness = in["m_compactness"].as<float>();
	m_thickness.Load("m_thickness", in);
}

void SoilDescriptor::OnInspect()
{
	bool changed = false;
	if (Editor::DragAndDropButton<HeightField>(m_heightField, "Height Field", true))
	{
		changed = true;
	}

	/*
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
	*/

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
	AssetRef tempSoilLayerDescriptorHolder;
	if (Editor::DragAndDropButton<SoilLayerDescriptor>(tempSoilLayerDescriptorHolder, "Drop new SoilLayerDescriptor here...")) {
		auto sld = tempSoilLayerDescriptorHolder.Get<SoilLayerDescriptor>();
		if (sld) {
			m_soilLayerDescriptors.emplace_back(sld);
			changed = true;
		}
		tempSoilLayerDescriptorHolder.Clear();
	}
	for (int i = 0; i < m_soilLayerDescriptors.size(); i++)
	{
		if (auto soilLayerDescriptor = m_soilLayerDescriptors[i].Get<SoilLayerDescriptor>())
		{
			if (ImGui::TreeNodeEx(("No." + std::to_string(i + 1)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
			{
				ImGui::Text(("Name: " + soilLayerDescriptor->GetTitle()).c_str());
				
				if (ImGui::Button("Remove"))
				{
					m_soilLayerDescriptors.erase(m_soilLayerDescriptors.begin() + i);
					changed = true;
					ImGui::TreePop();
					continue;
				}
				if(!soilLayerDescriptor->Saved())
				{
					ImGui::SameLine();
					if (ImGui::Button("Save"))
					{
						soilLayerDescriptor->Save();
					}
				}
				if (i < m_soilLayerDescriptors.size() - 1) {
					ImGui::SameLine();
					if (ImGui::Button("Move down"))
					{
						changed = true;
						const auto temp = m_soilLayerDescriptors[i];
						m_soilLayerDescriptors[i] = m_soilLayerDescriptors[i + 1];
						m_soilLayerDescriptors[i + 1] = temp;
					}
				}
				if (i > 0) {
					ImGui::SameLine();
					if (ImGui::Button("Move up"))
					{
						changed = true;
						const auto temp = m_soilLayerDescriptors[i - 1];
						m_soilLayerDescriptors[i - 1] = m_soilLayerDescriptors[i];
						m_soilLayerDescriptors[i] = temp;
					}
				}
				if (ImGui::TreeNode("Settings")) {
					soilLayerDescriptor->OnInspect();
					ImGui::TreePop();
				}
				ImGui::TreePop();
			}
		}
		else
		{
			m_soilLayerDescriptors.erase(m_soilLayerDescriptors.begin() + i);
			i--;
		}
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
		if (ImGui::Button("Initialize"))
		{
			InitializeSoilModel();
		}
		if (ImGui::Button("Reset"))
		{
			m_soilModel.Reset();
		}

		if (ImGui::Button("Split root test"))
		{
			SplitRootTestSetup();
		}

		ImGui::InputFloat("Diffusion Force", &m_soilModel.m_diffusionForce);
		ImGui::InputFloat3("Gravity Force", &m_soilModel.m_gravityForce.x);

		ImGui::Checkbox("Auto step", &m_autoStep);
		if (ImGui::Button("Step") || m_autoStep)
		{
			if (m_irrigation)
				m_soilModel.Irrigation();
			m_soilModel.Step();
		}
		ImGui::SliderFloat("Irrigation amount", &m_soilModel.m_irrigationAmount, 0.01, 100, "%.2f", ImGuiSliderFlags_Logarithmic);
		ImGui::Checkbox("apply Irrigation", &m_irrigation);

		ImGui::InputFloat3("Source position", (float*)&m_sourcePositon);
		ImGui::SliderFloat("Source amount", &m_sourceAmount, 1, 10000, "%.4f", ImGuiSliderFlags_Logarithmic);
		ImGui::InputFloat("Source width", &m_sourceWidth, 0.1, 100, "%.4f", ImGuiSliderFlags_Logarithmic);
		if (ImGui::Button("Apply Source"))
		{
			m_soilModel.ChangeWater(m_sourcePositon, m_sourceAmount, m_sourceWidth);
		}


		// Show some general properties:
		auto bbmin = m_soilModel.GetBoundingBoxMin();
		auto bbmax = m_soilModel.GetBoundingBoxMax();
		ImGui::InputFloat3("BB Min", &bbmin.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat3("BB Max", &bbmax.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Total Water", &m_soilModel.m_w_sum, 0.0f, 0.0f, "%.6f", ImGuiInputTextFlags_ReadOnly);
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
	heightField->GenerateMesh(glm::vec2(soilDescriptor->m_soilParameters.m_boundingBoxMin.x, soilDescriptor->m_soilParameters.m_boundingBoxMin.z),
		glm::uvec2(soilDescriptor->m_soilParameters.m_voxelResolution.x, soilDescriptor->m_soilParameters.m_voxelResolution.z), soilDescriptor->m_soilParameters.m_deltaX, vertices, triangles);

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

void SetSoilPhysicalMaterial(SoilPhysicalMaterial& target, float sandRatio, float siltRatio, float clayRatio, float compactness)
{
	assert(compactness <= 1 && compactness >= 0);

	float weight = sandRatio + siltRatio + clayRatio;
	sandRatio /= weight;
	siltRatio /= weight;
	clayRatio /= weight;
	float  airRatio = 1.f - compactness;

	//                         c    p     d
	static SoilPhysicalMaterial sand({
		[&](const glm::vec3& pos) { return 0.9f; },
		[&](const glm::vec3& pos) { return 15.0f; },
		[&](const glm::vec3& pos) { return 0.5f; },
		[&](const glm::vec3& pos) { return 0.0f; },
		[&](const glm::vec3& pos) { return 0.0f; } });

	static SoilPhysicalMaterial clay({
		[&](const glm::vec3& pos) { return 2.1f; },
		[&](const glm::vec3& pos) { return 0.05f; },
		[&](const glm::vec3& pos) { return 1.0f; },
		[&](const glm::vec3& pos) { return 0.0f; },
		[&](const glm::vec3& pos) { return 0.0f; } });

	static SoilPhysicalMaterial silt({
		[&](const glm::vec3& pos) { return 1.9f; },
		[&](const glm::vec3& pos) { return 1.5f; },
		[&](const glm::vec3& pos) { return 0.8f; },
		[&](const glm::vec3& pos) { return 0.0f; },
		[&](const glm::vec3& pos) { return 0.0f; } });

	static SoilPhysicalMaterial air({
		[&](const glm::vec3& pos) { return 5.0f; },
		[&](const glm::vec3& pos) { return 30.0f; },
		[&](const glm::vec3& pos) { return 1.0f; },
		[&](const glm::vec3& pos) { return 0.0f; },
		[&](const glm::vec3& pos) { return 0.0f; } });

	target.m_c = [&](const glm::vec3& pos) { return sandRatio * sand.m_c(pos) + siltRatio * silt.m_c(pos) + clayRatio * clay.m_c(pos) + airRatio * air.m_c(pos); };
	target.m_p = [&](const glm::vec3& pos) { return sandRatio * sand.m_p(pos) + siltRatio * silt.m_p(pos) + clayRatio * clay.m_p(pos) + airRatio * air.m_p(pos); };
	target.m_d = [&](const glm::vec3& pos) { return sandRatio * sand.m_d(pos) + siltRatio * silt.m_d(pos) + clayRatio * clay.m_d(pos) + airRatio * air.m_d(pos); };

}
void Soil::InitializeSoilModel()
{
	auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (soilDescriptor)
	{
		auto heightField = soilDescriptor->m_heightField.Get<HeightField>();

		auto params = soilDescriptor->m_soilParameters;
		params.m_boundary_x = SoilModel::Boundary::wrap;
		params.m_boundary_y = SoilModel::Boundary::absorb;
		params.m_boundary_z = SoilModel::Boundary::wrap;

		SoilSurface soilSurface;
		std::vector<SoilLayer> soilLayers;


		if (heightField)
		{
			soilSurface.m_height = [heightField](const glm::vec2& position)
			{
				return heightField->GetValue(glm::vec2(position.x, position.y));
			};
		}
		else {

			soilSurface.m_height = [&](const glm::vec2& position)
			{
				return 0.0f;
			};

		}
		//Add top air layer
		soilLayers.emplace_back();
		soilLayers.back().m_mat = SoilPhysicalMaterial({
					[&](const glm::vec3& pos) { return 1.0f; },
					[&](const glm::vec3& pos) { return 0.0f; },
					[&](const glm::vec3& pos) { return 0.0f; },
					[&](const glm::vec3& pos) { return 0.0f; },
					[&](const glm::vec3& pos) { return 0.0f; } });
		soilLayers.back().m_thickness = [](const glm::vec2& position) {return 0.f; };

		//Add user defined layers
		auto& soilLayerDescriptors = soilDescriptor->m_soilLayerDescriptors;
		for (int i = 0; i < soilDescriptor->m_soilLayerDescriptors.size(); i++)
		{
			if (auto soilLayerDescriptor = soilLayerDescriptors[i].Get<SoilLayerDescriptor>())
			{
				soilLayers.emplace_back();
				auto& soilLayer = soilLayers.back();
				SetSoilPhysicalMaterial(soilLayer.m_mat, soilLayerDescriptor->m_sandRatio, soilLayerDescriptor->m_siltRatio, soilLayerDescriptor->m_clayRatio, soilLayerDescriptor->m_compactness);
				soilLayer.m_thickness = [soilLayerDescriptor](const glm::vec2& position)
				{
					return soilLayerDescriptor->m_thickness.GetValue(position);
				};
				soilLayer.m_mat.m_d = [&, soilSurface](const glm::vec2& position)
				{
					const auto height = soilSurface.m_height(position);
					return position.y > height - 1 ? glm::clamp(height - params.m_deltaX / 2.0f - position.y, 0.0f, 1.0f) : 1.0f + height - position.y;;
				};
			}
			else
			{
				soilLayerDescriptors.erase(soilLayerDescriptors.begin() + i);
				i--;
			}
		}

		//Add bottom layer
		soilLayers.emplace_back();
		SetSoilPhysicalMaterial(soilLayers.back().m_mat, 0.4, 0.4, 0.2, 1);
		soilLayers.back().m_thickness = [](const glm::vec2& position) {return 1000.f; };

		m_soilModel.Initialize(params, soilSurface, soilLayers);
	}
}

void Soil::SplitRootTestSetup()
{
	//InitializeSoilModel();
	auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (soilDescriptor) {
		auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
		for (int i = 0; i < m_soilModel.m_n.size(); i++)
		{
			auto position = m_soilModel.GetPositionFromCoordinate(m_soilModel.GetCoordinateFromIndex(i));
			bool underGround = true;
			if (heightField)
			{
				auto height = heightField->GetValue(glm::vec2(position.x, position.z));
				if (position.y >= height) underGround = false;
			}
			if (underGround) {
				if (position.x > 0)
				{
					m_soilModel.m_n[i] = 2.0f;
				}
				else
				{
					m_soilModel.m_n[i] = 0.5f;
				}
			}
			else
			{
				m_soilModel.m_n[i] = 0.01f;
			}
		}
	}
}

void SerializeSoilParameters(const std::string& name, const SoilParameters& soilParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_voxelResolution" << YAML::Value << soilParameters.m_voxelResolution;
	out << YAML::Key << "m_deltaX" << YAML::Value << soilParameters.m_deltaX;
	out << YAML::Key << "m_deltaTime" << YAML::Value << soilParameters.m_deltaTime;
	out << YAML::Key << "m_boundingBoxMin" << YAML::Value << soilParameters.m_boundingBoxMin;

	out << YAML::Key << "m_boundary_x" << YAML::Value << static_cast<int>(soilParameters.m_boundary_x);
	out << YAML::Key << "m_boundary_y" << YAML::Value << static_cast<int>(soilParameters.m_boundary_y);
	out << YAML::Key << "m_boundary_z" << YAML::Value << static_cast<int>(soilParameters.m_boundary_z);

	out << YAML::Key << "m_diffusionForce" << YAML::Value << soilParameters.m_diffusionForce;
	out << YAML::Key << "m_gravityForce" << YAML::Value << soilParameters.m_gravityForce;
	out << YAML::EndMap;
}

void DeserializeSoilParameters(const std::string& name, SoilParameters& soilParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_voxelResolution"]) soilParameters.m_voxelResolution = param["m_voxelResolution"].as<glm::uvec3>();
		else {
			UNIENGINE_WARNING("DeserializeSoilParameters: m_voxelResolution not found!");
			//UNIENGINE_ERROR("DeserializeSoilParameters: m_voxelResolution not found!");
			//UNIENGINE_LOG("DeserializeSoilParameters: m_voxelResolution not found!");
		}
		if (param["m_deltaX"]) soilParameters.m_deltaX = param["m_deltaX"].as<float>();
		if (param["m_deltaTime"]) soilParameters.m_deltaTime = param["m_deltaTime"].as<float>();
		if (param["m_boundingBoxMin"]) soilParameters.m_boundingBoxMin = param["m_boundingBoxMin"].as<glm::vec3>();

		if (param["m_boundary_x"]) soilParameters.m_boundary_x = static_cast<SoilModel::Boundary>(param["m_boundary_x"].as<int>());
		if (param["m_boundary_y"]) soilParameters.m_boundary_y = static_cast<SoilModel::Boundary>(param["m_boundary_y"].as<int>());
		if (param["m_boundary_z"]) soilParameters.m_boundary_z = static_cast<SoilModel::Boundary>(param["m_boundary_z"].as<int>());

		if (param["m_diffusionForce"]) soilParameters.m_diffusionForce = param["m_diffusionForce"].as<float>();
		if (param["m_gravityForce"]) soilParameters.m_gravityForce = param["m_gravityForce"].as<glm::vec3>();
	}
}

void SoilDescriptor::Serialize(YAML::Emitter& out)
{
	m_heightField.Save("m_heightField", out);
	SerializeSoilParameters("m_soilParameters", m_soilParameters, out);

	out << YAML::Key << "m_soilLayerDescriptors" << YAML::Value << YAML::BeginSeq;
	for (int i = 0; i < m_soilLayerDescriptors.size(); i++)
	{
		if (auto soilLayerDescriptor = m_soilLayerDescriptors[i].Get<SoilLayerDescriptor>())
		{
			out << YAML::BeginMap;
			m_soilLayerDescriptors[i].Serialize(out);
			out << YAML::EndMap;
		}
		else
		{
			m_soilLayerDescriptors.erase(m_soilLayerDescriptors.begin() + i);
			i--;
		}
	}
	out << YAML::EndSeq;
}

void SoilDescriptor::Deserialize(const YAML::Node& in)
{
	m_heightField.Load("m_heightField", in);
	DeserializeSoilParameters("m_soilParameters", m_soilParameters, in);
	m_soilLayerDescriptors.clear();
	if (in["m_soilLayerDescriptors"])
	{
		for (const auto& i : in["m_soilLayerDescriptors"])
		{
			m_soilLayerDescriptors.emplace_back();
			m_soilLayerDescriptors.back().Deserialize(i);
		}
	}
}

void SoilDescriptor::CollectAssetRef(std::vector<AssetRef>& list)
{
	list.push_back(m_heightField);

	for (int i = 0; i < m_soilLayerDescriptors.size(); i++)
	{
		if (auto soilLayerDescriptor = m_soilLayerDescriptors[i].Get<SoilLayerDescriptor>())
		{
			list.push_back(m_soilLayerDescriptors[i]);
		}
		else
		{
			m_soilLayerDescriptors.erase(m_soilLayerDescriptors.begin() + i);
			i--;
		}
	}
}

