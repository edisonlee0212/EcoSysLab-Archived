#include "TreePointCloudScanner.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include <RayTracerLayer.hpp>
#include <CUDAModule.hpp>
#include <RayTracer.hpp>
#include "Tinyply.hpp"
#include "Soil.hpp"
using namespace tinyply;
#endif


using namespace EcoSysLab;

void TreePointCloudScanner::PointCloudPointSettings::OnInspect()
{
	ImGui::DragFloat("Point variance", &m_variance, 0.01f);
	ImGui::DragFloat("Point uniform random radius", &m_ballRandRadius, 0.01f);
	ImGui::DragFloat("Bounding box offset", &m_boundingBoxLimit, 0.01f);
	ImGui::Checkbox("Type Index", &m_typeIndex);
	ImGui::Checkbox("Instance Index", &m_instanceIndex);
	ImGui::Checkbox("Branch Index", &m_branchIndex);
	ImGui::Checkbox("Internode Index", &m_internodeIndex);
}

void TreePointCloudScanner::PointCloudPointSettings::Serialize(const std::string& name, YAML::Emitter& out) const
{
	out << YAML::Key << name << YAML::Value << YAML::BeginMap;
	out << YAML::Key << "m_variance" << YAML::Value << m_variance;
	out << YAML::Key << "m_ballRandRadius" << YAML::Value << m_ballRandRadius;
	out << YAML::Key << "m_typeIndex" << YAML::Value << m_typeIndex;
	out << YAML::Key << "m_instanceIndex" << YAML::Value << m_instanceIndex;
	out << YAML::Key << "m_branchIndex" << YAML::Value << m_branchIndex;
	out << YAML::Key << "m_internodeIndex" << YAML::Value << m_internodeIndex;
	out << YAML::Key << "m_boundingBoxLimit" << YAML::Value << m_boundingBoxLimit;
	out << YAML::EndMap;
}

void TreePointCloudScanner::PointCloudPointSettings::Deserialize(const std::string& name, const YAML::Node& in)
{
	if (in[name]) {
		auto& cd = in[name];
		if (cd["m_variance"]) m_variance = cd["m_variance"].as<float>();
		if (cd["m_ballRandRadius"]) m_ballRandRadius = cd["m_ballRandRadius"].as<float>();
		if (cd["m_typeIndex"]) m_typeIndex = cd["m_typeIndex"].as<bool>();
		if (cd["m_instanceIndex"]) m_instanceIndex = cd["m_instanceIndex"].as<bool>();
		if (cd["m_branchIndex"]) m_branchIndex = cd["m_branchIndex"].as<bool>();
		if (cd["m_internodeIndex"]) m_internodeIndex = cd["m_internodeIndex"].as<bool>();
		if (cd["m_boundingBoxLimit"]) m_boundingBoxLimit = cd["m_boundingBoxLimit"].as<float>();
	}
}

void TreePointCloudScanner::CameraCaptureSettings::OnInspect()
{
	ImGui::Checkbox("Auto adjust focus point", &m_autoAdjustFocusPoint);
	if (!m_autoAdjustFocusPoint) {
		ImGui::Text("Position:");
		ImGui::DragFloat3("Focus point", &m_focusPoint.x, 0.1f);
		ImGui::DragFloat("Distance to focus point", &m_distance, 0.1);
	}
	ImGui::Separator();
	ImGui::Text("Rotation:");
	ImGui::DragInt3("Pitch Angle Start/Step/End", &m_pitchAngleStart, 1);
	ImGui::DragInt3("Turn Angle Start/Step/End", &m_turnAngleStart, 1);
	ImGui::Separator();
	ImGui::Text("Camera Settings:");
	ImGui::DragFloat("FOV", &m_fov);
	ImGui::DragInt2("Resolution", &m_resolution.x);
	ImGui::DragFloat("Max Depth", &m_cameraDepthMax);
	ImGui::Checkbox("Use clear color", &m_useClearColor);
	if (m_useClearColor) ImGui::ColorEdit3("Clear Color", &m_backgroundColor.x);

	ImGui::Separator();
	ImGui::Text("Ray tracer Settings");
	ImGui::DragInt("Bounce", &m_rayBounce);
	ImGui::DragInt("Sample", &m_raySample);
}

void TreePointCloudScanner::CameraCaptureSettings::Serialize(const std::string& name, YAML::Emitter& out) const
{
	out << YAML::Key << name << YAML::Value << YAML::BeginMap;
	out << YAML::Key << "m_autoAdjustFocusPoint" << YAML::Value
		<< m_autoAdjustFocusPoint;
	out << YAML::Key << "m_focusPoint" << YAML::Value << m_focusPoint;
	out << YAML::Key << "m_pitchAngleStart" << YAML::Value << m_pitchAngleStart;
	out << YAML::Key << "m_pitchAngleStep" << YAML::Value << m_pitchAngleStep;
	out << YAML::Key << "m_pitchAngleEnd" << YAML::Value << m_pitchAngleEnd;
	out << YAML::Key << "m_turnAngleStart" << YAML::Value << m_turnAngleStart;
	out << YAML::Key << "m_turnAngleStep" << YAML::Value << m_turnAngleStep;
	out << YAML::Key << "m_turnAngleEnd" << YAML::Value << m_turnAngleEnd;
	out << YAML::Key << "m_distance" << YAML::Value << m_distance;
	out << YAML::Key << "m_fov" << YAML::Value << m_fov;
	out << YAML::Key << "m_resolution" << YAML::Value << m_resolution;
	out << YAML::Key << "m_useClearColor" << YAML::Value << m_useClearColor;
	out << YAML::Key << "m_backgroundColor" << YAML::Value << m_backgroundColor;
	out << YAML::Key << "m_cameraDepthMax" << YAML::Value << m_cameraDepthMax;
	out << YAML::Key << "m_rayBounce" << YAML::Value << m_rayBounce;
	out << YAML::Key << "m_raySample" << YAML::Value << m_raySample;
	out << YAML::EndMap;
}

void TreePointCloudScanner::CameraCaptureSettings::Deserialize(const std::string& name, const YAML::Node& in)
{
	if (in[name]) {
		auto& cd = in[name];
		if (cd["m_autoAdjustFocusPoint"]) m_autoAdjustFocusPoint = cd["m_autoAdjustFocusPoint"].as<bool>();
		if (cd["m_focusPoint"]) m_focusPoint = cd["m_focusPoint"].as<glm::vec3>();
		if (cd["m_pitchAngleStart"]) m_pitchAngleStart = cd["m_pitchAngleStart"].as<int>();
		if (cd["m_pitchAngleStep"]) m_pitchAngleStep = cd["m_pitchAngleStep"].as<int>();
		if (cd["m_pitchAngleEnd"]) m_pitchAngleEnd = cd["m_pitchAngleEnd"].as<int>();
		if (cd["m_turnAngleStart"]) m_turnAngleStart = cd["m_turnAngleStart"].as<int>();
		if (cd["m_turnAngleStep"]) m_turnAngleStep = cd["m_turnAngleStep"].as<int>();
		if (cd["m_turnAngleEnd"]) m_turnAngleEnd = cd["m_turnAngleEnd"].as<int>();
		if (cd["m_distance"]) m_distance = cd["m_distance"].as<float>();
		if (cd["m_fov"]) m_fov = cd["m_fov"].as<float>();
		if (cd["m_resolution"]) m_resolution = cd["m_resolution"].as<glm::ivec2>();
		if (cd["m_useClearColor"]) m_useClearColor = cd["m_useClearColor"].as<bool>();
		if (cd["m_backgroundColor"]) m_backgroundColor = cd["m_backgroundColor"].as<glm::vec3>();
		if (cd["m_cameraDepthMax"]) m_cameraDepthMax = cd["m_cameraDepthMax"].as<float>();
		if (cd["m_rayBounce"]) m_rayBounce = cd["m_rayBounce"].as<float>();
		if (cd["m_raySample"]) m_raySample = cd["m_raySample"].as<float>();
	}
}

GlobalTransform TreePointCloudScanner::CameraCaptureSettings::GetTransform(const Bound& bound,
	const float turnAngle, const float pitchAngle) const
{
	GlobalTransform cameraGlobalTransform;
	float distance = m_distance;
	glm::vec3 focusPoint = m_focusPoint;
	if (m_autoAdjustFocusPoint) {
		focusPoint = (bound.m_min + bound.m_max) / 2.0f;
		const float halfAngle = (m_fov - 35.0f) / 2.0f;
		float width = bound.m_max.y - bound.m_min.y;
		if (width < bound.m_max.x - bound.m_min.x) {
			width = bound.m_max.x - bound.m_min.x;
		}
		if (width < bound.m_max.z - bound.m_min.z) {
			width = bound.m_max.z - bound.m_min.z;
		}
		width /= 2.0f;
		distance = width / glm::tan(glm::radians(halfAngle));
	}
	const auto height = distance * glm::sin(glm::radians((float)pitchAngle));
	const auto groundDistance =
		distance * glm::cos(glm::radians((float)pitchAngle));
	const glm::vec3 cameraPosition =
		glm::vec3(glm::sin(glm::radians((float)turnAngle)) * groundDistance,
			height,
			glm::cos(glm::radians((float)turnAngle)) * groundDistance);


	cameraGlobalTransform.SetPosition(cameraPosition + focusPoint);
	cameraGlobalTransform.SetRotation(glm::quatLookAt(glm::normalize(-cameraPosition), glm::vec3(0, 1, 0)));
	return cameraGlobalTransform;
}

void TreePointCloudScanner::GeneratePointCloud(const std::filesystem::path& savePath)
{
#ifdef BUILD_WITH_RAYTRACER
	auto tree = m_tree.Get<Tree>();
	if (!tree) {
		EVOENGINE_ERROR("No tree!");
		return;
	}

	std::unordered_map<Handle, Handle> branchMeshRendererHandles, foliageMeshRendererHandles;
	Bound plantBound{};
	auto scene = GetScene();
	auto treeOwner = tree->GetOwner();
	if (scene->IsEntityValid(treeOwner)) {
		scene->ForEachChild(treeOwner, [&](Entity child) {
			if (scene->GetEntityName(child) == "Branch Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
				const auto branchMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
				branchMeshRendererHandles.insert({ branchMeshRenderer->GetHandle(), treeOwner.GetIndex() });

				const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
				const auto mesh = branchMeshRenderer->m_mesh.Get<Mesh>();
				plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_min, 1.0f)));
				plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_max, 1.0f)));
			}
			else if (scene->GetEntityName(child) == "Foliage Mesh" &&
				scene->HasPrivateComponent<MeshRenderer>(child)) {
				const auto foliageMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
				foliageMeshRendererHandles.insert({ foliageMeshRenderer->GetHandle(), treeOwner.GetIndex() });

				const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
				const auto mesh = foliageMeshRenderer->m_mesh.Get<Mesh>();
				plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_min, 1.0f)));
				plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_max, 1.0f)));
			}
			else if (scene->GetEntityName(child) == "Twig Strands" &&
				scene->HasPrivateComponent<StrandsRenderer>(child)) {
				const auto twigStrandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(child).lock();
				branchMeshRendererHandles.insert({ twigStrandsRenderer->GetHandle(), treeOwner.GetIndex() });

				const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
				const auto strands = twigStrandsRenderer->m_strands.Get<Strands>();
				plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(strands->GetBound().m_min, 1.0f)));
				plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(strands->GetBound().m_max, 1.0f)));
			}
			});
	}

	Handle groundMeshRendererHandle = 0;
	const auto soil = m_soil.Get<Soil>();
	auto soilOwner = soil->GetOwner();
	if (scene->IsEntityValid(soilOwner)) {
		scene->ForEachChild(soilOwner, [&](Entity child) {
			if (scene->GetEntityName(child) == "Ground Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
				groundMeshRendererHandle = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->GetHandle();
			}
			}
		);
	}
	
	std::vector<PointCloudSample> pcSamples;
	int counter = 0;
	for (int turnAngle = m_pointCloudSettings.m_turnAngleStart;
		turnAngle < m_pointCloudSettings.m_turnAngleEnd; turnAngle += m_pointCloudSettings.m_turnAngleStep) {
		for (int pitchAngle = m_pointCloudSettings.m_pitchAngleStart;
			pitchAngle < m_pointCloudSettings.m_pitchAngleEnd; pitchAngle += m_pointCloudSettings.m_pitchAngleStep) {
			pcSamples.resize((counter + 1) * m_pointCloudSettings.m_resolution.x * m_pointCloudSettings.m_resolution.y);
			auto scannerGlobalTransform = m_pointCloudSettings.GetTransform(plantBound, turnAngle, pitchAngle);
			auto front = scannerGlobalTransform.GetRotation() * glm::vec3(0, 0, -1);
			auto up = scannerGlobalTransform.GetRotation() * glm::vec3(0, 1, 0);
			auto left = scannerGlobalTransform.GetRotation() * glm::vec3(1, 0, 0);
			auto position = scannerGlobalTransform.GetPosition();
			std::vector<std::shared_future<void>> results;
			Jobs::ParallelFor(
				m_pointCloudSettings.m_resolution.x * m_pointCloudSettings.m_resolution.y,
				[&](unsigned i) {
					unsigned x = i % m_pointCloudSettings.m_resolution.x;
					unsigned y = i / m_pointCloudSettings.m_resolution.x;
					const float xAngle = (x - m_pointCloudSettings.m_resolution.x / 2.0f) /
						(float)m_pointCloudSettings.m_resolution.x * m_pointCloudSettings.m_fov /
						2.0f;
					const float yAngle = (y - m_pointCloudSettings.m_resolution.y / 2.0f) /
						(float)m_pointCloudSettings.m_resolution.y * m_pointCloudSettings.m_fov /
						2.0f;
					auto& sample = pcSamples[
						counter * m_pointCloudSettings.m_resolution.x * m_pointCloudSettings.m_resolution.y +
							i];
					sample.m_direction = glm::normalize(glm::rotate(glm::rotate(front, glm::radians(xAngle), left),
						glm::radians(yAngle), up));
					sample.m_start = position;
				},
				results);
			for (const auto& i : results)
				i.wait();

			counter++;
		}
	}
	CudaModule::SamplePointCloud(
		Application::GetLayer<RayTracerLayer>()->m_environmentProperties,
		pcSamples);
	
	std::vector<glm::vec3> points;

	std::vector<int> internodeIndex;
	std::vector<int> branchIndex;
	std::vector<int> instanceIndex;
	std::vector<int> typeIndex;

	

	for (const auto& sample : pcSamples) {
		if (!sample.m_hit) continue;
		auto& position = sample.m_hitInfo.m_position;
		if (position.x<(plantBound.m_min.x - m_pointCloudPointSettings.m_boundingBoxLimit) ||
			position.y<(plantBound.m_min.y - m_pointCloudPointSettings.m_boundingBoxLimit) ||
			position.z<(plantBound.m_min.z - m_pointCloudPointSettings.m_boundingBoxLimit) ||
			position.x>(plantBound.m_max.x + m_pointCloudPointSettings.m_boundingBoxLimit) ||
			position.y>(plantBound.m_max.y + m_pointCloudPointSettings.m_boundingBoxLimit) ||
			position.z>(plantBound.m_max.z + m_pointCloudPointSettings.m_boundingBoxLimit))
			continue;
		auto ballRand = glm::vec3(0.0f);
		if (m_pointCloudPointSettings.m_ballRandRadius > 0.0f) {
			ballRand = glm::ballRand(m_pointCloudPointSettings.m_ballRandRadius);
		}
		points.emplace_back(
			sample.m_hitInfo.m_position +
			glm::vec3(glm::gaussRand(0.0f, m_pointCloudPointSettings.m_variance),
				glm::gaussRand(0.0f, m_pointCloudPointSettings.m_variance),
				glm::gaussRand(0.0f, m_pointCloudPointSettings.m_variance))
			+ ballRand);
		
		if (m_pointCloudPointSettings.m_internodeIndex) {

			internodeIndex.emplace_back(static_cast<int>(sample.m_hitInfo.m_data.x + 0.1f));
		}
		if (m_pointCloudPointSettings.m_branchIndex)
		{
			branchIndex.emplace_back(static_cast<int>(sample.m_hitInfo.m_data.y + 0.1f));
		}
		auto branchSearch = branchMeshRendererHandles.find(sample.m_handle);
		auto foliageSearch = foliageMeshRendererHandles.find(sample.m_handle);
		if (m_pointCloudPointSettings.m_instanceIndex)
		{
			if (branchSearch != branchMeshRendererHandles.end())
			{
				instanceIndex.emplace_back(branchSearch->second);
			}
			else if (foliageSearch != foliageMeshRendererHandles.end())
			{
				instanceIndex.emplace_back(foliageSearch->second);
			}else
			{
				instanceIndex.emplace_back(0);
			}
		}
		
		if (m_pointCloudPointSettings.m_typeIndex) {
			if (branchSearch != branchMeshRendererHandles.end())
			{
				typeIndex.emplace_back(0);
			}
			else if (foliageSearch != foliageMeshRendererHandles.end())
			{
				typeIndex.emplace_back(1);
			}
			else if (sample.m_handle == groundMeshRendererHandle) {
				typeIndex.emplace_back(2);
			}
			else {
				typeIndex.emplace_back(-1);
			}
		}
	}
	std::filebuf fb_binary;
	fb_binary.open(savePath.string(), std::ios::out | std::ios::binary);
	std::ostream outstream_binary(&fb_binary);
	if (outstream_binary.fail())
		throw std::runtime_error("failed to open " + savePath.string());
	/*
	std::filebuf fb_ascii;
	fb_ascii.open(filename + "-ascii.ply", std::ios::out);
	std::ostream outstream_ascii(&fb_ascii);
	if (outstream_ascii.fail()) throw std::runtime_error("failed to open " +
	filename);
	*/
	PlyFile cube_file;
	cube_file.add_properties_to_element(
		"vertex", { "x", "y", "z" }, Type::FLOAT32, points.size(),
		reinterpret_cast<uint8_t*>(points.data()), Type::INVALID, 0);

	if (m_pointCloudPointSettings.m_typeIndex)
		cube_file.add_properties_to_element(
			"type_index", { "type_index" }, Type::INT32, typeIndex.size(),
			reinterpret_cast<uint8_t*>(typeIndex.data()), Type::INVALID, 0);

	if (m_pointCloudPointSettings.m_instanceIndex) {
		cube_file.add_properties_to_element(
			"instance_index", { "instance_index" }, Type::INT32, instanceIndex.size(),
			reinterpret_cast<uint8_t*>(instanceIndex.data()), Type::INVALID, 0);
	}
	if (m_pointCloudPointSettings.m_branchIndex)
	{
		cube_file.add_properties_to_element(
			"branch_index", { "branch_index" }, Type::INT32, branchIndex.size(),
			reinterpret_cast<uint8_t*>(branchIndex.data()), Type::INVALID, 0);
	}
	if (m_pointCloudPointSettings.m_internodeIndex)
	{
		cube_file.add_properties_to_element(
			"internode_index", { "internode_index" }, Type::INT32, internodeIndex.size(),
			reinterpret_cast<uint8_t*>(internodeIndex.data()), Type::INVALID, 0);
	}
	// Write a binary file
	cube_file.write(outstream_binary, true);
#endif
}

void TreePointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	editorLayer->DragAndDropButton<Tree>(m_tree, "Target tree");
	editorLayer->DragAndDropButton<Soil>(m_soil, "Target soil");
	if (ImGui::TreeNodeEx("Point cloud settings")) {
		m_pointCloudSettings.OnInspect();
		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Point settings")) {
		m_pointCloudPointSettings.OnInspect();
		ImGui::TreePop();
	}

	FileUtils::SaveFile("Capture point cloud", "Point Cloud", { ".ply" }, [&](const std::filesystem::path& path) {
		GeneratePointCloud(path);
		}, false);
}

void TreePointCloudScanner::Serialize(YAML::Emitter& out)
{
	m_pointCloudPointSettings.Serialize("m_pointCloudPointSettings", out);
	m_pointCloudSettings.Serialize("m_pointCloudSettings", out);
	m_tree.Save("m_tree", out);
	m_soil.Save("m_soil", out);
}

void TreePointCloudScanner::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene)
{
	m_tree.Relink(map, scene);
	m_soil.Relink(map, scene);
}

void TreePointCloudScanner::Deserialize(const YAML::Node& in)
{
	m_pointCloudPointSettings.Deserialize("m_pointCloudPointSettings", in);
	m_pointCloudSettings.Deserialize("m_pointCloudSettings", in);
	m_tree.Load("m_tree", in, GetScene());
	m_soil.Load("m_soil", in, GetScene());
}
