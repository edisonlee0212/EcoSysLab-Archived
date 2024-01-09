#include "TreePointCloudScanner.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include <RayTracerLayer.hpp>
#include <CUDAModule.hpp>
#include <RayTracer.hpp>
#endif
#include "Tinyply.hpp"
using namespace tinyply;
#include "Soil.hpp"
#include "EcoSysLabLayer.hpp"
using namespace EcoSysLab;


void TreePointCloudScanner::Capture(const std::filesystem::path& savePath, const std::shared_ptr<PointCloudCaptureSettings>& captureSettings) const
{
#ifdef BUILD_WITH_RAYTRACER
	
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	std::shared_ptr<Soil> soil;
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) soil = soilCandidate.lock();
	if(!soil)
	{
		EVOENGINE_ERROR("No soil!");
		return;
	}
	std::unordered_map<Handle, Handle> branchMeshRendererHandles, foliageMeshRendererHandles;
	Bound plantBound{};
	auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	if(treeEntities == nullptr)
	{
		EVOENGINE_ERROR("No trees!");
		return;
	}
	for (const auto& treeEntity : *treeEntities) {
		if (scene->IsEntityValid(treeEntity)) {

			//auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			//auto copyPath = savePath;
			//tree->ExportJunction(ecoSysLabLayer->m_meshGeneratorSettings, copyPath.replace_extension(".yml"));

			scene->ForEachChild(treeEntity, [&](Entity child) {
				if (scene->GetEntityName(child) == "Branch Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
					const auto branchMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
					branchMeshRendererHandles.insert({ branchMeshRenderer->GetHandle(), treeEntity.GetIndex() });

					const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
					const auto mesh = branchMeshRenderer->m_mesh.Get<Mesh>();
					plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_min, 1.0f)));
					plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_max, 1.0f)));
				}
				else if (scene->GetEntityName(child) == "Foliage Mesh" &&
					scene->HasPrivateComponent<MeshRenderer>(child)) {
					const auto foliageMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
					foliageMeshRendererHandles.insert({ foliageMeshRenderer->GetHandle(), treeEntity.GetIndex() });

					const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
					const auto mesh = foliageMeshRenderer->m_mesh.Get<Mesh>();
					plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_min, 1.0f)));
					plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_max, 1.0f)));
				}
				else if (scene->GetEntityName(child) == "Twig Strands" &&
					scene->HasPrivateComponent<StrandsRenderer>(child)) {
					const auto twigStrandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(child).lock();
					branchMeshRendererHandles.insert({ twigStrandsRenderer->GetHandle(), treeEntity.GetIndex() });

					const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
					const auto strands = twigStrandsRenderer->m_strands.Get<Strands>();
					plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(strands->GetBound().m_min, 1.0f)));
					plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(strands->GetBound().m_max, 1.0f)));
				}
				}
			);
		}
	}
	Handle groundMeshRendererHandle = 0;
	auto soilEntity = soil->GetOwner();
	if (scene->IsEntityValid(soilEntity)) {
		scene->ForEachChild(soilEntity, [&](Entity child) {
			if (scene->GetEntityName(child) == "Ground Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
				groundMeshRendererHandle = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->GetHandle();
			}
			}
		);
	}
	
	std::vector<PointCloudSample> pcSamples;
	captureSettings->GenerateSamples(pcSamples);
	CudaModule::SamplePointCloud(
		Application::GetLayer<RayTracerLayer>()->m_environmentProperties,
		pcSamples);
	
	std::vector<glm::vec3> points;

	std::vector<int> internodeIndex;
	std::vector<int> branchIndex;
	std::vector<int> treePartIndex;
	std::vector<int> lineIndex;
	std::vector<int> instanceIndex;
	std::vector<int> typeIndex;

	

	for (const auto& sample : pcSamples) {
		if (!sample.m_hit) continue;
		if (!captureSettings->SampleFilter(sample)) continue;
		auto& position = sample.m_hitInfo.m_position;
		if (position.x<(plantBound.m_min.x - m_pointSettings.m_boundingBoxLimit) ||
			position.y<(plantBound.m_min.y - m_pointSettings.m_boundingBoxLimit) ||
			position.z<(plantBound.m_min.z - m_pointSettings.m_boundingBoxLimit) ||
			position.x>(plantBound.m_max.x + m_pointSettings.m_boundingBoxLimit) ||
			position.y>(plantBound.m_max.y + m_pointSettings.m_boundingBoxLimit) ||
			position.z>(plantBound.m_max.z + m_pointSettings.m_boundingBoxLimit))
			continue;
		auto ballRand = glm::vec3(0.0f);
		if (m_pointSettings.m_ballRandRadius > 0.0f) {
			ballRand = glm::ballRand(m_pointSettings.m_ballRandRadius);
		}
		const auto distance = glm::distance(sample.m_hitInfo.m_position, sample.m_start);
		points.emplace_back(
			sample.m_hitInfo.m_position +
			distance * glm::vec3(glm::gaussRand(0.0f, m_pointSettings.m_variance),
				glm::gaussRand(0.0f, m_pointSettings.m_variance),
				glm::gaussRand(0.0f, m_pointSettings.m_variance))
			+ ballRand);
		
		if (m_pointSettings.m_internodeIndex) {

			internodeIndex.emplace_back(static_cast<int>(sample.m_hitInfo.m_data.x + 0.1f));
		}
		if (m_pointSettings.m_branchIndex)
		{
			branchIndex.emplace_back(static_cast<int>(sample.m_hitInfo.m_data.y + 0.1f));
		}
		if (m_pointSettings.m_lineIndex)
		{
			lineIndex.emplace_back(static_cast<int>(sample.m_hitInfo.m_data.z + 0.1f));
		}
		if (m_pointSettings.m_treePartIndex)
		{
			treePartIndex.emplace_back(static_cast<int>(sample.m_hitInfo.m_data2.x + 0.1f));
		}
		
		auto branchSearch = branchMeshRendererHandles.find(sample.m_handle);
		auto foliageSearch = foliageMeshRendererHandles.find(sample.m_handle);
		if (m_pointSettings.m_instanceIndex)
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
		
		if (m_pointSettings.m_typeIndex) {
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

	if (m_pointSettings.m_typeIndex)
		cube_file.add_properties_to_element(
			"type_index", { "type_index" }, Type::INT32, typeIndex.size(),
			reinterpret_cast<uint8_t*>(typeIndex.data()), Type::INVALID, 0);

	if (m_pointSettings.m_instanceIndex) {
		cube_file.add_properties_to_element(
			"instance_index", { "instance_index" }, Type::INT32, instanceIndex.size(),
			reinterpret_cast<uint8_t*>(instanceIndex.data()), Type::INVALID, 0);
	}
	if (m_pointSettings.m_branchIndex)
	{
		cube_file.add_properties_to_element(
			"branch_index", { "branch_index" }, Type::INT32, branchIndex.size(),
			reinterpret_cast<uint8_t*>(branchIndex.data()), Type::INVALID, 0);
	}
	if (m_pointSettings.m_treePartIndex)
	{
		cube_file.add_properties_to_element(
			"tree_part_index", { "tree_part_index" }, Type::INT32, treePartIndex.size(),
			reinterpret_cast<uint8_t*>(treePartIndex.data()), Type::INVALID, 0);
	}
	if (m_pointSettings.m_lineIndex)
	{
		cube_file.add_properties_to_element(
			"line_index", { "line_index" }, Type::INT32, lineIndex.size(),
			reinterpret_cast<uint8_t*>(lineIndex.data()), Type::INVALID, 0);
	}
	if (m_pointSettings.m_internodeIndex)
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
	if (ImGui::TreeNodeEx("Circular Capture")) {
		static std::shared_ptr<PointCloudCircularCaptureSettings> captureSettings = std::make_shared<PointCloudCircularCaptureSettings>();
		captureSettings->OnInspect();
		FileUtils::SaveFile("Capture", "Point Cloud", { ".ply" }, [&](const std::filesystem::path& path) {
			Capture(path, captureSettings);
			}, false);
		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Grid Capture")) {
		static std::shared_ptr<PointCloudGridCaptureSettings> captureSettings = std::make_shared<PointCloudGridCaptureSettings>();
		captureSettings->OnInspect();
		FileUtils::SaveFile("Capture", "Point Cloud", { ".ply" }, [&](const std::filesystem::path& path) {
			Capture(path, captureSettings);
			}, false);
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx("Point settings")) {
		m_pointSettings.OnInspect();
		ImGui::TreePop();
	}

	
}

void TreePointCloudScanner::OnDestroy()
{
	m_pointSettings = {};
}

void TreePointCloudScanner::Serialize(YAML::Emitter& out)
{
	m_pointSettings.Serialize("m_pointSettings", out);
}


void TreePointCloudScanner::Deserialize(const YAML::Node& in)
{
	m_pointSettings.Deserialize("m_pointSettings", in);
}
