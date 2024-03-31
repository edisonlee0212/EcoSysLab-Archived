#include "SorghumPointCloudScanner.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include <RayTracerLayer.hpp>
#include <CUDAModule.hpp>
#include <RayTracer.hpp>
#endif
#include "EcoSysLabLayer.hpp"
#include "Sorghum.hpp"
#include "Tinyply.hpp"
#include "TreePointCloudScanner.hpp"
using namespace tinyply;
using namespace EcoSysLab;

void SorghumPointCloudPointSettings::OnInspect()
{
}

void SorghumPointCloudPointSettings::Save(const std::string& name, YAML::Emitter& out) const
{
}

void SorghumPointCloudPointSettings::Load(const std::string& name, const YAML::Node& in)
{
}

void SorghumPointCloudScanner::Capture(const std::filesystem::path& savePath,
                                       const std::shared_ptr<PointCloudCaptureSettings>& captureSettings) const
{
#ifdef BUILD_WITH_RAYTRACER
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	std::shared_ptr<Soil> soil;
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) soil = soilCandidate.lock();
	if (!soil)
	{
		EVOENGINE_ERROR("No soil!");
		return;
	}
	Bound plantBound{};
	std::unordered_map<Handle, Handle> leafMeshRendererHandles, stemMeshRendererHandles, panicleMeshRendererHandles;
	const auto scene = GetScene();
	const std::vector<Entity>* sorghumEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Sorghum>();
	if (sorghumEntities == nullptr)
	{
		EVOENGINE_ERROR("No sorghums!");
		return;
	}
	for (const auto& sorghumEntity : *sorghumEntities) {
		if (scene->IsEntityValid(sorghumEntity))
		{
			scene->ForEachChild(sorghumEntity, [&](Entity child)
				{
					if (scene->GetEntityName(child) == "Leaf Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
						const auto leafMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
						leafMeshRendererHandles.insert({ leafMeshRenderer->GetHandle(), sorghumEntity.GetIndex() });

						const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
						const auto mesh = leafMeshRenderer->m_mesh.Get<Mesh>();
						plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_min, 1.0f)));
						plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_max, 1.0f)));
					}
					else if (scene->GetEntityName(child) == "Stem Mesh" &&
						scene->HasPrivateComponent<Particles>(child)) {
						const auto stemMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
						stemMeshRendererHandles.insert({ stemMeshRenderer->GetHandle(), sorghumEntity.GetIndex() });

						const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
						const auto mesh = stemMeshRenderer->m_mesh.Get<Mesh>();
						plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_min, 1.0f)));
						plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_max, 1.0f)));
					}
					else if (scene->GetEntityName(child) == "Panicle Strands" &&
						scene->HasPrivateComponent<StrandsRenderer>(child)) {
						const auto panicleMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
						panicleMeshRendererHandles.insert({ panicleMeshRenderer->GetHandle(), sorghumEntity.GetIndex() });

						const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
						const auto mesh = panicleMeshRenderer->m_mesh.Get<Mesh>();
						plantBound.m_min = glm::min(plantBound.m_min, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_min, 1.0f)));
						plantBound.m_max = glm::max(plantBound.m_max, glm::vec3(globalTransform.m_value * glm::vec4(mesh->GetBound().m_max, 1.0f)));
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
	std::vector<int> leafIndex;
	std::vector<int> instanceIndex;
	std::vector<int> typeIndex;

	for (const auto& sample : pcSamples) {
		if (!sample.m_hit) continue;
		if (!captureSettings->SampleFilter(sample)) continue;
		auto& position = sample.m_hitInfo.m_position;
		if (position.x<(plantBound.m_min.x - m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
			position.y<(plantBound.m_min.y - m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
			position.z<(plantBound.m_min.z - m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
			position.x>(plantBound.m_max.x + m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
			position.y>(plantBound.m_max.y + m_sorghumPointCloudPointSettings.m_boundingBoxLimit) ||
			position.z>(plantBound.m_max.z + m_sorghumPointCloudPointSettings.m_boundingBoxLimit))
			continue;
		auto ballRand = glm::vec3(0.0f);
		if (m_sorghumPointCloudPointSettings.m_ballRandRadius > 0.0f) {
			ballRand = glm::ballRand(m_sorghumPointCloudPointSettings.m_ballRandRadius);
		}
		const auto distance = glm::distance(sample.m_hitInfo.m_position, sample.m_start);
		points.emplace_back(
			sample.m_hitInfo.m_position +
			distance * glm::vec3(glm::gaussRand(0.0f, m_sorghumPointCloudPointSettings.m_variance),
				glm::gaussRand(0.0f, m_sorghumPointCloudPointSettings.m_variance),
				glm::gaussRand(0.0f, m_sorghumPointCloudPointSettings.m_variance))
			+ ballRand);

		
		if (m_sorghumPointCloudPointSettings.m_leafIndex)
		{
			leafIndex.emplace_back(static_cast<int>(sample.m_hitInfo.m_data.x + 0.1f));
		}

		auto leafSearch = leafMeshRendererHandles.find(sample.m_handle);
		auto stemSearch = stemMeshRendererHandles.find(sample.m_handle);
		auto panicleSearch = panicleMeshRendererHandles.find(sample.m_handle);
		if (m_sorghumPointCloudPointSettings.m_instanceIndex)
		{
			if (leafSearch != leafMeshRendererHandles.end())
			{
				instanceIndex.emplace_back(leafSearch->second);
			}
			else if (stemSearch != stemMeshRendererHandles.end())
			{
				instanceIndex.emplace_back(stemSearch->second);
			}
			else if (panicleSearch != panicleMeshRendererHandles.end())
			{
				instanceIndex.emplace_back(panicleSearch->second);
			}
			else
			{
				instanceIndex.emplace_back(0);
			}
		}

		if (m_sorghumPointCloudPointSettings.m_typeIndex) {
			if (leafSearch != leafMeshRendererHandles.end())
			{
				typeIndex.emplace_back(0);
			}
			else if (stemSearch != stemMeshRendererHandles.end())
			{
				typeIndex.emplace_back(1);
			}
			else if (panicleSearch != panicleMeshRendererHandles.end())
			{
				typeIndex.emplace_back(2);
			}
			else if (sample.m_handle == groundMeshRendererHandle) {
				typeIndex.emplace_back(3);
			}
			else {
				typeIndex.emplace_back(-1);
			}
		}
	}
	std::filebuf fb_binary;
	fb_binary.open(savePath.string(), std::ios::out | std::ios::binary);
	std::ostream ostream(&fb_binary);
	if (ostream.fail())
		throw std::runtime_error("failed to open " + savePath.string());

	PlyFile cube_file;
	cube_file.add_properties_to_element(
		"vertex", { "x", "y", "z" }, Type::FLOAT32, points.size(),
		reinterpret_cast<uint8_t*>(points.data()), Type::INVALID, 0);

	if (m_sorghumPointCloudPointSettings.m_typeIndex)
		cube_file.add_properties_to_element(
			"type_index", { "type_index" }, Type::INT32, typeIndex.size(),
			reinterpret_cast<uint8_t*>(typeIndex.data()), Type::INVALID, 0);

	if (m_sorghumPointCloudPointSettings.m_instanceIndex) {
		cube_file.add_properties_to_element(
			"instance_index", { "instance_index" }, Type::INT32, instanceIndex.size(),
			reinterpret_cast<uint8_t*>(instanceIndex.data()), Type::INVALID, 0);
	}

	if (m_sorghumPointCloudPointSettings.m_leafIndex)
	{
		cube_file.add_properties_to_element(
			"leaf_index", { "leaf_index" }, Type::INT32, leafIndex.size(),
			reinterpret_cast<uint8_t*>(leafIndex.data()), Type::INVALID, 0);
	}

	// Write a binary file
	cube_file.write(ostream, true);
#endif
}

void SorghumPointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	if (ImGui::TreeNodeEx("Grid Capture")) {
		static std::shared_ptr<TreePointCloudGridCaptureSettings> captureSettings = std::make_shared<TreePointCloudGridCaptureSettings>();
		captureSettings->OnInspect();
		FileUtils::SaveFile("Capture", "Point Cloud", { ".ply" }, [&](const std::filesystem::path& path) {
			Capture(path, captureSettings);
			}, false);
		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Point settings")) {
		m_sorghumPointCloudPointSettings.OnInspect();
		ImGui::TreePop();
	}
}

void SorghumPointCloudScanner::OnDestroy()
{
	m_sorghumPointCloudPointSettings = {};
}

void SorghumPointCloudScanner::Serialize(YAML::Emitter& out)
{
	m_sorghumPointCloudPointSettings.Save("m_sorghumPointCloudPointSettings", out);
}

void SorghumPointCloudScanner::Deserialize(const YAML::Node& in)
{
	m_sorghumPointCloudPointSettings.Load("m_sorghumPointCloudPointSettings", in);
}
