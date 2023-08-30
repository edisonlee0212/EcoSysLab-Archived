#include "pybind11/pybind11.h"
#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "WindowLayer.hpp"
#include "RenderLayer.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "PlayerController.hpp"
#include "Prefab.hpp"
#include "Time.hpp"
#include "PhysicsLayer.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "PhysicsLayer.hpp"
#include "ClassRegistry.hpp"
#include "TreeModel.hpp"
#include "Tree.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Trees.hpp"
#include "EcoSysLabLayer.hpp"
#include "RadialBoundingVolume.hpp"
#include "HeightField.hpp"
#include "ObjectRotator.hpp"
#include "SinglePipeProfile.hpp"
#include "SorghumLayer.hpp"
#include "TreePointCloud.hpp"
#include "PipeModelBase.hpp"
#include "Scene.hpp"
#ifdef RAYTRACERFACILITY

#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>

#endif

using namespace EvoEngine;

#ifdef RAYTRACERFACILITY
using namespace RayTracerFacility;
#endif
using namespace EcoSysLab;

namespace py = pybind11;

void RegisterClasses() {
    ClassRegistry::RegisterPrivateComponent<Tree>("Tree");
    ClassRegistry::RegisterPrivateComponent<TreePointCloud>("TreePointCloud");
    ClassRegistry::RegisterPrivateComponent<Soil>("Soil");
    ClassRegistry::RegisterPrivateComponent<Climate>("Climate");
    ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
    ClassRegistry::RegisterAsset<Trees>("Trees", { ".trees" });
    ClassRegistry::RegisterAsset<TreeDescriptor>("TreeDescriptor", { ".td" });
    ClassRegistry::RegisterAsset<SoilDescriptor>("SoilDescriptor", { ".sd" });
    ClassRegistry::RegisterAsset<ClimateDescriptor>("ClimateDescriptor", { ".cd" });
    ClassRegistry::RegisterAsset<RadialBoundingVolume>("RadialBoundingVolume", { ".rbv" });
    ClassRegistry::RegisterAsset<HeightField>("HeightField", { ".hf" });
    ClassRegistry::RegisterAsset<NoiseSoilLayerDescriptor>("NoiseSoilLayerDescriptor", { ".nsld" });
    ClassRegistry::RegisterPrivateComponent<SinglePipeProfile>("SinglePipeProfile");
    ClassRegistry::RegisterPrivateComponent<PipeModelBase>("PipeModelBase");
}

void PushWindowLayer() {
    Application::PushLayer<WindowLayer>();
}
void PushEditorLayer() {
    Application::PushLayer<EditorLayer>();
}
void PushRenderLayer() {
    Application::PushLayer<RenderLayer>();
}
void PushEcoSysLabLayer() {
    Application::PushLayer<EcoSysLabLayer>();
}
void PushSorghumLayer() {
    Application::PushLayer<SorghumLayer>();
}

void Initialize(const std::string& projectPath) {
    ApplicationInfo applicationInfo;
    applicationInfo.m_applicationName = "EcoSysLab";
    applicationInfo.m_projectPath = projectPath;
    Application::Initialize(applicationInfo);
}

void RenderScene(const std::string& projectPath, const int resolutionX, const int resolutionY, const std::string& outputPath)
{
    if (std::filesystem::path(projectPath).extension().string() != ".eveproj") {
        EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
        return;
    }
    if (!std::filesystem::is_regular_file(projectPath))
    {
        EVOENGINE_ERROR("Project doesn't exist!");
        return;
    }
    if (resolutionX <= 0 || resolutionY <= 0)
    {
        EVOENGINE_ERROR("Resolution error!");
        return;
    }
    RegisterClasses();
    Application::PushLayer<RenderLayer>();
    Application::PushLayer<EcoSysLabLayer>();
    Application::PushLayer<SorghumLayer>();

    ApplicationInfo applicationInfo;
    applicationInfo.m_projectPath = std::filesystem::path(projectPath);
    Application::Initialize(applicationInfo);
    EVOENGINE_LOG("Loaded scene at " + projectPath);
    Application::Start();
    
    
    auto scene = Application::GetActiveScene();
    const auto mainCamera = scene->m_mainCamera.Get<Camera>();
    mainCamera->Resize({ resolutionX, resolutionY });

    Application::Loop();
    Graphics::WaitForDeviceIdle();

    mainCamera->GetRenderTexture()->StoreToPng(outputPath);
    EVOENGINE_LOG("Exported image to " + outputPath);
    Application::Terminate();
}

void RenderMesh(const std::string& meshPath, const int resolutionX, const int resolutionY, const std::string& outputPath)
{
    if (!std::filesystem::is_regular_file(meshPath))
    {
        EVOENGINE_ERROR("Project doesn't exist!");
        return;
    }
    if (resolutionX <= 0 || resolutionY <= 0)
    {
        EVOENGINE_ERROR("Resolution error!");
        return;
    }
}

void YamlToMesh(const std::string& yamlPath, 
    const ConnectivityGraphSettings &connectivityGraphSettings, 
    const ReconstructionSettings& reconstructionSettings,
    const TreeMeshGeneratorSettings& meshGeneratorSettings,
    const std::string& meshPath) {
    auto treePointCloud = std::make_shared<TreePointCloud>();
    treePointCloud->ImportGraph(yamlPath);
    
    treePointCloud->EstablishConnectivityGraph(connectivityGraphSettings);
    treePointCloud->BuildTreeStructure(reconstructionSettings);
    auto meshes = treePointCloud->GenerateMeshes(meshGeneratorSettings);
    if (meshes.size() == 1) {
        meshes[0]->Export(meshPath);
    }
    else {
        int index = 0;
        for (const auto& i : meshes) {
            auto savePath = std::filesystem::path(meshPath);
            savePath.replace_filename(savePath.filename().string() + "_" + std::to_string(index));
            index++;
            i->Export(savePath);
        }
    }
}

PYBIND11_MODULE(pyecosyslab, m) {
    py::class_<Entity>(m, "Entity")
        .def("GetIndex", &Entity::GetIndex)
        .def("GetVersion", &Entity::GetVersion);

    py::class_<ConnectivityGraphSettings>(m, "ConnectivityGraphSettings");

    py::class_<ReconstructionSettings>(m, "ReconstructionSettings");

    py::class_<TreeMeshGeneratorSettings>(m, "TreeMeshGeneratorSettings");

    py::class_<Scene>(m, "Scene")
        .def("CreateEntity", static_cast<Entity(Scene::*)(const std::string&)>(&Scene::CreateEntity))
        .def("DeleteEntity", &Scene::DeleteEntity);


    py::class_<Application>(m, "Application")
        .def_static("Start", &Application::Start)
        .def("Loop", &Application::Loop)
        .def("Terminate", &Application::Terminate)
        .def("GetActiveScene", &Application::GetActiveScene);

    py::class_<ProjectManager>(m, "ProjectManager")
        .def("GetOrCreateProject", &ProjectManager::GetOrCreateProject);

    m.doc() = "EcoSysLab"; // optional module docstring
    m.def("yaml_to_mesh", &YamlToMesh, "YamlToMesh");
}