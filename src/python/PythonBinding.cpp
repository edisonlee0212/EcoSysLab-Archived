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

void StartEditor(const std::string& projectPath) {
    RegisterClasses();

    Application::PushLayer<WindowLayer>();
    Application::PushLayer<PhysicsLayer>();
    Application::PushLayer<EditorLayer>();
    Application::PushLayer<RenderLayer>();
    Application::PushLayer<EcoSysLabLayer>();
    Application::PushLayer<SorghumLayer>();
#ifdef RAYTRACERFACILITY
    Application::PushLayer<RayTracerLayer>();
#endif

    ApplicationInfo applicationInfo;
    applicationInfo.m_applicationName = "EcoSysLab";
    applicationInfo.m_projectPath = projectPath;
    Application::Initialize(applicationInfo);
    Application::Start();

    Application::Terminate();
}

void StartEmptyEditor() {
    RegisterClasses();
    Application::PushLayer<WindowLayer>();
    Application::PushLayer<PhysicsLayer>();
    Application::PushLayer<EditorLayer>();
    Application::PushLayer<RenderLayer>();
    Application::PushLayer<EcoSysLabLayer>();
    Application::PushLayer<SorghumLayer>();
#ifdef RAYTRACERFACILITY
    Application::PushLayer<RayTracerLayer>();
#endif

    ApplicationInfo applicationInfo;
    applicationInfo.m_applicationName = "EcoSysLab";
    Application::Initialize(applicationInfo);
    Application::Start();

    Application::Terminate();
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
    Application::PushLayer<PhysicsLayer>();
    Application::PushLayer<RenderLayer>();
    Application::PushLayer<EcoSysLabLayer>();
    Application::PushLayer<SorghumLayer>();

    ApplicationInfo applicationInfo;
    applicationInfo.m_projectPath = std::filesystem::path(projectPath);
    ProjectManager::SetScenePostLoadActions([&](const std::shared_ptr<Scene>& scene)
        {
            const auto mainCamera = scene->m_mainCamera.Get<Camera>();
            mainCamera->Resize({ resolutionX, resolutionY });
        });
    Application::Initialize(applicationInfo);
    EVOENGINE_LOG("Loaded scene at " + projectPath);
    Application::Start(false);
    Application::Loop();
    Graphics::WaitForDeviceIdle();
    auto scene = Application::GetActiveScene();
    const auto mainCamera = scene->m_mainCamera.Get<Camera>();
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

PYBIND11_MODULE(pyevoengine, m) {
    m.doc() = "EvoEngine"; // optional module docstring
    m.def("start_editor", &StartEditor, "Start editor with target project");
    m.def("start_empty_editor", &StartEmptyEditor, "Start editor without project");
    m.def("render_scene", &RenderScene, "Render the default scene for given project");
}