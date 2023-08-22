// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef RAYTRACERFACILITY

#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>

#endif

#include <Time.hpp>

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
using namespace RayTracerFacility;
#endif
using namespace EcoSysLab;

void EngineSetup();


int main() {
    const std::filesystem::path resourceFolderPath("../../../Resources");
    for (auto i : std::filesystem::recursive_directory_iterator(resourceFolderPath))
    {
        if (i.is_directory()) continue;
        auto oldPath = i.path();
        auto newPath = i.path();
        bool remove = false;
        if (i.path().extension().string() == ".uescene")
        {
            newPath.replace_extension(".evescene");
            remove = true;
        }
        if (i.path().extension().string() == ".umeta")
        {
            newPath.replace_extension(".evefilemeta");
            remove = true;
        }
        if (i.path().extension().string() == ".ueproj")
        {
            newPath.replace_extension(".eveproj");
            remove = true;
        }
        if (i.path().extension().string() == ".ufmeta")
        {
            newPath.replace_extension(".evefoldermeta");
            remove = true;
        }
        if (remove) {
            std::filesystem::copy(oldPath, newPath);
            std::filesystem::remove(oldPath);
        }
    }

	EngineSetup();

    Application::PushLayer<WindowLayer>();
    Application::PushLayer<PhysicsLayer>();
    Application::PushLayer<EditorLayer>();
    Application::PushLayer<RenderLayer>();
    Application::PushLayer<EcoSysLabLayer>();
    Application::PushLayer<SorghumLayer>();
    Application::PushLayer<RayTracerLayer>();

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

    ApplicationInfo applicationConfigs;
    applicationConfigs.m_applicationName = "EcoSysLab";
    Application::Initialize(applicationConfigs);
    
#ifdef RAYTRACERFACILITY
    
    auto rayTracerLayer = Application::GetLayer<RayTracerLayer>();
    rayTracerLayer->m_showCameraWindow = false;
    rayTracerLayer->m_showSceneWindow = false;
    rayTracerLayer->m_showRayTracerWindow = false;
#endif

    // adjust default camera speed
    auto editorLayer = Application::GetLayer<EditorLayer>();
    editorLayer->m_velocity = 2.f;
    editorLayer->m_defaultSceneCameraPosition = glm::vec3(1.124, 0.218, 14.089);
    // override default scene camera position etc.
    editorLayer->m_showCameraWindow = false;
    editorLayer->m_showSceneWindow = true;
    editorLayer->m_showEntityExplorerWindow = true;
    editorLayer->m_showEntityInspectorWindow = true;
    
#pragma region Engine Loop
    Application::Start();
#pragma endregion
    Application::End();
}

void EngineSetup() {
    ProjectManager::SetScenePostLoadActions([=](const std::shared_ptr<Scene>& scene) {
#pragma region Engine Setup
        Transform transform;
        transform.SetEulerRotation(glm::radians(glm::vec3(150, 30, 0)));
#pragma region Preparations
        Time::SetTimeStep(0.016f);
        transform = Transform();
        transform.SetPosition(glm::vec3(0, 2, 35));
        transform.SetEulerRotation(glm::radians(glm::vec3(15, 0, 0)));
        auto mainCamera = Application::GetActiveScene()->m_mainCamera.Get<EvoEngine::Camera>();
        if (mainCamera) {
            
            scene->SetDataComponent(mainCamera->GetOwner(), transform);
            mainCamera->m_useClearColor = true;
            mainCamera->m_clearColor = glm::vec3(0.5f);
        }
#pragma endregion
#pragma endregion

    });
}
