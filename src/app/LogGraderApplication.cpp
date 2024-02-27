// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#include "Times.hpp"

#include "ProjectManager.hpp"
#include "ClassRegistry.hpp"
#include "Tree.hpp"
#include "HeightField.hpp"
#include "LogGrader.hpp"
#include "ObjectRotator.hpp"
#include "WindowLayer.hpp"
using namespace EcoSysLab;

void EngineSetup();


int main() {
	EngineSetup();

	Application::PushLayer<WindowLayer>();
	Application::PushLayer<EditorLayer>();
	Application::PushLayer<RenderLayer>();

	ClassRegistry::RegisterPrivateComponent<LogGrader>("LogGrader");
	ClassRegistry::RegisterAsset<BranchShape>("BranchShape", { ".bs" });

	ApplicationInfo applicationConfigs;
	applicationConfigs.m_applicationName = "Log Grader";
	std::filesystem::create_directory(std::filesystem::path(".") / "LogGraderProject");
	applicationConfigs.m_projectPath = std::filesystem::absolute(std::filesystem::path(".") / "LogGraderProject" / "Default.eveproj");
	Application::Initialize(applicationConfigs);

	// adjust default camera speed
	const auto editorLayer = Application::GetLayer<EditorLayer>();
	editorLayer->m_velocity = 2.f;
	editorLayer->m_defaultSceneCameraPosition = glm::vec3(1.124, 0.218, 14.089);
	// override default scene camera position etc.
	editorLayer->m_showCameraWindow = false;
	editorLayer->m_showSceneWindow = true;
	editorLayer->m_showEntityExplorerWindow = false;
	editorLayer->m_showEntityInspectorWindow = true;
	editorLayer->GetSceneCamera()->m_useClearColor = true;
	editorLayer->m_defaultSceneCameraPosition = glm::vec3(0, 2.5, 6);
	editorLayer->SetCameraPosition(editorLayer->GetSceneCamera(), editorLayer->m_defaultSceneCameraPosition);
	editorLayer->m_enableGizmos = false;
	editorLayer->GetSceneCamera()->m_clearColor = glm::vec3(1.f);
	const auto renderLayer = Application::GetLayer<RenderLayer>();
	renderLayer->m_enableParticles = false;

	ProjectManager::GetInstance().m_showAssetInspectorWindow = false;
	ProjectManager::GetInstance().m_showProjectWindow = false;
#pragma region Engine Loop
	Application::Start();
	Application::Run();
#pragma endregion
	Application::Terminate();
}

void EngineSetup() {
	ProjectManager::SetActionAfterSceneLoad([=](const std::shared_ptr<Scene>& scene) {
#pragma region Engine Setup
#pragma endregion
		std::vector<Entity> entities;
		scene->GetAllEntities(entities);
		bool found = false;
		for(const auto& entity : entities)
		{
			if(scene->HasPrivateComponent<LogGrader>(entity))
			{
				const auto editorLayer = Application::GetLayer<EditorLayer>();
				editorLayer->SetSelectedEntity(entity);
				editorLayer->SetLockEntitySelection(true);
				found = true;
				break;
			}
		}
		if(!found)
		{
			const auto entity = scene->CreateEntity("LogGrader");
			scene->GetOrSetPrivateComponent<LogGrader>(entity);
			const auto editorLayer = Application::GetLayer<EditorLayer>();
			editorLayer->SetSelectedEntity(entity);
			editorLayer->SetLockEntitySelection(true);
		}
		});
	ProjectManager::SetActionAfterNewScene([=](const std::shared_ptr<Scene>& scene) {
#pragma region Engine Setup
#pragma region Preparations
		Times::SetTimeStep(0.016f);
#pragma endregion
#pragma endregion
		const auto entity = scene->CreateEntity("LogGrader");
		scene->GetOrSetPrivateComponent<LogGrader>(entity);
		const auto editorLayer = Application::GetLayer<EditorLayer>();
		editorLayer->SetSelectedEntity(entity);
		editorLayer->SetLockEntitySelection(true);
		});
}
