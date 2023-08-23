#pragma once

#pragma once

#include "PipeModelData.hpp"
#include "TreeVisualizer.hpp"
#include "PipeModelBase.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class SinglePipeProfile : public IPrivateComponent
	{
	public:
		void OnCreate() override;
		void OnDestroy() override;
		bool m_showProfile = true;
		void Update() override;

		PrivateComponentRef m_pipeModelBase;
		ProfileHandle m_profileHandle = -1;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
	};
}