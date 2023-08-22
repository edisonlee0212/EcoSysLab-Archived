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
		bool m_showProfile = false;
		void Update() override;

		PrivateComponentRef m_pipeModelBase;
		ProfileHandle m_profileHandle = -1;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}