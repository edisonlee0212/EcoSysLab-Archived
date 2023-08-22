#pragma once
#include "PipeModel.hpp"

using namespace EvoEngine;

namespace EcoSysLab
{
	class PipeModelBase : public IPrivateComponent
	{
	public:
		PipeModel m_pipeModel;
		ProfileHandle m_baseProfileHandle = -1;
		PipeModelParameters m_pipeModelParameters {};
		void Update() override;
		void AssignProfiles();
		
		void EstablishPipes();
		void InitializeStrandRenderer() const;
		void ClearStrands() const;
		void OnCreate() override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}