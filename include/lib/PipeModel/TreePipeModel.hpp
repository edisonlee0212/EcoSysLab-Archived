#pragma once

#include "TreeModel.hpp"
#include "PipeModel.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class TreePipeModel
	{
	public:
		PipeModel m_shootPipeModel{};
		PipeModel m_rootPipeModel{};
		PipeModelParameters m_pipeModelParameters{};
		void UpdatePipeModels(const TreeModel& targetTreeModel);
	};
}