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
		void UpdatePipeModels(const TreeModel& targetTreeModel, const PipeModelParameters& pipeModelParameters);
		void SimulateAllProfiles(size_t iteration, const PipeModelParameters& pipeModelParameters);
		void ApplySimulationResults(const PipeModelParameters& pipeModelParameters);
	};
}