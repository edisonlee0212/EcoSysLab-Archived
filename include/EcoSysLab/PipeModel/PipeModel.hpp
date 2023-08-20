#pragma once
#include "PipeModelParameters.hpp"
#include "PipeProfile.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct CellSortSettings
	{
		bool m_flatCut = false;
	};
	struct CellSqueezeSettings
	{
		
	};
	class PipeModel
	{
	public:
		PipeModelPipeGroup m_pipeGroup;
	};
}
