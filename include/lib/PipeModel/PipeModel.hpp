#pragma once
#include "PipeModelParameters.hpp"
#include "PipeProfile.hpp"
#include "PipeModelData.hpp"
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
		PipeModelPipeGroup_Old m_pipeGroup;
		PipeModelPipeProfileGroup_Old m_pipeProfileGroup;
		PipeModelSkeleton_Old m_skeleton;
		void CalculatePipeSegmentInfos(const PipeModelParameters& pipeModelParameters);
	};
}
