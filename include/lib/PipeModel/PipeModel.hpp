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
		PipeModelPipeGroup m_pipeGroup;
		PipeModelPipeProfileGroup m_pipeProfileGroup;
		PipeModelSkeleton m_skeleton;
		void CalculatePipeSegmentInfos(const PipeModelParameters& pipeModelParameters);
	};
}
