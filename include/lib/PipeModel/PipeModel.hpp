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
		void MapProfiles(ProfileHandle srcProfileHandle, const std::vector<ProfileHandle>& dstProfileHandles);
	public:
		PipeModelPipeGroup m_pipeGroup;
		PipeModelPipeProfileGroup m_pipeProfileGroup;
		PipeModelSkeleton m_skeleton;
		void CalculatePipeSegmentInfos(const PipeModelParameters& pipeModelParameters);
		void InitializePipes(const PipeModelParameters& pipeModelParameters);
	};
}
