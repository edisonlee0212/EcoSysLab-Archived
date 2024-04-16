#pragma once
#include "Skeleton.hpp"
#include "StrandGroup.hpp"

#include "StrandModelProfile.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct StrandModelStrandGroupData
	{
	};

	struct StrandModelStrandData
	{
	};

	struct StrandModelStrandSegmentData
	{
		/**
		 * \brief The handle of the internode this pipe segment belongs to. Pipe -> PipeSegment <-> Cell <- Profile <- Internode
		 */
		SkeletonNodeHandle m_nodeHandle = -1;
		ParticleHandle m_profileParticleHandle = -1;
	};

	typedef StrandGroup<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData> StrandModelStrandGroup;


	struct CellParticlePhysicsData
	{
	};
}