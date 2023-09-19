#pragma once
#include "Skeleton.hpp"
#include "VigorSink.hpp"
#include "TreeIlluminationEstimator.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct SpaceColonizationNodeGrowthData
	{
		
	};
	struct SpaceColonizationStemGrowthData
	{

	};
	struct SpaceColonizationTreeGrowthData
	{

	};
	typedef Skeleton<SpaceColonizationTreeGrowthData, SpaceColonizationStemGrowthData, SpaceColonizationNodeGrowthData> SpaceColonizationTreeSkeleton;
}