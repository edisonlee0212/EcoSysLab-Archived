#pragma once
#include "Skeleton.hpp"
#include "PipeGroup.hpp"

#include "ParticlePhysics2D.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelPipeGroupData
	{
	};

	struct PipeModelPipeData
	{
	};

	struct PipeModelPipeSegmentData
	{
	};

	typedef PipeGroup<PipeModelPipeGroupData, PipeModelPipeData, PipeModelPipeSegmentData> PipeModelPipeGroup;


	struct CellParticlePhysicsData
	{
		PipeHandle m_pipeHandle = -1;
		PipeSegmentHandle m_pipeSegmentHandle = -1;
		bool m_mainChild = false;
		bool m_base = false;
	};
}