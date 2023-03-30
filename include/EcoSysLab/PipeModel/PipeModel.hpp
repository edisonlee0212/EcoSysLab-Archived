#pragma once
#include "PipeModelParameters.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	class PipeModel
	{
	public:
		PipeModelSkeleton m_shootSkeleton;
		PipeModelSkeleton m_rootSkeleton;
		void CalculateGraph(const PipeModelParameters& pipeModelParameters);
	};
}