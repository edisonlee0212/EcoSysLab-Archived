#pragma once
#include "PipeModelParameters.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	class PipeModel
	{
		static void CalculatePipeTransforms(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters);
		void DistributePipes(bool isShoot, const PipeModelParameters& pipeModelParameters);

		void SplitPipes(PipeModelSkeleton& targetSkeleton, NodeHandle nodeHandle, HexagonGridHandle newGridHandle) const;
	public:
		std::unordered_map<NodeHandle, NodeHandle> m_shootSkeletonLinks;
		std::unordered_map<NodeHandle, NodeHandle> m_rootSkeletonLinks;
		PipeModelBaseHexagonGrid m_baseGrid;
		PipeModelSkeleton m_shootSkeleton;
		PipeModelSkeleton m_rootSkeleton;
		void BuildGraph(const PipeModelParameters& pipeModelParameters);
	};
}
