#pragma once
#include "PipeModelParameters.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	class PipeModel
	{
		static void CalculatePipeLocalPositions(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters);
		static void CalculatePipeTransforms(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters);
		void DistributePipes(bool isShoot, const PipeModelParameters& pipeModelParameters);

		void SplitPipes(std::unordered_map<NodeHandle, HexagonGridHandle>& gridHandleMap, PipeModelHexagonGridGroup& gridGroup, 
			PipeModelSkeleton& targetSkeleton, NodeHandle nodeHandle, HexagonGridHandle newGridHandle, const PipeModelParameters& pipeModelParameters) const;
	public:
		std::unordered_map<NodeHandle, NodeHandle> m_shootSkeletonLinks;
		std::unordered_map<NodeHandle, NodeHandle> m_rootSkeletonLinks;
		PipeModelBaseHexagonGrid m_baseGrid;
		PipeModelSkeleton m_shootSkeleton;
		PipeModelSkeleton m_rootSkeleton;
		void BuildGraph(const PipeModelParameters& pipeModelParameters);
	};
}
