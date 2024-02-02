#pragma once
#include "TreeGrowthData.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{
	class StrandModel
	{
	public:
		StrandModelSkeleton m_strandModelSkeleton;
		void ResetAllProfiles(const StrandModelParameters& strandModelParameters);
		void InitializeProfiles(const StrandModelParameters& strandModelParameters);
		void CalculateProfiles(const StrandModelParameters& strandModelParameters);
		void CalculateProfile(float maxRootDistance, NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters, bool scheduling);
		void Wait(NodeHandle nodeHandle);

		void PackTask(NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters, bool parallel);
		void MergeTask(float maxRootDistance, NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters);
		void CopyFrontToBackTask(NodeHandle nodeHandle);
		void CalculateShiftTask(NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters);
		void ApplyProfile(const StrandModelParameters& strandModelParameters,
			NodeHandle nodeHandle);
		void ApplyProfiles(const StrandModelParameters& strandModelParameters);
		void CalculateStrandProfileAdjustedTransforms(const StrandModelParameters& strandModelParameters);
		[[nodiscard]] glm::vec3 InterpolateStrandSegmentPosition(StrandSegmentHandle strandSegmentHandle, float a) const;
		[[nodiscard]] glm::vec3 InterpolateStrandSegmentAxis(StrandSegmentHandle strandSegmentHandle, float a) const;
		[[nodiscard]] float InterpolateStrandSegmentRadius(StrandSegmentHandle strandSegmentHandle, float a) const;
	};
}