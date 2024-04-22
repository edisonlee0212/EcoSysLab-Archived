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
		JobDependency CalculateProfiles(const StrandModelParameters& strandModelParameters);
		void CalculateProfile(float maxRootDistance, SkeletonNodeHandle nodeHandle, const StrandModelParameters& strandModelParameters);

		void PackTask(SkeletonNodeHandle nodeHandle, const StrandModelParameters& strandModelParameters);
		void MergeTask(float maxRootDistance, SkeletonNodeHandle nodeHandle, const StrandModelParameters& strandModelParameters);
		void CopyFrontToBackTask(SkeletonNodeHandle nodeHandle);
		void ApplyProfile(const StrandModelParameters& strandModelParameters,
			SkeletonNodeHandle nodeHandle);
		void ApplyProfiles(const StrandModelParameters& strandModelParameters);
		void CalculateStrandProfileAdjustedTransforms(const StrandModelParameters& strandModelParameters);
		[[nodiscard]] glm::vec3 InterpolateStrandSegmentPosition(StrandSegmentHandle strandSegmentHandle, float a) const;
		[[nodiscard]] glm::vec3 InterpolateStrandSegmentAxis(StrandSegmentHandle strandSegmentHandle, float a) const;
		[[nodiscard]] float InterpolateStrandSegmentRadius(StrandSegmentHandle strandSegmentHandle, float a) const;
	};
}