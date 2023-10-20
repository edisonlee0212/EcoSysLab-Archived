#pragma once
#include "Skeleton.hpp"
#include "PipeGroup.hpp"
#include "HexagonProfileData.hpp"

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
		NodeHandle m_nodeHandle = -1;
		FlowHandle m_flowHandle = -1;
		CellHandle m_cellHandle = -1;
	};

	typedef PipeGroup<PipeModelPipeGroupData, PipeModelPipeData, PipeModelPipeSegmentData> PipeModelPipeGroup;


	struct PipeCellData
	{
		PipeHandle m_pipeHandle = -1;
		PipeSegmentHandle m_pipeSegmentHandle = -1;
	};

	struct CellParticlePhysicsData
	{
		PipeHandle m_pipeHandle = -1;
	};

	struct PipeProfileData
	{
		NodeHandle m_nodeHandle = -1;
	};

	struct PipeProfileGroupData
	{
		
	};

	typedef PipeProfile<PipeProfileData, PipeCellData> PipeModelPipeProfile;
	typedef PipeProfileGroup<PipeProfileGroupData, PipeProfileData, PipeCellData> PipeModelPipeProfileGroup;
	struct PipeModelNodeData
	{
		int m_endNodeCount = 0;

		glm::quat m_globalEndRotation = {};
		glm::quat m_regulatedGlobalEndRotation = {};
		ProfileHandle m_profileHandle = -1;
		PipeHandle m_pipeHandle = -1;
		NodeHandle m_treeSkeletonNodeHandle = -1;

		//For shifting.
		glm::quat m_localRotation = glm::vec3(0.0f);
		glm::vec3 m_localPosition = glm::vec3(0.0f);
	};

	struct PipeModelFlowData
	{
		ParticlePhysics2D<CellParticlePhysicsData> m_startParticlePhysics2D;
		ParticlePhysics2D<CellParticlePhysicsData> m_endParticlePhysics2D;
		glm::vec2 m_offset = glm::vec2(0.0f);
	};

	struct PipeModelSkeletonData
	{
		ProfileHandle m_baseProfileHandle = -1;

		std::unordered_map<NodeHandle, NodeHandle> m_nodeMap;
	};

	typedef Skeleton<PipeModelSkeletonData, PipeModelFlowData, PipeModelNodeData> PipeModelSkeleton;
}