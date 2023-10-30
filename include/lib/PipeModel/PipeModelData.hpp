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
		//NodeHandle m_nodeHandle = -1;
		//FlowHandle m_flowHandle = -1;
		CellHandle m_cellHandle = -1;
	};

	typedef PipeGroup<PipeModelPipeGroupData, PipeModelPipeData, PipeModelPipeSegmentData> PipeModelPipeGroup;


	struct PipeCellData
	{
		PipeHandle m_pipeHandle = -1;
		//PipeSegmentHandle m_pipeSegmentHandle = -1;
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
		glm::vec2 m_offset = glm::vec3(0.0f);
	};

	struct PipeModelFlowData
	{
		ParticlePhysics2D<CellParticlePhysicsData> m_startParticlePhysics2D;
		ParticlePhysics2D<CellParticlePhysicsData> m_endParticlePhysics2D;

		std::unordered_map<PipeHandle, ParticleHandle> m_startParticleMap{};
		std::unordered_map<PipeHandle, ParticleHandle> m_endParticleMap{};
	};

	struct PipeModelSkeletonData
	{
		ProfileHandle m_baseProfileHandle = -1;

		std::unordered_map<NodeHandle, NodeHandle> m_nodeMap;
	};

	typedef Skeleton<PipeModelSkeletonData, PipeModelFlowData, PipeModelNodeData> PipeModelSkeleton;



	struct PipeModelPipeGroupData_Old
	{
	};

	struct PipeModelPipeData_Old
	{
	};

	struct PipeModelPipeSegmentData_Old
	{
		NodeHandle m_nodeHandle = -1;
		FlowHandle m_flowHandle = -1;
		CellHandle m_cellHandle = -1;
	};

	typedef PipeGroup<PipeModelPipeGroupData_Old, PipeModelPipeData_Old, PipeModelPipeSegmentData_Old> PipeModelPipeGroup_Old;


	struct PipeCellData_Old
	{
		PipeHandle m_pipeHandle = -1;
		PipeSegmentHandle m_pipeSegmentHandle = -1;
	};

	struct CellParticlePhysicsData_Old
	{
		PipeHandle m_pipeHandle = -1;
	};

	struct PipeProfileData_Old
	{
		NodeHandle m_nodeHandle = -1;
	};

	struct PipeProfileGroupData_Old
	{
		
	};

	typedef PipeProfile<PipeProfileData_Old, PipeCellData_Old> PipeModelPipeProfile_Old;
	typedef PipeProfileGroup<PipeProfileGroupData_Old, PipeProfileData_Old, PipeCellData_Old> PipeModelPipeProfileGroup_Old;
	struct PipeModelNodeData_Old
	{
		int m_endNodeCount = 0;

		glm::quat m_globalEndRotation = {};
		glm::quat m_regulatedGlobalEndRotation = {};
		ProfileHandle m_profileHandle = -1;
		PipeHandle m_pipeHandle = -1;
		NodeHandle m_treeSkeletonNodeHandle = -1;

		//For shifting.
		glm::quat m_localRotation = glm::vec3(0.0f);
		glm::vec2 m_offset = glm::vec3(0.0f);
	};

	struct PipeModelFlowData_Old
	{
		ParticlePhysics2D<CellParticlePhysicsData_Old> m_startParticlePhysics2D;
		ParticlePhysics2D<CellParticlePhysicsData_Old> m_endParticlePhysics2D;

		std::unordered_map<PipeHandle, ParticleHandle> m_startParticleMap{};
		std::unordered_map<PipeHandle, ParticleHandle> m_endParticleMap{};
	};

	struct PipeModelSkeletonData_Old
	{
		ProfileHandle m_baseProfileHandle = -1;

		std::unordered_map<NodeHandle, NodeHandle> m_nodeMap;
	};

	typedef Skeleton<PipeModelSkeletonData_Old, PipeModelFlowData_Old, PipeModelNodeData_Old> PipeModelSkeleton_Old;
}