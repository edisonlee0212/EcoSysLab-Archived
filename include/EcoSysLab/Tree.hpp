#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"

#include "LSystemString.hpp"
#include "TreeGraph.hpp"

using namespace UniEngine;
namespace EcoSysLab {

	class TreeDescriptor : public IAsset {
	public:
		ShootGrowthParameters m_shootGrowthParameters;
		RootGrowthParameters m_rootGrowthParameters;
		void OnCreate() override;

		void OnInspect() override;

		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;
	};

	class Tree : public IPrivateComponent {
		friend class EcoSysLabLayer;
		bool TryGrow(float deltaTime);
		template<typename PipeGroupData, typename PipeData, typename PipeNodeData>
		void BuildStrand(const PipeGroup<PipeGroupData, PipeData, PipeNodeData>& pipeGroup, const Pipe<PipeData>& pipe, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points) const;


	public:
		template<typename PipeGroupData, typename PipeData, typename PipeNodeData>
		void BuildStrands(const PipeGroup<PipeGroupData, PipeData, PipeNodeData>& pipeGroup, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points) const;

		void InitializeStrandRenderer() const;

		void Serialize(YAML::Emitter& out) override;
		bool m_splitRootTest = true;
		bool m_recordBiomassHistory = true;
		float m_leftSideBiomass;
		float m_rightSideBiomass;

		TreeMeshGeneratorSettings m_meshGeneratorSettings;
		int m_temporalProgressionIteration = 0;
		bool m_temporalProgression = false;
		void Update() override;

		void Deserialize(const YAML::Node& in) override;

		std::vector<float> m_rootBiomassHistory;
		std::vector<float> m_shootBiomassHistory;

		PrivateComponentRef m_soil;
		PrivateComponentRef m_climate;
		AssetRef m_treeDescriptor;
		bool m_enableHistory = false;
		int m_historyIteration = 30;
		TreeModel m_treeModel;
		void OnInspect() override;

		void OnDestroy() override;

		void OnCreate() override;

		void ClearMeshes();

		void GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration = -1);

		void FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString);
		void FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph);
		void FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2);
	};

	template <typename PipeGroupData, typename PipeData, typename PipeNodeData>
	void Tree::BuildStrand(const PipeGroup<PipeGroupData, PipeData, PipeNodeData>& pipeGroup,
		const Pipe<PipeData>& pipe, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points) const
	{
		const auto& nodeHandles = pipe.PeekPipeNodeHandles();
		if (nodeHandles.empty()) return;
		strands.emplace_back(points.size());
		auto frontPointIndex = points.size();
		StrandPoint point;
		const auto& firstNode = pipeGroup.PeekPipeNode(nodeHandles.front());
		point.m_normal = glm::normalize(firstNode.m_info.m_globalStartRotation * glm::vec3(0, 0, -1));
		point.m_position = firstNode.m_info.m_globalStartPosition;
		point.m_thickness = firstNode.m_info.m_startThickness;
		point.m_color = glm::vec4(glm::linearRand(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1)), 1.0f);

		points.emplace_back(point);
		points.emplace_back(point);

		const auto& pipeNodeHandles = pipe.PeekPipeNodeHandles();
		if (pipeNodeHandles.size() < 3)
		{
			const auto& pipeNode = pipeGroup.PeekPipeNode(pipe.PeekPipeNodeHandles()[0]);
			auto distance = glm::distance(pipeNode.m_info.m_globalStartPosition, pipeNode.m_info.m_globalEndPosition) * 0.25f;
			point.m_normal = glm::normalize(pipeNode.m_info.m_globalStartRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeNode.m_info.m_globalStartPosition + pipeNode.m_info.m_globalStartRotation * glm::vec3(0, 0, -1) * distance;
			point.m_thickness = pipeNode.m_info.m_startThickness * 0.75f + pipeNode.m_info.m_endThickness * 0.25f;
			points.emplace_back(point);

			point.m_normal = glm::normalize(pipeNode.m_info.m_globalStartRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeNode.m_info.m_globalEndPosition + pipeNode.m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance;
			point.m_thickness = pipeNode.m_info.m_startThickness * 0.25f + pipeNode.m_info.m_endThickness * 0.75f;
			points.emplace_back(point);
		}

		for (int i = 0; i < pipeNodeHandles.size(); i++)
		{
			const auto& pipeNode = pipeGroup.PeekPipeNode(pipeNodeHandles[i]);
			point.m_normal = glm::normalize(pipeNode.m_info.m_globalEndRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeNode.m_info.m_globalEndPosition;
			point.m_thickness = pipeNode.m_info.m_endThickness;
			points.emplace_back(point);
		}

		StrandPoint frontPoint;
		frontPoint = points.at(frontPointIndex);
		frontPoint.m_position = 2.0f * frontPoint.m_position - points.at(frontPointIndex + 2).m_position;
		frontPoint.m_normal = 2.0f * frontPoint.m_normal - points.at(frontPointIndex + 2).m_normal;
		frontPoint.m_thickness = 2.0f * frontPoint.m_thickness - points.at(frontPointIndex + 2).m_thickness;
		points.at(frontPointIndex) = frontPoint;

		StrandPoint backPoint;
		backPoint = points.at(points.size() - 2);
		backPoint.m_position = 2.0f * points.at(points.size() - 1).m_position - backPoint.m_position;
		backPoint.m_normal = 2.0f * points.at(points.size() - 1).m_normal - backPoint.m_normal;
		backPoint.m_thickness = 2.0f * points.at(points.size() - 1).m_thickness - backPoint.m_thickness;
		points.emplace_back(backPoint);

	}

	template <typename PipeGroupData, typename PipeData, typename PipeNodeData>
	void Tree::BuildStrands(const PipeGroup<PipeGroupData, PipeData, PipeNodeData>& pipeGroup, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points) const
	{
		for (const auto& pipe : pipeGroup.PeekPipes())
		{
			if (pipe.IsRecycled()) continue;
			BuildStrand(pipeGroup, pipe, strands, points);
		}
		if(!points.empty()) strands.emplace_back(points.size());
	}
}
