#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "PipeModel.hpp"
#include "LSystemString.hpp"
#include "TreeGraph.hpp"

using namespace UniEngine;
namespace EcoSysLab {
	class RootGrowthParameters {
	public:
		float m_rootNodeGrowthRate;

		/**
		 * \brief How much the soil density affects the growth;
		 */
		float m_environmentalFriction;
		/**
		 * \brief How much will the increase of soil density affects the growth;
		 */
		float m_environmentalFrictionFactor;
		/**
		 * \brief The root node length
		 */
		float m_rootNodeLength;
		
		/**
		 * \brief Thickness of end internode
		 */
		float m_endNodeThickness;
		/**
		 * \brief The thickness accumulation factor
		 */
		float m_thicknessAccumulationFactor;
		/**
		 * \brief The extra thickness gained from node length.
		 */
		float m_thicknessAccumulateAgeFactor;
		/**
		* The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		glm::vec2 m_branchingAngleMeanVariance;
		/**
		* The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_rollAngleMeanVariance;
		/**
		* The mean and variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		glm::vec2 m_apicalAngleMeanVariance;
		/**
		 * \brief Apical control base
		 */
		float m_apicalControl;
		/**
		 * \brief Age influence on apical control
		 */
		float m_apicalControlAgeFactor;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
		/**
		* \brief How much inhibitor will shrink when the tree ages.
		*/
		float m_apicalDominanceAgeFactor;
		/**
		* \brief How much inhibitor will shrink when going through the branch.
		*/
		float m_apicalDominanceDistanceFactor;
		/**
		* The possibility of the lateral branch having different tropism as the parent branch
		*/
		float m_tropismSwitchingProbability;
		/**
		* The distance factor of the possibility of the lateral branch having different tropism as the parent branch
		*/
		float m_tropismSwitchingProbabilityDistanceFactor;
		/**
		* The overall intensity of the tropism.
		*/
		float m_tropismIntensity;
		float m_rootNodeVigorRequirement = 1.0f;
		float m_vigorRequirementAggregateLoss = 1.0f;
		float m_branchingProbability = 1.0f;

		float m_fineRootSegmentLength = 0.02f;
		float m_fineRootApicalAngleVariance = 2.5f;
		float m_fineRootBranchingAngle = 60.f;
		float m_fineRootThickness = 0.002f;
		float m_fineRootMinNodeThickness = 0.05f;
		int m_fineRootNodeCount = 2;
	};

	class ShootGrowthParameters {
	public:
		float m_internodeGrowthRate;
		float m_leafGrowthRate = 0.05f;
		float m_fruitGrowthRate = 0.05f;

#pragma region Bud
		/**
		 * \brief The number of lateral buds an internode contains
		 */
		int m_lateralBudCount;
		/**
		 * \brief The number of fruit buds an internode contains
		 */
		int m_fruitBudCount;
		/**
		 * \brief The number of leaf buds an internode contains
		 */
		int m_leafBudCount;
		/**
		* \brief The mean and variance of the angle between the direction of a lateral bud and its parent shoot.
		*/
		glm::vec2 m_branchingAngleMeanVariance{};
		/**
		* \brief The mean and variance of an angular difference orientation of lateral buds between two internodes
		*/
		glm::vec2 m_rollAngleMeanVariance{};
		/**
		* \brief The mean and variance of the angular difference between the growth direction and the direction of the apical bud
		*/
		glm::vec2 m_apicalAngleMeanVariance{};
		/**
		 * \brief The gravitropism.
		 */
		float m_gravitropism;
		/**
		 * \brief The phototropism
		 */
		float m_phototropism;

		/**
		 * \brief Flushing prob of lateral bud related to the temperature.
		 */
		glm::vec4 m_lateralBudFlushingProbabilityTemperatureRange;
		/**
		 * \brief Flushing prob of leaf bud related to the temperature.
		 */
		glm::vec4 m_leafBudFlushingProbabilityTemperatureRange;
		/**
		 * \brief Flushing prob of fruit bud related to the temperature.
		 */
		glm::vec4 m_fruitBudFlushingProbabilityTemperatureRange;
		/**
		 * \brief The lighting factor for apical bud elongation rate.
		 */
		float m_apicalBudLightingFactor;
		/**
		 * \brief The lighting factor for lateral bud flushing probability.
		 */
		float m_lateralBudLightingFactor;
		/**
		 * \brief The lighting factor for leaf bud flushing probability.
		 */
		float m_leafBudLightingFactor;
		/**
		 * \brief The lighting factor for fruit bud flushing probability.
		 */
		float m_fruitBudLightingFactor;
		/**
		 * \brief Apical control base
		 */
		float m_apicalControl;
		/**
		 * \brief Age influence on apical control
		 */
		float m_apicalControlAgeFactor;
		/**
		* \brief How much inhibitor will an internode generate.
		*/
		float m_apicalDominance;
		/**
		* \brief How much inhibitor will shrink when the tree ages.
		*/
		float m_apicalDominanceAgeFactor;
		/**
		* \brief How much inhibitor will shrink when going through the branch.
		*/
		float m_apicalDominanceDistanceFactor;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_apicalBudExtinctionRate;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_lateralBudExtinctionRate;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_leafBudExtinctionRate;
		/**
		* \brief The probability of internode being removed.
		*/
		float m_fruitBudExtinctionRate;

		/**
		* \brief Productive resource requirement factor for internode elongation
		*/
		float m_internodeVigorRequirement;
		/**
		* \brief Base resource requirement factor for leaf
		*/
		float m_leafVigorRequirement;
		/**
		* \brief Base resource requirement factor for fruit
		*/
		float m_fruitVigorRequirement;

		float m_vigorRequirementAggregateLoss = 1.0f;
#pragma endregion
#pragma region Internode
		/**
		 * \brief The internode length
		 */
		float m_internodeLength;

		/**
		 * \brief Thickness of end internode
		 */
		float m_endNodeThickness;
		/**
		 * \brief The thickness accumulation factor
		 */
		float m_thicknessAccumulationFactor;
		/**
		 * \brief The extra thickness gained from node length.
		 */
		float m_thicknessAccumulateAgeFactor;
		/**
		* \brief The limit of lateral branches being cut off when too close to the
		* root.
		*/
		float m_lowBranchPruning;
		/**
		 * \brief The The impact of the amount of incoming light on the shedding of end internodes.
		 */
		float m_endNodePruningLightFactor;
		/**
		 * \brief The strength of gravity bending.
		 */
		glm::vec3 m_saggingFactorThicknessReductionMax = glm::vec3(0.8f, 1.75f, 1.0f);


#pragma endregion

#pragma region Leaf

		glm::vec3 m_maxLeafSize;
		float m_leafPositionVariance;
		float m_leafRotationVariance;
		float m_leafChlorophyllLoss;
		float m_leafChlorophyllSynthesisFactorTemperature;
		float m_leafFallProbability;

		float m_leafDistanceToBranchEndLimit;
#pragma endregion
#pragma region Fruit

		glm::vec3 m_maxFruitSize;
		float m_fruitPositionVariance;
		float m_fruitRotationVariance;

		float m_fruitFallProbability;
#pragma endregion
	};


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

		ShootGrowthController m_shootGrowthController;
		RootGrowthController m_rootGrowthController;
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
		PipeModel m_pipeModel;
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
		const auto& pipeNodeHandles = pipe.PeekPipeNodeHandles();
		if(pipeNodeHandles.size() < 2) return;

		strands.emplace_back(points.size());
		auto frontPointIndex = points.size();
		StrandPoint point;
		const auto& firstNode = pipeGroup.PeekPipeNode(nodeHandles.front());
		point.m_normal = glm::normalize(firstNode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
		point.m_position = firstNode.m_info.m_globalPosition;
		point.m_thickness = firstNode.m_info.m_thickness;
		point.m_color = pipe.m_info.m_color;

		points.emplace_back(point);
		points.emplace_back(point);

		
		if (pipeNodeHandles.size() == 2)
		{
			const auto& pipeNode = pipeGroup.PeekPipeNode(pipe.PeekPipeNodeHandles()[0]);
			const auto& secondPipeNode = pipeGroup.PeekPipeNode(pipe.PeekPipeNodeHandles()[1]);
			auto distance = glm::distance(pipeNode.m_info.m_globalPosition, secondPipeNode.m_info.m_globalPosition) * 0.25f;
			point.m_normal = glm::normalize(pipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1) * 0.75f + secondPipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1) * 0.25f);
			point.m_position = pipeNode.m_info.m_globalPosition + pipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1) * distance;
			point.m_thickness = pipeNode.m_info.m_thickness * 0.75f + secondPipeNode.m_info.m_thickness * 0.25f;
			points.emplace_back(point);

			point.m_normal = glm::normalize(pipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1) * 0.25f + secondPipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1) * 0.75f);
			point.m_position = secondPipeNode.m_info.m_globalPosition + secondPipeNode.m_info.m_globalRotation * glm::vec3(0, 0, 1) * distance;
			point.m_thickness = pipeNode.m_info.m_thickness * 0.25f + secondPipeNode.m_info.m_thickness * 0.75f;
			points.emplace_back(point);
		}else if(pipeNodeHandles.size() == 3)
		{
			const auto& pipeNode = pipeGroup.PeekPipeNode(pipe.PeekPipeNodeHandles()[0]);
			const auto& secondPipeNode = pipeGroup.PeekPipeNode(pipe.PeekPipeNodeHandles()[1]);
			auto distance = glm::distance(pipeNode.m_info.m_globalPosition, secondPipeNode.m_info.m_globalPosition) * 0.5f;
			point.m_normal = glm::normalize(pipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1) + secondPipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeNode.m_info.m_globalPosition + secondPipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1) * distance;
			point.m_thickness = pipeNode.m_info.m_thickness * 0.5f + secondPipeNode.m_info.m_thickness * 0.5f;
			points.emplace_back(point);
		}

		for (int i = 1; i < pipeNodeHandles.size(); i++)
		{
			const auto& pipeNode = pipeGroup.PeekPipeNode(pipeNodeHandles[i]);
			point.m_normal = glm::normalize(pipeNode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeNode.m_info.m_globalPosition;
			point.m_thickness = pipeNode.m_info.m_thickness;
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
		
	}
}
