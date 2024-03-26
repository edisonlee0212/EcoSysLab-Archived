#pragma once
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "RadialBoundingVolume.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	enum class ShootGrowthParameterType
	{
		GrowthRate,
		BranchingAngleMean,
		BranchingAngleVariance,
		RollAngleMean,
		RollAngleVariance,
		ApicalAngleMean,
		ApicalAngleVariance,
		Gravitropism,
		Phototropism,
		Sagging,
		SaggingThicknessFactor,
		MaxSagging,
		InternodeLength,
		InternodeLengthThicknessFactor,
		EndNodeThickness,
		ThicknessAccumulationFactor,
		ThicknessAgeFactor,
		ShadowFactor,

		ApicalBudExtinctionRate,
		LateralBudExtinctionRate,
		ApicalBudLightingFactor,
		LateralBudLightingFactor,
		ApicalControl,
		ApicalDominance,
		ApicalDominanceLoss,

		LowBranchPruning,
		LowBranchPruningThicknessFactor,
		LightPruningFactor,
		ThicknessPruningFactor

	};

	struct ShootGrowthParameterOffset
	{
		unsigned m_type = static_cast<unsigned>(ShootGrowthParameterType::GrowthRate);
		glm::vec2 m_range = { 0.0f, 0.0f };
		Curve2D m_offset;
	};

	class ShootDescriptorGenerator : public IAsset
	{
		ShootDescriptor m_baseShootDescriptor {};
		std::vector<ShootGrowthParameterOffset> m_shootDescriptorOffsets;
	public:
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		static void ApplyOffsets(ShootDescriptor& treeGrowthParameters, const std::vector<ShootGrowthParameterOffset>& offsets);
		static bool OnInspectShootGrowthParametersOffset(std::vector<ShootGrowthParameterOffset>& shootGrowthParameterOffsets);
	};
}