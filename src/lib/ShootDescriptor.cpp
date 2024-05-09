#include "ShootDescriptor.hpp"

using namespace EcoSysLab;

void ShootDescriptor::PrepareController(ShootGrowthController& shootGrowthController) const
{
	shootGrowthController.m_baseInternodeCount = m_baseInternodeCount;
	shootGrowthController.m_breakingForce = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			if (m_branchStrength != 0.f && !internode.IsEndNode() && internode.m_info.m_thickness != 0.f && internode.m_info.m_length != 0.f) {
				float branchWaterFactor = 1.f;
				if (m_branchStrengthLightingThreshold != 0.f && internode.m_data.m_maxDescendantLightIntensity < m_branchStrengthLightingThreshold)
				{
					branchWaterFactor = 1.f - m_branchStrengthLightingLoss;
				}

				return glm::pow(internode.m_info.m_thickness / m_endNodeThickness, m_branchStrengthThicknessFactor) * branchWaterFactor * internode.m_data.m_strength * m_branchStrength;
			}
			return FLT_MAX;
		};
	shootGrowthController.m_sagging = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			float strength = m_endNodeThickness * internode.m_data.m_saggingForce * m_gravityBendingStrength / glm::pow(internode.m_info.m_thickness / m_endNodeThickness, m_gravityBendingThicknessFactor);
			strength = m_gravityBendingMax * (1.f - glm::exp(-glm::abs(strength)));
			return glm::max(internode.m_data.m_sagging, strength);
		};
	shootGrowthController.m_baseNodeApicalAngle = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return glm::gaussRand(m_baseNodeApicalAngleMeanVariance.x, m_baseNodeApicalAngleMeanVariance.y);
		};

	shootGrowthController.m_internodeGrowthRate = m_growthRate / m_internodeLength;

	shootGrowthController.m_branchingAngle = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			float value = glm::gaussRand(m_branchingAngleMeanVariance.x, m_branchingAngleMeanVariance.y);
		/*
			if(const auto noise = m_branchingAngle.Get<ProceduralNoise2D>())
			{
				noise->Process(glm::vec2(internode.GetHandle(), internode.m_info.m_rootDistance), value);
			}*/
			return value;
		};
	shootGrowthController.m_rollAngle = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			float value = glm::gaussRand(m_rollAngleMeanVariance.x, m_rollAngleMeanVariance.y);
		/*
			if (const auto noise = m_rollAngle.Get<ProceduralNoise2D>())
			{
				noise->Process(glm::vec2(internode.GetHandle(), internode.m_info.m_rootDistance), value);
			}*/
			value += m_rollAngleNoise2D.GetValue(glm::vec2(internode.GetHandle(), internode.m_info.m_rootDistance));
			return value;
		};
	shootGrowthController.m_apicalAngle = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			if (m_straightTrunk != 0.f && internode.m_data.m_order == 0 && internode.m_info.m_rootDistance < m_straightTrunk) return 0.f;
			float value = glm::gaussRand(m_apicalAngleMeanVariance.x, m_apicalAngleMeanVariance.y);
		/*
			if (const auto noise = m_apicalAngle.Get<ProceduralNoise2D>())
			{
				noise->Process(glm::vec2(internode.GetHandle(), internode.m_info.m_rootDistance), value);
			}*/
			value += m_apicalAngleNoise2D.GetValue(glm::vec2(internode.GetHandle(), internode.m_info.m_rootDistance));
			return value;
		};
	shootGrowthController.m_gravitropism = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return m_gravitropism;
		};
	shootGrowthController.m_phototropism = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return m_phototropism;
		};
	shootGrowthController.m_horizontalTropism = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return m_horizontalTropism;
		};
	

	shootGrowthController.m_internodeStrength = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return 1.f;
		};

	shootGrowthController.m_internodeLength = m_internodeLength;
	shootGrowthController.m_internodeLengthThicknessFactor = m_internodeLengthThicknessFactor;
	shootGrowthController.m_endNodeThickness = m_endNodeThickness;
	shootGrowthController.m_thicknessAccumulationFactor = m_thicknessAccumulationFactor;
	shootGrowthController.m_thicknessAgeFactor = m_thicknessAgeFactor;
	shootGrowthController.m_internodeShadowFactor = m_internodeShadowFactor;

	shootGrowthController.m_lateralBudCount = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			if(m_maxOrder == -1 || internode.m_data.m_order < m_maxOrder)
			{
				return m_lateralBudCount;
			}
			return 0;
		};
	shootGrowthController.m_apicalBudExtinctionRate = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			if (internode.m_info.m_rootDistance < 0.5f) return 0.f;
			return m_apicalBudExtinctionRate;
		};
	shootGrowthController.m_lateralBudFlushingRate = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			float flushingRate = m_lateralBudFlushingRate;
			if (internode.m_data.m_inhibitorSink > 0.0f) flushingRate *= glm::exp(-internode.m_data.m_inhibitorSink);
			return flushingRate;
		};
	shootGrowthController.m_apicalControl = m_apicalControl;
	shootGrowthController.m_rootDistanceControl = m_rootDistanceControl;
	shootGrowthController.m_heightControl = m_heightControl;

	shootGrowthController.m_apicalDominance = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return m_apicalDominance * internode.m_data.m_lightIntensity;
		};
	shootGrowthController.m_apicalDominanceLoss = m_apicalDominanceLoss;
	shootGrowthController.m_leaf = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return internode.m_data.m_lightIntensity > m_leafFlushingLightingRequirement;
		};

	shootGrowthController.m_leafFallProbability = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return m_leafFallProbability;
		};
	shootGrowthController.m_fruit = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return internode.m_data.m_lightIntensity > m_fruitFlushingLightingRequirement;
		};
	shootGrowthController.m_fruitFallProbability = [&](const SkeletonNode<InternodeGrowthData>& internode)
		{
			return m_fruitFallProbability;
		};


	
}

void ShootDescriptor::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_baseInternodeCount" << YAML::Value << m_baseInternodeCount;
	out << YAML::Key << "m_straightTrunk" << YAML::Value << m_straightTrunk;
	out << YAML::Key << "m_baseNodeApicalAngleMeanVariance" << YAML::Value << m_baseNodeApicalAngleMeanVariance;

	out << YAML::Key << "m_growthRate" << YAML::Value << m_growthRate;
	m_rollAngle.Save("m_rollAngle", out);
	m_apicalAngle.Save("m_apicalAngle", out);

	m_rollAngleNoise2D.Save("m_rollAngleNoise2D", out);
	m_apicalAngleNoise2D.Save("m_apicalAngleNoise2D", out);

	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << m_gravitropism;
	out << YAML::Key << "m_phototropism" << YAML::Value << m_phototropism;
	out << YAML::Key << "m_horizontalTropism" << YAML::Value << m_horizontalTropism;
	out << YAML::Key << "m_gravityBendingStrength" << YAML::Value << m_gravityBendingStrength;
	out << YAML::Key << "m_gravityBendingThicknessFactor" << YAML::Value << m_gravityBendingThicknessFactor;
	out << YAML::Key << "m_gravityBendingMax" << YAML::Value << m_gravityBendingMax;

	out << YAML::Key << "m_internodeLength" << YAML::Value << m_internodeLength;
	out << YAML::Key << "m_internodeLengthThicknessFactor" << YAML::Value << m_internodeLengthThicknessFactor;
	out << YAML::Key << "m_endNodeThickness" << YAML::Value << m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAgeFactor" << YAML::Value << m_thicknessAgeFactor;
	out << YAML::Key << "m_internodeShadowFactor" << YAML::Value << m_internodeShadowFactor;

	out << YAML::Key << "m_lateralBudCount" << YAML::Value << m_lateralBudCount;
	out << YAML::Key << "m_maxOrder" << YAML::Value << m_maxOrder;
	out << YAML::Key << "m_apicalBudExtinctionRate" << YAML::Value << m_apicalBudExtinctionRate;
	out << YAML::Key << "m_lateralBudFlushingRate" << YAML::Value << m_lateralBudFlushingRate;
	out << YAML::Key << "m_apicalControl" << YAML::Value << m_apicalControl;
	out << YAML::Key << "m_heightControl" << YAML::Value << m_heightControl;
	out << YAML::Key << "m_rootDistanceControl" << YAML::Value << m_rootDistanceControl;

	out << YAML::Key << "m_apicalDominance" << YAML::Value << m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceLoss" << YAML::Value << m_apicalDominanceLoss;

	out << YAML::Key << "m_trunkProtection" << YAML::Value << m_trunkProtection;
	out << YAML::Key << "m_maxFlowLength" << YAML::Value << m_maxFlowLength;
	out << YAML::Key << "m_lightPruningFactor" << YAML::Value << m_lightPruningFactor;
	out << YAML::Key << "m_branchStrength" << YAML::Value << m_branchStrength;
	out << YAML::Key << "m_branchStrengthThicknessFactor" << YAML::Value << m_branchStrengthThicknessFactor;
	out << YAML::Key << "m_branchStrengthLightingThreshold" << YAML::Value << m_branchStrengthLightingThreshold;
	out << YAML::Key << "m_branchStrengthLightingLoss" << YAML::Value << m_branchStrengthLightingLoss;
	out << YAML::Key << "m_branchBreakingFactor" << YAML::Value << m_branchBreakingFactor;
	out << YAML::Key << "m_branchBreakingMultiplier" << YAML::Value << m_branchBreakingMultiplier;

	out << YAML::Key << "m_leafFlushingLightingRequirement" << YAML::Value << m_leafFlushingLightingRequirement;
	out << YAML::Key << "m_leafFallProbability" << YAML::Value << m_leafFallProbability;
	out << YAML::Key << "m_leafDistanceToBranchEndLimit" << YAML::Value << m_leafDistanceToBranchEndLimit;

	out << YAML::Key << "m_fruitFlushingLightingRequirement" << YAML::Value << m_fruitFlushingLightingRequirement;
	out << YAML::Key << "m_fruitFallProbability" << YAML::Value << m_fruitFallProbability;
	out << YAML::Key << "m_fruitDistanceToBranchEndLimit" << YAML::Value << m_fruitDistanceToBranchEndLimit;

	m_barkMaterial.Save("m_barkMaterial", out);
}

void ShootDescriptor::Deserialize(const YAML::Node& in)
{
	if (in["m_baseInternodeCount"]) m_baseInternodeCount = in["m_baseInternodeCount"].as<int>();
	if (in["m_straightTrunk"]) m_straightTrunk = in["m_straightTrunk"].as<float>();
	if (in["m_baseNodeApicalAngleMeanVariance"]) m_baseNodeApicalAngleMeanVariance = in["m_baseNodeApicalAngleMeanVariance"].as<glm::vec2>();

	if (in["m_growthRate"]) m_growthRate = in["m_growthRate"].as<float>();
	m_rollAngle.Load("m_rollAngle", in);
	m_apicalAngle.Load("m_apicalAngle", in);

	m_rollAngleNoise2D.Load("m_rollAngleNoise2D", in);
	m_apicalAngleNoise2D.Load("m_apicalAngleNoise2D", in);

	if (in["m_branchingAngleMeanVariance"]) m_branchingAngleMeanVariance = in["m_branchingAngleMeanVariance"].as<glm::vec2>();
	if (in["m_rollAngleMeanVariance"]) m_rollAngleMeanVariance = in["m_rollAngleMeanVariance"].as<glm::vec2>();
	if (in["m_apicalAngleMeanVariance"]) m_apicalAngleMeanVariance = in["m_apicalAngleMeanVariance"].as<glm::vec2>();
	if (in["m_gravitropism"]) m_gravitropism = in["m_gravitropism"].as<float>();
	if (in["m_phototropism"]) m_phototropism = in["m_phototropism"].as<float>();
	if (in["m_horizontalTropism"]) m_horizontalTropism = in["m_horizontalTropism"].as<float>();
	if (in["m_gravityBendingStrength"]) m_gravityBendingStrength = in["m_gravityBendingStrength"].as<float>();
	if (in["m_gravityBendingThicknessFactor"]) m_gravityBendingThicknessFactor = in["m_gravityBendingThicknessFactor"].as<float>();
	if (in["m_gravityBendingMax"]) m_gravityBendingMax = in["m_gravityBendingMax"].as<float>();

	if (in["m_internodeLength"]) m_internodeLength = in["m_internodeLength"].as<float>();
	if (in["m_internodeLengthThicknessFactor"]) m_internodeLengthThicknessFactor = in["m_internodeLengthThicknessFactor"].as<float>();
	if (in["m_endNodeThickness"]) m_endNodeThickness = in["m_endNodeThickness"].as<float>();
	if (in["m_thicknessAccumulationFactor"]) m_thicknessAccumulationFactor = in["m_thicknessAccumulationFactor"].as<float>();
	if (in["m_thicknessAgeFactor"]) m_thicknessAgeFactor = in["m_thicknessAgeFactor"].as<float>();
	if (in["m_internodeShadowFactor"]) m_internodeShadowFactor = in["m_internodeShadowFactor"].as<float>();


	if (in["m_lateralBudCount"]) m_lateralBudCount = in["m_lateralBudCount"].as<int>();
	if (in["m_maxOrder"]) m_maxOrder = in["m_maxOrder"].as<int>();
	if (in["m_apicalBudExtinctionRate"]) m_apicalBudExtinctionRate = in["m_apicalBudExtinctionRate"].as<float>();
	if (in["m_lateralBudFlushingRate"]) m_lateralBudFlushingRate = in["m_lateralBudFlushingRate"].as<float>();
	if (in["m_apicalControl"]) m_apicalControl = in["m_apicalControl"].as<float>();
	if (in["m_rootDistanceControl"]) m_rootDistanceControl = in["m_rootDistanceControl"].as<float>();
	if (in["m_heightControl"]) m_heightControl = in["m_heightControl"].as<float>();

	if (in["m_apicalDominance"]) m_apicalDominance = in["m_apicalDominance"].as<float>();
	if (in["m_apicalDominanceLoss"]) m_apicalDominanceLoss = in["m_apicalDominanceLoss"].as<float>();

	if (in["m_trunkProtection"]) m_trunkProtection = in["m_trunkProtection"].as<bool>();
	if (in["m_maxFlowLength"]) m_maxFlowLength = in["m_maxFlowLength"].as<int>();
	if (in["m_lightPruningFactor"]) m_lightPruningFactor = in["m_lightPruningFactor"].as<float>();
	if (in["m_branchStrength"]) m_branchStrength = in["m_branchStrength"].as<float>();
	if (in["m_branchStrengthThicknessFactor"]) m_branchStrengthThicknessFactor = in["m_branchStrengthThicknessFactor"].as<float>();
	if (in["m_branchStrengthLightingThreshold"]) m_branchStrengthLightingThreshold = in["m_branchStrengthLightingThreshold"].as<float>();
	if (in["m_branchStrengthLightingLoss"]) m_branchStrengthLightingLoss = in["m_branchStrengthLightingLoss"].as<float>();
	if (in["m_branchBreakingFactor"]) m_branchBreakingFactor = in["m_branchBreakingFactor"].as<float>();
	if (in["m_branchBreakingMultiplier"]) m_branchBreakingMultiplier = in["m_branchBreakingMultiplier"].as<float>();

	if (in["m_leafFlushingLightingRequirement"]) m_leafFlushingLightingRequirement = in["m_leafFlushingLightingRequirement"].as<float>();
	if (in["m_leafFallProbability"]) m_leafFallProbability = in["m_leafFallProbability"].as<float>();
	if (in["m_leafDistanceToBranchEndLimit"]) m_leafDistanceToBranchEndLimit = in["m_leafDistanceToBranchEndLimit"].as<float>();

	//Structure
	if (in["m_fruitFlushingLightingRequirement"]) m_fruitFlushingLightingRequirement = in["m_fruitFlushingLightingRequirement"].as<float>();
	if (in["m_fruitFallProbability"]) m_fruitFallProbability = in["m_fruitFallProbability"].as<float>();
	if (in["m_fruitDistanceToBranchEndLimit"]) m_fruitDistanceToBranchEndLimit = in["m_fruitDistanceToBranchEndLimit"].as<float>();

	m_barkMaterial.Load("m_barkMaterial", in);
}

void ShootDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	changed = ImGui::DragFloat("Growth rate", &m_growthRate, 0.01f, 0.0f, 10.0f) || changed;
	changed = ImGui::DragFloat("Straight Trunk", &m_straightTrunk, 0.1f, 0.0f, 100.f) || changed;
	if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen))
	{
		changed = ImGui::DragInt("Base node count", &m_baseInternodeCount, 1, 0, 3) || changed;
		changed = ImGui::DragFloat2("Base apical angle mean/var", &m_baseNodeApicalAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;

		changed = ImGui::DragInt("Lateral bud count", &m_lateralBudCount, 1, 0, 3) || changed;
		changed = ImGui::DragInt("Max Order", &m_maxOrder, 1, -1, 100) || changed;
		if (ImGui::TreeNodeEx("Angles"))
		{
			changed = ImGui::DragFloat2("Branching angle base/var", &m_branchingAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			//editorLayer->DragAndDropButton<ProceduralNoise2D>(m_branchingAngle, "Branching Angle Noise");
			changed = ImGui::DragFloat2("Roll angle base/var", &m_rollAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			//editorLayer->DragAndDropButton<ProceduralNoise2D>(m_rollAngle, "Roll Angle Noise");
			changed = ImGui::DragFloat2("Apical angle base/var", &m_apicalAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			//editorLayer->DragAndDropButton<ProceduralNoise2D>(m_apicalAngle, "Apical Angle Noise");
			if (ImGui::TreeNodeEx("Roll Angle Noise2D"))
			{
				changed = m_rollAngleNoise2D.OnInspect() | changed;
				ImGui::TreePop();
			}
			if (ImGui::TreeNodeEx("Apical Angle Noise2D"))
			{
				changed = m_apicalAngleNoise2D.OnInspect() | changed;
				ImGui::TreePop();
			}
			ImGui::TreePop();
		}
		changed = ImGui::DragFloat("Internode length", &m_internodeLength, 0.001f) || changed;
		changed = ImGui::DragFloat("Internode length thickness factor", &m_internodeLengthThicknessFactor, 0.0001f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat3("Thickness min/factor/age", &m_endNodeThickness, 0.0001f, 0.0f, 1.0f, "%.6f") || changed;

		changed = ImGui::DragFloat("Bending strength", &m_gravityBendingStrength, 0.01f, 0.0f, 1.0f, "%.3f") || changed;
		changed = ImGui::DragFloat("Bending thickness factor", &m_gravityBendingThicknessFactor, 0.1f, 0.0f, 10.f, "%.3f") || changed;
		changed = ImGui::DragFloat("Bending angle factor", &m_gravityBendingMax, 0.01f, 0.0f, 1.0f, "%.3f") || changed;

		changed = ImGui::DragFloat("Internode shadow factor", &m_internodeShadowFactor, 0.001f, 0.0f, 1.0f) || changed;

		
		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Bud fate", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed = ImGui::DragFloat("Gravitropism", &m_gravitropism, 0.01f) || changed;
		changed = ImGui::DragFloat("Phototropism", &m_phototropism, 0.01f) || changed;
		changed = ImGui::DragFloat("Horizontal Tropism", &m_horizontalTropism, 0.01f) || changed;

		changed = ImGui::DragFloat("Apical bud extinction rate", &m_apicalBudExtinctionRate, 0.01f, 0.0f, 1.0f, "%.5f") || changed;
		changed = ImGui::DragFloat("Lateral bud flushing rate", &m_lateralBudFlushingRate, 0.01f, 0.0f, 1.0f, "%.5f") || changed;
		
		changed = ImGui::DragFloat2("Inhibitor val/loss", &m_apicalDominance, 0.01f) || changed;
		ImGui::TreePop();
	}
	if(ImGui::TreeNodeEx("Tree Shape Control", ImGuiTreeNodeFlags_DefaultOpen))
	{
		changed = ImGui::DragFloat("Apical control", &m_apicalControl, 0.01f) || changed;
		changed = ImGui::DragFloat("Root distance control", &m_rootDistanceControl, 0.01f) || changed;
		changed = ImGui::DragFloat("Height control", &m_heightControl, 0.01f) || changed;

		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Pruning", ImGuiTreeNodeFlags_DefaultOpen))
	{
		changed = ImGui::Checkbox("Trunk Protection", &m_trunkProtection) || changed;
		changed = ImGui::DragInt("Max chain length", &m_maxFlowLength, 1) || changed;
		changed = ImGui::DragFloat("Light pruning threshold", &m_lightPruningFactor, 0.01f) || changed;
		
		changed = ImGui::DragFloat("Branch strength", &m_branchStrength, 0.01f, 0.0f) || changed;
		changed = ImGui::DragFloat("Branch strength thickness factor", &m_branchStrengthThicknessFactor, 0.01f, 0.0f) || changed;
		changed = ImGui::DragFloat("Branch strength lighting threshold", &m_branchStrengthLightingThreshold, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Branch strength lighting loss", &m_branchStrengthLightingLoss, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Branch breaking multiplier", &m_branchBreakingMultiplier, 0.01f, 0.01f, 10.0f) || changed;

		changed = ImGui::DragFloat("Branch breaking factor", &m_branchBreakingFactor, 0.01f, 0.01f, 10.0f) || changed;


		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Leaf"))
	{
		changed = ImGui::DragFloat("Lighting requirement", &m_leafFlushingLightingRequirement, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Drop prob", &m_leafFallProbability, 0.01f) || changed;
		changed = ImGui::DragFloat("Distance To End Limit", &m_leafDistanceToBranchEndLimit, 0.01f) || changed;
		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Fruit"))
	{
		changed = ImGui::DragFloat("Lighting requirement", &m_fruitFlushingLightingRequirement, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Drop prob", &m_fruitFallProbability, 0.01f) || changed;
		changed = ImGui::DragFloat("Distance To End Limit", &m_fruitDistanceToBranchEndLimit, 0.01f) || changed;
		ImGui::TreePop();
	}

	editorLayer->DragAndDropButton<Material>(m_barkMaterial, "Bark Material##SBS");

	if (changed) m_saved = false;
}

void ShootDescriptor::CollectAssetRef(std::vector<AssetRef>& list)
{
	if (m_rollAngle.Get<ProceduralNoise2D>()) list.push_back(m_rollAngle);
	if (m_apicalAngle.Get<ProceduralNoise2D>()) list.push_back(m_apicalAngle);

	if (m_barkMaterial.Get<Material>()) list.push_back(m_barkMaterial);
}
