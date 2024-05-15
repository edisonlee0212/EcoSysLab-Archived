#pragma once
#include "Skeleton.hpp"

using namespace EvoEngine;


namespace EcoSysLab
{
	struct TwigParameters
	{
		float m_segmentLength = 0.01f;
		float m_apicalAngleVariance = 3.0f;
		float m_branchingAngle = 30.f;
		float m_thickness = 0.002f;
		float m_maxNodeThickness = 0.003f;
		float m_minRootDistance = 1.75f;
		float m_maxEndDistance = 999.0f;
		int m_segmentSize = 8;
		float m_unitDistance = 0.03f;
		void Save(const std::string& name, YAML::Emitter& out) const;
		void Load(const std::string& name, const YAML::Node& in);
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};

	class FoliageDescriptor : public IAsset
	{
	public:
		glm::vec2 m_leafSize = glm::vec2(0.02f, 0.04f);
		int m_leafCountPerInternode = 5;
		float m_positionVariance = 0.175f;
		float m_rotationVariance = 10.f;
		float m_branchingAngle = 30.f;
		float m_maxNodeThickness = 1.0f;
		float m_minRootDistance = 0.0f;
		float m_maxEndDistance = 0.2f;


		float m_horizontalTropism = 0.f;
		float m_gravitropism = 0.f;
		TwigParameters m_twigParameters{};

		AssetRef m_leafMaterial;
		AssetRef m_twigMaterial;
		void Serialize(YAML::Emitter& out) const override;
		void Deserialize(const YAML::Node& in) override;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internodeInfo, const float treeSize) const;
	};


}