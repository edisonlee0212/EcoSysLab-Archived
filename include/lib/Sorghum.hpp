#pragma once
#include "SorghumState.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class Sorghum : public IPrivateComponent
	{
	public:
		AssetRef m_sorghumDescriptor;
		AssetRef m_sorghumGrowthDescriptor;
		AssetRef m_sorghumState;
		void ClearGeometryEntities();
		void GenerateGeometryEntities(const SorghumMeshGeneratorSettings& sorghumMeshGeneratorSettings);

		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void CollectAssetRef(std::vector<AssetRef>& list) override;
	};
}