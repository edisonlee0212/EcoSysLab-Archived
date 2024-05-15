#pragma once

#include "ClimateModel.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class ClimateDescriptor : public IAsset {
	public:
		ClimateParameters m_climateParameters;

		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void Serialize(YAML::Emitter& out) const override;

		void Deserialize(const YAML::Node& in) override;
	};
	class Climate : public IPrivateComponent {

	public:
		ClimateModel m_climateModel;
		AssetRef m_climateDescriptor;
		
		/**ImGui menu goes here. Also, you can take care you visualization with Gizmos here.
		 * Note that the visualization will only be activated while you are inspecting the soil private component in the entity inspector.
		 */
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Serialize(YAML::Emitter& out) const override;

		void Deserialize(const YAML::Node& in) override;

		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void InitializeClimateModel();

		void PrepareForGrowth();
	};
}
