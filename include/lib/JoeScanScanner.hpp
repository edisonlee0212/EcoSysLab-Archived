#pragma once

#include "Json.hpp"
#include "jsSetupConfigParser.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class JoeScanScanner : public IPrivateComponent {
	public:
		AssetRef m_config;
		static bool InitializeScanSystem(const std::shared_ptr<Json>& json, jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads);
		static void FreeScanSystem(jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads);
		static void Scan(const jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads, std::vector<glm::vec2>& result);
		jsScanSystem m_scanSystem = 0;
		std::vector<jsScanHead> m_scanHeads;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Update() override;
		void OnCreate() override;
		void OnDestroy() override;
		void CollectAssetRef(std::vector<AssetRef>& list) override;
	};
}
