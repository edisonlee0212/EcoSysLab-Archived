#pragma once

#include "Json.hpp"
#include "jsSetupConfigParser.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class JoeScanScanner : public IPrivateComponent {
		std::mutex* m_scannerMutex = nullptr;
		bool m_scanEnabled = false;
		JobDependency m_scannerJob{};
		float m_scanTimeStep = 0.5f;

		std::vector<glm::vec2> m_points;
	public:
		AssetRef m_config;
		void StopScanningProcess();
		void StartScanProcess();
		JoeScanScanner();
		~JoeScanScanner() override;
		static bool InitializeScanSystem(const std::shared_ptr<Json>& json, jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads);
		static void FreeScanSystem(jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads);

		static void InitializeScanHeads(jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads);
		//static void Scan(const jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads, std::vector<glm::vec2>& result);
		jsScanSystem m_scanSystem = 0;
		std::vector<jsScanHead> m_scanHeads;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void FixedUpdate() override;
		void OnCreate() override;
		void OnDestroy() override;
		void CollectAssetRef(std::vector<AssetRef>& list) override;
	};
}
