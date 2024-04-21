#include "JoeScanScanner.hpp"
#include "Json.hpp"
using namespace EcoSysLab;
void logger(const jsError err, const std::string msg)
{
	EVOENGINE_LOG(msg);
	if (0 != err) {
		// If `err` is non-zero, `jsSetupConfigParse` failed parsing or initializing
		// something in the JSON file.
		const char* err_str = nullptr;
		jsGetError(err, &err_str);
		EVOENGINE_ERROR("JoeScan Error (" + std::to_string(err) + "): " + err_str)
	}
}

bool JoeScanScanner::InitializeScanSystem(const std::shared_ptr<Json>& json, jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads)
{
	try
	{
		FreeScanSystem(scanSystem, scanHeads);
		if (!json)
		{
			EVOENGINE_ERROR("JoeScan Error: Json config missing!");
			return false;
		}
		const auto path = json->GetAbsolutePath().string();
		int retVal = joescan::jsSetupConfigParse(path, scanSystem, scanHeads, &logger);
		if (0 > retVal) {
			// The Scan System and Scan Heads should be assumed to be in an
			// indeterminate state; only action to take is to free the Scan System.
			EVOENGINE_ERROR("JoeScan Error: Configuration failed");
			return false;
		}
		// Scan System and Scan Heads are fully configured.
		EVOENGINE_LOG("JoeScan: Configured successfully");

		retVal = jsScanSystemConnect(scanSystem, 5);
		if (retVal < 0)
		{
			EVOENGINE_ERROR("JoeScan Error: Connection failed");
			return false;
		}
		EVOENGINE_LOG("JoeScan: Connected to " + std::to_string(retVal) + " heads");

	}
	catch (const std::exception& e)
	{
		EVOENGINE_ERROR(e.what());
		return false;
	}
	return true;
}

void JoeScanScanner::FreeScanSystem(jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads)
{
	try
	{
		jsScanSystemDisconnect(scanSystem);
		EVOENGINE_LOG("JoeScan: Disconnected " + std::to_string(scanHeads.size()) + " heads");
		scanHeads.clear();
		if (scanSystem != 0)
		{
			jsScanSystemFree(scanSystem);
		}
		scanSystem = 0;
	}
	catch (const std::exception& e)
	{
		EVOENGINE_ERROR(e.what());
		return;
	}
	EVOENGINE_LOG("JoeScan: ScanSysten Freed!");
}

void JoeScanScanner::Scan(const jsScanSystem& scanSystem, std::vector<jsScanHead>& scanHeads, std::vector<glm::vec2>& result)
{
	jsScanSystemStartFrameScanning(scanSystem, 1000, JS_DATA_FORMAT_XY_FULL);
	jsScanSystemWaitUntilFrameAvailable(scanSystem, 1024);
	const auto profileSize = jsScanSystemGetProfilesPerFrame(scanSystem);
	std::vector<jsProfile> profiles;
	std::vector<jsRawProfile> rawProfiles;
	profiles.resize(profileSize);
	rawProfiles.resize(profileSize);
	jsScanSystemGetFrame(scanSystem, profiles.data());
	jsScanSystemGetRawFrame(scanSystem, rawProfiles.data());
	jsScanSystemClearFrames(scanSystem);
	jsScanSystemStopScanning(scanSystem);
}

void JoeScanScanner::Serialize(YAML::Emitter& out)
{
	m_config.Save("m_config", out);
}

void JoeScanScanner::Deserialize(const YAML::Node& in)
{
	m_config.Load("m_config", in);
}

void JoeScanScanner::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	editorLayer->DragAndDropButton<Json>(m_config, "Json Config");
	const auto config = m_config.Get<Json>();
	if (config && ImGui::Button("Initialize ScanSystem"))
	{
		InitializeScanSystem(config, m_scanSystem, m_scanHeads);
	}

	if (m_scanSystem != 0 && ImGui::Button("Free ScanSystem"))
	{
		FreeScanSystem(m_scanSystem, m_scanHeads);
	}

	ImGui::Separator();
	if(m_scanSystem != 0 && ImGui::Button("Scan"))
	{
		std::vector<glm::vec2> results;
		Scan(m_scanSystem, m_scanHeads, results);
	}
}

void JoeScanScanner::Update()
{

}

void JoeScanScanner::OnCreate()
{


}

void JoeScanScanner::OnDestroy()
{

}

void JoeScanScanner::CollectAssetRef(std::vector<AssetRef>& list)
{
	if (m_config.Get<Json>()) list.emplace_back(m_config);
}
