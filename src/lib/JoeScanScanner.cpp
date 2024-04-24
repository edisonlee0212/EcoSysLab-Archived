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

void JoeScanScanner::StopScanningProcess()
{
	if (m_scanEnabled) {
		m_scanEnabled = false;
	}else
	{
		return;
	}
	if (m_scannerJob.Valid()) {
		Jobs::Wait(m_scannerJob);
		m_scannerJob = {};
	}
}
/*
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
*/
void JoeScanScanner::StartScanProcess()
{
	StopScanningProcess();
	const int32_t minPeriod = jsScanSystemGetMinScanPeriod(m_scanSystem);
	if (0 >= minPeriod) {
		EVOENGINE_ERROR("Failed to read min scan period.");
	}

	const int startScanningResult = jsScanSystemStartFrameScanning(m_scanSystem, minPeriod, JS_DATA_FORMAT_XY_BRIGHTNESS_FULL);
	if (0 > startScanningResult) {
		EVOENGINE_ERROR("Failed to start scanning.");
		return;
	}

	m_scanEnabled = true;
	m_scannerJob = Jobs::Run([&]()
		{
			SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
			const auto profileSize = jsScanSystemGetProfilesPerFrame(m_scanSystem);
			while (m_scanEnabled)
			{
				const int scanResult = jsScanSystemWaitUntilFrameAvailable(m_scanSystem, 1000);
				if (0 == scanResult) {
					continue;
				}
				if (0 > scanResult) {
					EVOENGINE_ERROR("Failed to wait for frame.");
					break;
				}

				std::vector<jsProfile> profiles;
				profiles.resize(profileSize);
				const int getFrameResult = jsScanSystemGetFrame(m_scanSystem, profiles.data());
				if (0 >= getFrameResult) {
					EVOENGINE_ERROR("Failed to read frame.");
					break;
				}
				size_t validCount = 0;
				std::vector<glm::vec2> points;
				for (int profileIndex = 0; profileIndex < profileSize; profileIndex++) {
					if (jsRawProfileIsValid(profiles[profileIndex]))
					{
						bool containRealData = false;
						for (const auto& point : profiles[profileIndex].data)
						{
							if (point.x != 0 || point.y != 0)
							{
								containRealData = true;
								points.emplace_back(glm::vec2(point.x, point.y) / 100000.f);
							}
						}
						if (containRealData) validCount++;
					}
				}
				if (validCount != 0) {
					EVOENGINE_LOG("Received [" + std::to_string(validCount) + "/" + std::to_string(profileSize) + "] profiles");
					std::lock_guard lock(*m_scannerMutex);
					m_points = points;
				}
			}
		}
	);
}

JoeScanScanner::JoeScanScanner()
{
	m_scannerMutex = new std::mutex;
}

JoeScanScanner::~JoeScanScanner()
{
	delete m_scannerMutex;
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
	if(m_scanSystem != 0 && !m_scanEnabled && ImGui::Button("Start Scanning"))
	{
		std::vector<glm::vec2> results;
		StartScanProcess();
	}

	if(m_scanEnabled && ImGui::Button("Stop Scanning"))
	{
		StopScanningProcess();
	}

	static bool enableRendering = true;
	static std::shared_ptr<ParticleInfoList> list;
	if (!list) list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if(enableRendering)
	{
		std::vector<glm::vec2> copy;
		{
			std::lock_guard lock(*m_scannerMutex);
			copy = m_points;
		}
		std::vector<ParticleInfo> data;
		data.resize(copy.size());
		Jobs::RunParallelFor(copy.size(), [&](unsigned i)
			{
				data[i].m_instanceMatrix.SetPosition(glm::vec3(copy[i].x, copy[i].y, -10.f));

			});
		list->SetParticleInfos(data);
		editorLayer->DrawGizmoCubes(list, glm::mat4(1), 0.1f);
	}
}

void JoeScanScanner::FixedUpdate()
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
