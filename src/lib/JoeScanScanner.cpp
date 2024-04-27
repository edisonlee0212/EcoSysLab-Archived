#include "JoeScanScanner.hpp"
#include "Json.hpp"
using namespace EcoSysLab;

void JoeScan::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_profiles" << YAML::Value << YAML::BeginSeq;
	for (const auto& profile : m_profiles)
	{
		out << YAML::BeginMap;
		{
			out << YAML::Key << "m_encoderValue" << YAML::Value << profile.m_encoderValue;
			out << YAML::Key << "m_points" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(profile.m_points.data()), profile.m_points.size() * sizeof(glm::vec2));
			out << YAML::Key << "m_brightness" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(profile.m_brightness.data()), profile.m_brightness.size() * sizeof(float));
		}
		out << YAML::EndMap;
	}
}

void JoeScan::Deserialize(const YAML::Node& in)
{
	if (in["m_profiles"])
	{
		m_profiles.clear();
		for (const auto& inProfile : in["m_profile"])
		{
			m_profiles.emplace_back();
			auto& profile = m_profiles.back();
			if (inProfile["m_encoderValue"]) profile.m_encoderValue = inProfile["m_encoderValue"].as<float>();
			if (inProfile["m_points"])
			{
				const auto inPoints = inProfile["m_points"].as<YAML::Binary>();
				profile.m_points.resize(inPoints.size() / sizeof(glm::vec2));
				std::memcpy(profile.m_points.data(), inPoints.data(), inPoints.size());
			}
			if (inProfile["m_brightness"])
			{
				const auto inBrightness = inProfile["m_brightness"].as<YAML::Binary>();
				profile.m_brightness.resize(inBrightness.size() / sizeof(float));
				std::memcpy(profile.m_brightness.data(), inBrightness.data(), inBrightness.size());
			}
		}
	}
}

void JoeScan::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static std::shared_ptr<ParticleInfoList> joeScanList;
	if (!joeScanList) joeScanList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

	static bool enableJoeScanRendering = true;
	ImGui::Checkbox("Render JoeScan", &enableJoeScanRendering);
	if (enableJoeScanRendering) {
		if (ImGui::Button("Refresh JoeScan"))
		{
			std::vector<ParticleInfo> data;
			for (const auto& profile : m_profiles)
			{
				const auto startIndex = data.size();
				data.resize(profile.m_points.size() + startIndex);
				Jobs::RunParallelFor(profile.m_points.size(), [&](unsigned i)
					{
						data[i].m_instanceMatrix.SetPosition(glm::vec3(profile.m_points[i + startIndex].x / 10000.f, profile.m_points[i + startIndex].y / 10000.f, profile.m_encoderValue / 100.f));
					}
				);
			}
			joeScanList->SetParticleInfos(data);
		}
	}

	if (enableJoeScanRendering)
	{
		editorLayer->DrawGizmoCubes(joeScanList, glm::mat4(1), 0.001f);
	}
}

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
	}
	else
	{
		return;
	}
	if (m_scannerJob.Valid()) {
		Jobs::Wait(m_scannerJob);
		m_scannerJob = {};
		m_points.clear();
		if (const auto joeScan = m_joeScan.Get<JoeScan>())
		{
			joeScan->m_profiles.clear();
			for (const auto& profile : m_preservedProfiles)
			{
				joeScan->m_profiles.emplace_back(profile.second);
			}
		}
	}
}

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
			int i = 0;
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
				JoeScanProfile joeScanProfile;
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
								joeScanProfile.m_points.emplace_back(glm::vec2(point.x, point.y));
								joeScanProfile.m_brightness.emplace_back(point.brightness);
							}
						}
						if (containRealData) validCount++;
					}
				}
				if (validCount != 0) {
					std::lock_guard lock(*m_scannerMutex);
					m_points = points;
					if (i % 50 == 0) {
						joeScanProfile.m_encoderValue = i;
						m_preservedProfiles[i] = joeScanProfile;
					}
					i++;
				}
			}
		}
	);
}

JoeScanScanner::JoeScanScanner()
{
	m_scannerMutex = std::make_shared<std::mutex>();
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
		int retVal = joescan::jsSetupConfigParse(json->m_json, scanSystem, scanHeads, &logger);
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
	editorLayer->DragAndDropButton<JoeScan>(m_joeScan, "JoeScan");
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
	if (m_scanSystem != 0 && !m_scanEnabled && ImGui::Button("Start Scanning"))
	{
		std::vector<glm::vec2> results;
		StartScanProcess();
	}

	if (m_scanEnabled && ImGui::Button("Stop Scanning"))
	{
		StopScanningProcess();
	}
	static std::shared_ptr<ParticleInfoList> latestPointList;
	if (!latestPointList) latestPointList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (m_scanEnabled) {
		static bool enableLatestPointRendering = true;
		ImGui::Checkbox("Render Latest Points", &enableLatestPointRendering);
		if (enableLatestPointRendering && !m_points.empty())
		{
			std::vector<ParticleInfo> data;
			data.resize(m_points.size());
			{
				std::lock_guard lock(*m_scannerMutex);
				Jobs::RunParallelFor(m_points.size(), [&](unsigned i)
					{
						data[i].m_instanceMatrix.SetPosition(glm::vec3(m_points[i].x, m_points[i].y, -1.f));

					});
			}
			latestPointList->SetParticleInfos(data);
			editorLayer->DrawGizmoCubes(latestPointList, glm::mat4(1), 0.001f);
		}
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
