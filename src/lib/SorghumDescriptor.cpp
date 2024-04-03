#include "ProjectManager.hpp"
#include "SorghumLayer.hpp"
#include "SorghumDescriptor.hpp"

#include "Times.hpp"

#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumSpline.hpp"

using namespace EcoSysLab;

void TipMenu(const std::string& content) {
	if (ImGui::IsItemHovered()) {
		ImGui::BeginTooltip();
		ImGui::TextUnformatted(content.c_str());
		ImGui::EndTooltip();
	}
}

void SorghumDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	if (ImGui::Button("Instantiate")) {
		auto entity = CreateEntity();
	}
	static bool autoSave = true;
	ImGui::Checkbox("Auto save", &autoSave);
	static bool intro = true;
	ImGui::Checkbox("Introduction", &intro);
	if (intro) {
		ImGui::TextWrapped(
			"This is the introduction of the parameter setting interface. "
			"\nFor each parameter, you are allowed to set average and "
			"variance value. \nInstantiate a new sorghum in the scene so you "
			"can preview the changes in real time. \nThe curve editors are "
			"provided for stem/leaf details to allow you have control of "
			"geometric properties along the stem/leaf. It's also provided "
			"for leaf settings to allow you control the distribution of "
			"different leaves from the bottom to top.\nMake sure you Save the "
			"parameters!\nValues are in meters or degrees.");
	}
	bool changed = false;
	if (ImGui::TreeNodeEx("Panicle settings", ImGuiTreeNodeFlags_DefaultOpen)) {
		TipMenu("The settings for panicle. The panicle will always be placed "
			"at the tip of the stem.");
		if (m_panicleSize.OnInspect("Size", 0.001f, "The size of panicle")) {
			changed = true;
		}
		if (m_panicleSeedAmount.OnInspect("Seed amount", 1.0f,
			"The amount of seeds in the panicle"))
			changed = true;
		if (m_panicleSeedRadius.OnInspect("Seed radius", 0.001f,
			"The size of the seed in the panicle"))
			changed = true;
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx("Stem settings", ImGuiTreeNodeFlags_DefaultOpen)) {
		TipMenu("The settings for stem.");
		if (m_stemTiltAngle.OnInspect("Stem tilt angle", 0.001f,
			"The tilt angle for stem")) {
			changed = true;
		}
		if (m_stemLength.OnInspect(
			"Length", 0.01f,
			"The length of the stem, use Ending Point in leaf settings to make "
			"stem taller than top leaf for panicle"))
			changed = true;
		if (m_stemWidth.OnInspect("Width", 0.001f,
			"The overall width of the stem, adjust the width "
			"along stem in Stem Details"))
			changed = true;
		if (ImGui::TreeNode("Stem Details")) {
			TipMenu("The detailed settings for stem.");
			if (m_widthAlongStem.OnInspect("Width along stem"))
				changed = true;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Leaves settings", ImGuiTreeNodeFlags_DefaultOpen)) {
		TipMenu("The settings for leaves.");
		if (m_leafAmount.OnInspect("Num of leaves", 1.0f,
			"The total amount of leaves"))
			changed = true;

		static PlottedDistributionSettings leafStartingPoint = {
			0.01f,
			{0.01f, false, true, ""},
			{0.01f, false, false, ""},
			"The starting point of each leaf along stem. Default each leaf "
			"located uniformly on stem." };

		if (m_leafStartingPoint.OnInspect("Starting point along stem",
			leafStartingPoint)) {
			changed = true;
		}

		static PlottedDistributionSettings leafCurling = { 0.01f,
														{0.01f, false, true, ""},
														{0.01f, false, false, ""},
														"The leaf curling." };

		if (m_leafCurling.OnInspect("Leaf curling", leafCurling)) {
			changed = true;
		}

		static PlottedDistributionSettings leafRollAngle = {
			0.01f,
			{},
			{},
			"The polar angle of leaf. Normally you should only change the "
			"deviation. Values are in degrees" };
		if (m_leafRollAngle.OnInspect("Roll angle", leafRollAngle))
			changed = true;

		static PlottedDistributionSettings leafBranchingAngle = {
			0.01f,
			{},
			{},
			"The branching angle of the leaf. Values are in degrees" };
		if (m_leafBranchingAngle.OnInspect("Branching angle", leafBranchingAngle))
			changed = true;

		static PlottedDistributionSettings leafBending = {
			1.0f,
			{1.0f, false, true, ""},
			{},
			"The bending of the leaf, controls how leaves bend because of "
			"gravity. Positive value results in leaf bending towards the "
			"ground, negative value results in leaf bend towards the sky" };
		if (m_leafBending.OnInspect("Bending", leafBending))
			changed = true;

		static PlottedDistributionSettings leafBendingAcceleration = {
			0.01f,
			{0.01f, false, true, ""},
			{},
			"The changes of bending along the leaf." };

		if (m_leafBendingAcceleration.OnInspect("Bending acceleration",
			leafBendingAcceleration))
			changed = true;

		static PlottedDistributionSettings leafBendingSmoothness = {
			0.01f,
			{0.01f, false, true, ""},
			{},
			"The smoothness of bending along the leaf." };

		if (m_leafBendingSmoothness.OnInspect("Bending smoothness",
			leafBendingSmoothness))
			changed = true;

		if (m_leafWaviness.OnInspect("Waviness"))
			changed = true;
		if (m_leafWavinessFrequency.OnInspect("Waviness Frequency"))
			changed = true;
		if (m_leafPeriodStart.OnInspect("Waviness Period Start"))
			changed = true;

		if (m_leafLength.OnInspect("Length"))
			changed = true;
		if (m_leafWidth.OnInspect("Width"))
			changed = true;

		if (ImGui::TreeNode("Leaf Details")) {
			if (m_widthAlongLeaf.OnInspect("Width along leaf"))
				changed = true;
			if (m_wavinessAlongLeaf.OnInspect("Waviness along leaf"))
				changed = true;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	if (changed) {
		m_saved = false;
		m_version++;
	}

	static double lastAutoSaveTime = 0;
	static float autoSaveInterval = 5;

	if (autoSave) {
		if (ImGui::TreeNodeEx("Auto save settings")) {
			if (ImGui::DragFloat("Time interval", &autoSaveInterval, 1.0f, 2.0f,
				300.0f)) {
				autoSaveInterval = glm::clamp(autoSaveInterval, 5.0f, 300.0f);
			}
			ImGui::TreePop();
		}
		if (lastAutoSaveTime == 0) {
			lastAutoSaveTime = Times::Now();
		}
		else if (lastAutoSaveTime + autoSaveInterval <
			Times::Now()) {
			lastAutoSaveTime = Times::Now();
			if (!m_saved) {
				Save();
				EVOENGINE_LOG(GetTypeName() + " autosaved!");
			}
		}
	}
	else {
		if (!m_saved) {
			ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
			ImGui::Text("[Changed unsaved!]");
			ImGui::PopStyleColor();
		}
	}
}
void SorghumDescriptor::Serialize(YAML::Emitter& out) {
	out << YAML::Key << "m_version" << YAML::Value << m_version;
	m_panicleSize.Save("m_panicleSize", out);
	m_panicleSeedAmount.Save("m_panicleSeedAmount", out);
	m_panicleSeedRadius.Save("m_panicleSeedRadius", out);

	m_stemTiltAngle.Save("m_stemTiltAngle", out);
	m_stemLength.Save("m_stemLength", out);
	m_stemWidth.Save("m_stemWidth", out);

	m_leafAmount.Save("m_leafAmount", out);
	m_leafStartingPoint.Save("m_leafStartingPoint", out);
	m_leafCurling.Save("m_leafCurling", out);
	m_leafRollAngle.Save("m_leafRollAngle", out);
	m_leafBranchingAngle.Save("m_leafBranchingAngle", out);
	m_leafBending.Save("m_leafBending", out);
	m_leafBendingAcceleration.Save("m_leafBendingAcceleration", out);
	m_leafBendingSmoothness.Save("m_leafBendingSmoothness", out);
	m_leafWaviness.Save("m_leafWaviness", out);
	m_leafWavinessFrequency.Save("m_leafWavinessFrequency", out);
	m_leafPeriodStart.Save("m_leafPeriodStart", out);
	m_leafLength.Save("m_leafLength", out);
	m_leafWidth.Save("m_leafWidth", out);

	m_widthAlongStem.Save("m_widthAlongStem", out);
	m_widthAlongLeaf.Save("m_widthAlongLeaf", out);
	m_wavinessAlongLeaf.Save("m_wavinessAlongLeaf",
		out);
}
void SorghumDescriptor::Deserialize(const YAML::Node& in) {
	if (in["m_version"])
		m_version = in["m_version"].as<unsigned>();

	m_panicleSize.Load("m_panicleSize", in);
	m_panicleSeedAmount.Load("m_panicleSeedAmount", in);
	m_panicleSeedRadius.Load("m_panicleSeedRadius", in);

	m_stemTiltAngle.Load("m_stemTiltAngle", in);
	m_stemLength.Load("m_stemLength", in);
	m_stemWidth.Load("m_stemWidth", in);

	m_leafAmount.Load("m_leafAmount", in);
	m_leafStartingPoint.Load("m_leafStartingPoint", in);
	m_leafCurling.Load("m_leafCurling", in);

	m_leafRollAngle.Load("m_leafRollAngle", in);
	m_leafBranchingAngle.Load("m_leafBranchingAngle", in);

	m_leafBending.Load("m_leafBending", in);
	m_leafBendingAcceleration.Load("m_leafBendingAcceleration", in);
	m_leafBendingSmoothness.Load("m_leafBendingSmoothness", in);
	m_leafWaviness.Load("m_leafWaviness", in);
	m_leafWavinessFrequency.Load("m_leafWavinessFrequency", in);
	m_leafPeriodStart.Load("m_leafPeriodStart", in);
	m_leafLength.Load("m_leafLength", in);
	m_leafWidth.Load("m_leafWidth", in);

	m_widthAlongStem.Load("m_widthAlongStem",
		in);
	m_widthAlongLeaf.Load("m_widthAlongLeaf",
		in);
	m_wavinessAlongLeaf.Load(
		"m_wavinessAlongLeaf", in);
}

Entity SorghumDescriptor::CreateEntity(const unsigned int seed) const
{
	const auto scene = Application::GetActiveScene();
	const auto entity = scene->CreateEntity(GetTitle());
	const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(entity).lock();
	const auto sorghumState = ProjectManager::CreateTemporaryAsset<SorghumState>();
	Apply(sorghumState, seed);
	sorghum->m_sorghumState = sorghumState;
	sorghum->m_sorghumDescriptor = GetSelf();
	sorghum->GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
	return entity;
}

void SorghumDescriptor::Apply(const std::shared_ptr<SorghumState>& targetState, const unsigned int seed) const
{
	srand(seed);

	const auto upDirection = glm::vec3(0, 1, 0);
	auto frontDirection = glm::vec3(0, 0, -1);
	frontDirection = glm::rotate(
		frontDirection, glm::radians(glm::linearRand(0.0f, 360.0f)), upDirection);

	glm::vec3 stemFront = glm::normalize(
		glm::rotate(upDirection,
			glm::radians(glm::gaussRand(m_stemTiltAngle.m_mean, m_stemTiltAngle.m_deviation)), frontDirection));
	float stemLength = m_stemLength.GetValue();
	Plot2D<float> widthAlongStem = { 0.0f, m_stemWidth.GetValue(),
										m_widthAlongStem };
	const auto sorghumLayer = Application::GetLayer<SorghumLayer>();
	//Build stem...
	targetState->m_stem.m_nodes.clear();
	int stemNodeAmount = static_cast<int>(glm::max(
		4.0f, stemLength / sorghumLayer->m_verticalSubdivisionMaxUnitLength));
	float stemUnitLength = stemLength / stemNodeAmount;
	glm::vec3 stemLeft = glm::normalize(glm::rotate(glm::vec3(1, 0, 0),
		glm::radians(glm::linearRand(0.0f, 0.0f)), stemFront));
	for (int i = 0; i <= stemNodeAmount; i++) {
		float stemWidth = widthAlongStem.GetValue(static_cast<float>(i) / stemNodeAmount);
		glm::vec3 stemNodePosition;
		stemNodePosition = stemFront * stemUnitLength * static_cast<float>(i);
		targetState->m_stem.m_nodes.emplace_back(stemNodePosition, 180.0f, stemWidth, 0.0f,
			-stemFront, stemLeft, static_cast<float>(i) / stemNodeAmount);
	}

	const int leafSize = glm::clamp(m_leafAmount.GetValue(), 2.0f, 128.0f);
	targetState->m_leaves.resize(leafSize);
	for (int i = 0; i < leafSize; i++) {
		const float step = static_cast<float>(i) / (static_cast<float>(leafSize) - 1.0f);
		auto& leafState = targetState->m_leaves[i];
		leafState.m_nodes.clear();
		leafState.m_index = i;
		leafState.m_wavinessFrequency.x = m_leafWavinessFrequency.GetValue(step);
		leafState.m_wavinessFrequency.y = m_leafWavinessFrequency.GetValue(step);

		leafState.m_wavinessPeriodStart.x = m_leafPeriodStart.GetValue(step);
		leafState.m_wavinessPeriodStart.y = m_leafPeriodStart.GetValue(step);

		float startingPointRatio = m_leafStartingPoint.GetValue(step);
		float leafLength = m_leafLength.GetValue(step);
		if (leafLength == 0.0f) return;

		Plot2D wavinessAlongLeaf = { 0.0f, m_leafWaviness.GetValue(step) * 2.0f, m_wavinessAlongLeaf };
		Plot2D widthAlongLeaf = { 0.0f, m_leafWidth.GetValue(step) * 2.0f, m_widthAlongLeaf };
		auto curling =
			glm::clamp(m_leafCurling.GetValue(step), 0.0f, 90.0f) / 90.0f;
		Plot2D curlingAlongLeaf = {0.0f, 90.0f, {curling, curling} };
		//auto curling = glm::clamp(m_leafCurling.GetValue(step), 0.0f, 90.0f) / 90.0f;
		//Plot2D curlingAlongLeaf = {0.0f, curling * 90.0f, m_curlingAlongLeaf };
		float branchingAngle = m_leafBranchingAngle.GetValue(step);
		float rollAngle = glm::mod((i % 2) * 180.0f + m_leafRollAngle.GetValue(step), 360.0f);
		auto bending = m_leafBending.GetValue(step);
		bending = (bending + 180) / 360.0f;
		const auto bendingAcceleration = m_leafBendingAcceleration.GetValue(step);
		const auto bendingSmoothness = m_leafBendingSmoothness.GetValue(step);

		Plot2D bendingAlongLeaf = {-180.0f, 180.0f, {0.5f, bending} };
		const glm::vec2 middle = glm::mix(glm::vec2(0, bending), glm::vec2(1, 0.5f), bendingAcceleration);
		auto& bendingAlongLeafCurve = bendingAlongLeaf.m_curve.UnsafeGetValues();
		bendingAlongLeafCurve.clear();
		bendingAlongLeafCurve.emplace_back(-0.1, 0.0f);
		bendingAlongLeafCurve.emplace_back(0, 0.5f);
		glm::vec2 leftDelta = { middle.x, middle.y - 0.5f };
		bendingAlongLeafCurve.push_back(leftDelta * (1.0f - bendingSmoothness));
		glm::vec2 rightDelta = { middle.x - 1.0f, bending - middle.y };
		bendingAlongLeafCurve.push_back(rightDelta * (1.0f - bendingSmoothness));
		bendingAlongLeafCurve.emplace_back(1.0, bending);
		bendingAlongLeafCurve.emplace_back(0.1, 0.0f);


		//Build nodes...
		float stemWidth = widthAlongStem.GetValue(startingPointRatio);
		float backTrackRatio = 0.05f;
		if (startingPointRatio < backTrackRatio) backTrackRatio = startingPointRatio;

		glm::vec3 leafLeft = glm::normalize(glm::rotate(glm::vec3(0, 0, -1), glm::radians(rollAngle),
			glm::vec3(0, 1, 0)));
		auto leafUp = glm::normalize(glm::cross(leafLeft, stemFront));
		glm::vec3 stemOffset = stemWidth * leafUp;
		glm::vec3 nodePosition = stemFront * startingPointRatio * stemLength + stemOffset;
		//BezierSpline middleSpline;
		
		auto direction = glm::rotate(glm::vec3(0, 1, 0), glm::radians(branchingAngle), leafLeft);
		bool modelToRoot = true;
		float sheathRatio = startingPointRatio - backTrackRatio;
		if (modelToRoot) {
			if (sheathRatio > 0) {
				int rootToSheathNodeCount =
					glm::min(2.0f, stemLength * sheathRatio /
						sorghumLayer->m_verticalSubdivisionMaxUnitLength);
				for (int i = 0; i < rootToSheathNodeCount; i++) {
					float currentRootToSheathPoint = static_cast<float>(i) / rootToSheathNodeCount * sheathRatio;
					glm::vec3 actualDirection = stemFront;
					leafState.m_nodes.emplace_back(glm::normalize(stemFront) * currentRootToSheathPoint * stemLength + stemOffset, 180.0f,
						stemWidth, 0.0f,
						-actualDirection, leafLeft, 0.0f);
				}
			}
		}
		
		int sheathNodeCount =
			glm::max(2.0f, stemLength * backTrackRatio /
				sorghumLayer->m_verticalSubdivisionMaxUnitLength);
		for (int i = 0; i <= sheathNodeCount; i++) {
			float currentSheathPoint = sheathRatio + static_cast<float>(i) / sheathNodeCount * backTrackRatio;
			glm::vec3 actualDirection =
				glm::mix(stemFront, direction, static_cast<float>(i) / sheathNodeCount);
			leafState.m_nodes.emplace_back(glm::normalize(stemFront) * currentSheathPoint * stemLength + stemOffset,
				180.0f - 90.0f * static_cast<float>(i) / sheathNodeCount,
				stemWidth + 0.002f * static_cast<float>(i) / sheathNodeCount,
				0.0f, -actualDirection, leafLeft, 0.0f);
		}

		int nodeAmount = glm::max(
			4.0f, leafLength / sorghumLayer->m_verticalSubdivisionMaxUnitLength);
		float unitLength = leafLength / nodeAmount;

		int nodeToFullExpand =
			0.1f * leafLength / sorghumLayer->m_verticalSubdivisionMaxUnitLength;

		for (int i = 1; i <= nodeAmount; i++) {
			const float factor = static_cast<float>(i) / nodeAmount;
			glm::vec3 currentDirection;

			float rotateAngle = bendingAlongLeaf.GetValue(factor);
			currentDirection =
				glm::rotate(direction, glm::radians(rotateAngle), leafLeft);
			nodePosition += currentDirection * unitLength;

			float expandAngle = curlingAlongLeaf.GetValue(factor);

			float collarFactor = glm::min(1.0f, static_cast<float>(i) / nodeToFullExpand);
			float waviness = wavinessAlongLeaf.GetValue(factor);
			float width = glm::mix(
				stemWidth + 0.002f,
				widthAlongLeaf.GetValue(factor),
				collarFactor);
			float angle = 90.0f - (90.0f - expandAngle) * glm::pow(collarFactor, 2.0f);
			leafState.m_nodes.emplace_back(nodePosition, angle,
				width,
				waviness, -currentDirection, leafLeft, factor);
		}
	}

	targetState->m_panicle.m_seedAmount = m_panicleSeedAmount.GetValue();
	const auto panicleSize = m_panicleSize.GetValue();
	targetState->m_panicle.m_panicleSize = glm::vec3(panicleSize.x, panicleSize.y, panicleSize.x);
	targetState->m_panicle.m_seedRadius = m_panicleSeedRadius.GetValue();
}

void SorghumDescriptor::OnCreate() {
	m_panicleSize.m_mean = glm::vec3(0.0, 0.0, 0.0);
	m_panicleSeedAmount.m_mean = 0;
	m_panicleSeedRadius.m_mean = 0.002f;

	m_stemTiltAngle.m_mean = 0.0f;
	m_stemTiltAngle.m_deviation = 0.0f;
	m_stemLength.m_mean = 0.449999988f;
	m_stemLength.m_deviation = 0.150000006f;
	m_stemWidth.m_mean = 0.0140000004;
	m_stemWidth.m_deviation = 0.0f;

	m_leafAmount.m_mean = 9.0f;
	m_leafAmount.m_deviation = 1.0f;

	m_leafStartingPoint.m_mean = { 0.0f, 1.0f,
								  Curve2D(0.1f, 1.0f) };
	m_leafStartingPoint.m_deviation = {
		0.0f, 1.0f, Curve2D(0.0f, 0.0f) };

	m_leafCurling.m_mean = { 0.0f, 90.0f,
							Curve2D(0.3f, 0.7f) };
	m_leafCurling.m_deviation = { 0.0f, 1.0f,
								 Curve2D(0.0f, 0.0f) };
	m_leafRollAngle.m_mean = { -1.0f, 1.0f,
							  Curve2D(0.5f, 0.5f) };
	m_leafRollAngle.m_deviation = { 0.0f, 6.0f,
								   Curve2D(0.3f, 1.0f) };

	m_leafBranchingAngle.m_mean = { 0.0f, 55.0f,
								   Curve2D(0.5f, 0.2f) };
	m_leafBranchingAngle.m_deviation = {
		0.0f, 3.0f, Curve2D(0.67f, 0.225f) };

	m_leafBending.m_mean = { -180.0f, 180.0f,
							Curve2D(0.5f, 0.5f) };
	m_leafBending.m_deviation = { 0.0f, 0.0f,
								 Curve2D(0.5f, 0.5f) };

	m_leafBendingAcceleration.m_mean = {
		0.0f, 1.0f, Curve2D(0.5f, 0.5f) };
	m_leafBendingSmoothness.m_mean = {
		0.0f, 1.0f, Curve2D(0.5f, 0.5f) };
	m_leafBendingAcceleration.m_deviation = {
		0.0f, 0.0f, Curve2D(0.5f, 0.5f) };

	m_leafWaviness.m_mean = { 0.0f, 20.0f,
							 Curve2D(0.5f, 0.5f) };
	m_leafWaviness.m_deviation = { 0.0f, 0.0f,
								  Curve2D(0.5f, 0.5f) };

	m_leafWavinessFrequency.m_mean = {
		0.0f, 1.0f, Curve2D(0.5f, 0.5f) };
	m_leafWavinessFrequency.m_deviation = {
		0.0f, 0.0f, Curve2D(0.5f, 0.5f) };

	m_leafLength.m_mean = { 0.0f, 2.5f,
						   Curve2D(0.165, 0.247) };
	m_leafLength.m_deviation = { 0.0f, 0.0f,
								Curve2D(0.5f, 0.5f) };

	m_leafWidth.m_mean = { 0.0f, 0.075f,
						  Curve2D(0.5f, 0.5f) };
	m_leafWidth.m_deviation = { 0.0f, 0.0f,
							   Curve2D(0.5f, 0.5f) };

	m_widthAlongStem = Curve2D(1.0f, 0.1f);
	m_widthAlongLeaf = Curve2D(0.5f, 0.1f);
	m_wavinessAlongLeaf = Curve2D(0.0f, 0.5f);
}
