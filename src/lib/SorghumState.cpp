//
// Created by lllll on 1/8/2022.
//
#include "SorghumGrowthDescriptor.hpp"
#include "SorghumData.hpp"
#include "SorghumLayer.hpp"
#include "SorghumDescriptor.hpp"
#include "Utilities.hpp"
#include "rapidcsv.h"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Scene.hpp"
#include "Times.hpp"
using namespace EcoSysLab;
static const char* StateModes[]{ "Default", "Cubic-Bezier" };

void SorghumGrowthDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
    if (ImGui::Button("Instantiate")) {
        auto sorghum = Application::GetLayer<SorghumLayer>()->CreateSorghum(
            std::dynamic_pointer_cast<SorghumGrowthDescriptor>(GetSelf()));
        Application::GetActiveScene()->SetEntityName(
            sorghum, GetSelf()->GetAssetRecord().lock()->GetAssetFileName());
    }
    static bool autoSave = false;
    ImGui::Checkbox("Auto save", &autoSave);
    if (!autoSave) {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
        ImGui::Text("[Auto save disabled!]");
        ImGui::PopStyleColor();
    }
    else {
        static double lastAutoSaveTime = 0;
        static float autoSaveInterval = 10;
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
    if (!m_saved) {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
        ImGui::Text("[Changed unsaved!]");
        ImGui::PopStyleColor();
    }
    bool changed = false;
    FileUtils::OpenFile(
        "Import CSV", "CSV", { ".csv", ".CSV" },
        [&](const std::filesystem::path& path) { changed = ImportCSV(path); },
        false);

    if (ImGui::Combo("Mode", &m_mode, StateModes, IM_ARRAYSIZE(StateModes))) {
        changed = false;
    }
    if (ImGui::TreeNodeEx("States", ImGuiTreeNodeFlags_DefaultOpen)) {
        float startTime =
            m_sorghumStates.empty() ? 1.0f : m_sorghumStates.begin()->first;
        if (startTime >= 0.01f) {
            if (ImGui::Button("New start state")) {
                changed = true;
                if (m_sorghumStates.empty()) {
                    Add(0.0f, SorghumState());
                }
                else {
                    Add(0.0f, m_sorghumStates.begin()->second);
                }
            }
        }

        float previousTime = 0.0f;
        int stateIndex = 1;
        for (auto it = m_sorghumStates.begin(); it != m_sorghumStates.end(); ++it) {
            if (ImGui::TreeNodeEx(
                ("State " + std::to_string(stateIndex) + ": " + it->second.m_name)
                .c_str())) {
                const std::string tag = "##SorghumState" + std::to_string(stateIndex);
                if (ImGui::BeginPopupContextItem(tag.c_str())) {
                    if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
                        static char newName[256];
                        ImGui::InputText(("New name" + tag).c_str(), newName, 256);
                        if (ImGui::Button(("Confirm" + tag).c_str())) {
                            it->second.m_name = newName;
                            memset(newName, 0, 256);
                        }
                        ImGui::EndMenu();
                    }
                    ImGui::EndPopup();
                }
                if (stateIndex != 1) {
                    if (ImGui::Button("Copy prev leaves shape")) {
                        for (int i = 0; i < (it - 1)->second.m_leaves.size() &&
                            i < it->second.m_leaves.size();
                            i++) {
                            it->second.m_leaves[i].CopyShape((it - 1)->second.m_leaves[i]);
                            it->second.m_saved = false;
                            changed = true;
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Duplicate prev")) {
                        it->second = (it - 1)->second;
                        it->second.m_saved = false;
                        for (auto& leafState : it->second.m_leaves)
                            leafState.m_saved = false;
                        it->second.m_panicle.m_saved = false;
                        it->second.m_stem.m_saved = false;
                        changed = true;
                    }
                }
                if (it != (--m_sorghumStates.end())) {
                    auto tit = it;
                    ++tit;
                    float nextTime = tit->first - 0.01f;
                    float currentTime = it->first;
                    if (ImGui::InputFloat("Time", &currentTime)) {
                        it->first = glm::clamp(currentTime, previousTime, nextTime);
                        changed = true;
                    }

                }
                else {
                    float currentTime = it->first;
                    if (ImGui::InputFloat("Time", &currentTime)) {
                        it->first = glm::clamp(currentTime, previousTime, 99999.0f);
                        changed = true;
                    }
                }

                if (it->second.OnInspect(m_mode)) {
                    changed = true;
                }

                ImGui::TreePop();
            }
            previousTime = it->first + 0.01f;
            stateIndex++;
        }

        if (!m_sorghumStates.empty()) {
            if (ImGui::Button("New end state")) {
                changed = true;
                float endTime = (--m_sorghumStates.end())->first;
                Add(endTime + 0.01f, (--m_sorghumStates.end())->second);
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove end state")) {
                changed = true;
                m_sorghumStates.erase(--m_sorghumStates.end());
            }
        }
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Import state...")) {
        static int seed = 0;
        ImGui::DragInt("Using seed", &seed);
        static AssetRef descriptor;
        editorLayer->DragAndDropButton<SorghumDescriptor>(
            descriptor, "Drag SPD here to add end state");
        auto temp = descriptor.Get<SorghumDescriptor>();
        if (temp) {
            float endTime =
                m_sorghumStates.empty() ? -0.01f : (--m_sorghumStates.end())->first;
            Add(endTime + 0.01f, temp->Generate(seed));
            descriptor.Clear();
            changed = true;
        }
        ImGui::TreePop();
    }

    if (changed) {
        m_saved = false;
        m_version++;
    }
}

void SorghumGrowthDescriptor::Serialize(YAML::Emitter& out) {
    out << YAML::Key << "m_mode" << YAML::Value << m_mode;
    out << YAML::Key << "m_version" << YAML::Value << m_version;
    out << YAML::Key << "m_sorghumStates" << YAML::Value << YAML::BeginSeq;
    for (auto& state : m_sorghumStates) {
        out << YAML::BeginMap;
        out << YAML::Key << "Time" << YAML::Value << state.first;
        state.second.Serialize(out);
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
}

void SorghumGrowthDescriptor::Deserialize(const YAML::Node& in) {
    if (in["m_mode"])
        m_mode = in["m_mode"].as<int>();
    if (in["m_version"])
        m_version = in["m_version"].as<unsigned>();
    if (in["m_sorghumStates"]) {
        m_sorghumStates.clear();
        for (const auto& inState : in["m_sorghumStates"]) {
            SorghumState state;
            state.Deserialize(inState);
            m_sorghumStates.emplace_back(inState["Time"].as<float>(), state);
        }
    }
}

void SorghumGrowthDescriptor::Add(float time, const SorghumState& state) {
    for (auto it = m_sorghumStates.begin(); it != m_sorghumStates.end(); ++it) {
        if (it->first == time) {
            it->second = state;
            return;
        }
        if (it->first > time) {
            m_sorghumStates.insert(it, { time, state });
            return;
        }
    }
    m_sorghumStates.emplace_back(time, state);
    m_sorghumStates.back().second.m_name = "Unnamed";
}

void SorghumGrowthDescriptor::ResetTime(float previousTime, float newTime) {
    for (auto& i : m_sorghumStates) {
        if (i.first == previousTime) {
            i.first = newTime;
            return;
        }
    }
    EVOENGINE_ERROR("Failed: State at previous time not exists!");
}
void SorghumGrowthDescriptor::Remove(float time) {
    for (auto it = m_sorghumStates.begin(); it != m_sorghumStates.end(); ++it) {
        if (it->first == time) {
            m_sorghumStates.erase(it);
            return;
        }
    }
}
float SorghumGrowthDescriptor::GetCurrentStartTime() const {
    if (m_sorghumStates.empty()) {
        return 0.0f;
    }
    return m_sorghumStates.begin()->first;
}
float SorghumGrowthDescriptor::GetCurrentEndTime() const {
    if (m_sorghumStates.empty()) {
        return 0.0f;
    }
    return (--m_sorghumStates.end())->first;
}

unsigned SorghumGrowthDescriptor::GetVersion() const { return m_version; }

bool SorghumGrowthDescriptor::ImportCSV(const std::filesystem::path& filePath) {
    try {
        rapidcsv::Document doc(filePath.string());
        std::vector<std::string> timePoints =
            doc.GetColumn<std::string>("Time Point");
        std::vector<float> stemHeights = doc.GetColumn<float>("Stem Height");
        std::vector<float> stemWidth = doc.GetColumn<float>("Stem Width");
        std::vector<float> leafIndex = doc.GetColumn<float>("Leaf Number");
        std::vector<float> leafLength = doc.GetColumn<float>("Leaf Length");
        std::vector<float> leafWidth = doc.GetColumn<float>("Leaf Width");
        std::vector<float> leafHeight = doc.GetColumn<float>("Leaf Height");
        std::vector<float> startingPoint = doc.GetColumn<float>("Start Point");
        std::vector<float> branchingAngle = doc.GetColumn<float>("Branching Angle");
        std::vector<float> panicleLength = doc.GetColumn<float>("Panicle Height");
        std::vector<float> panicleWidth = doc.GetColumn<float>("Panicle Width");

        m_sorghumStates.clear();

        std::map<std::string, std::pair<int, int>> columnIndices;
        int currentIndex = 0;
        for (int row = 0; row < timePoints.size(); row++) {
            auto& timePoint = timePoints[row];
            if (columnIndices.find(timePoint) == columnIndices.end()) {
                columnIndices[timePoint].first = currentIndex;
                currentIndex++;
            }
            if (columnIndices[timePoint].second < leafIndex[row])
                columnIndices[timePoint].second = leafIndex[row];
        }

        m_sorghumStates.resize(currentIndex);
        for (int row = 0; row < timePoints.size(); row++) {
            int stateIndex = columnIndices.at(timePoints[row]).first;
            auto& statePair = m_sorghumStates[stateIndex];
            auto& state = statePair.second;
            if (state.m_leaves.empty()) {
                statePair.first = stateIndex;
                state.m_name = timePoints[row];
                state.m_leaves.resize(columnIndices.at(timePoints[row]).second);
                for (auto& leaf : state.m_leaves)
                    leaf.m_dead = true;
                state.m_stem.m_length = stemHeights[row] / 100.0f;
                state.m_stem.m_widthAlongStem.m_minValue = 0.0f;
                state.m_stem.m_widthAlongStem.m_maxValue = stemWidth[row] * 2.0f;
                state.m_panicle.m_panicleSize.x = state.m_panicle.m_panicleSize.z = panicleWidth[row] / 100.0f;
                state.m_panicle.m_panicleSize.y = panicleLength[row] / 100.0f;
                state.m_panicle.m_seedAmount = state.m_panicle.m_panicleSize.x * state.m_panicle.m_panicleSize.y * state.m_panicle.m_panicleSize.z / 0.001f;
            }
            auto& leaf = state.m_leaves[leafIndex[row] - 1];
            leaf.m_index = leafIndex[row] - 1;
            leaf.m_length = leafLength[row] / 100.0f;
            if (leaf.m_length == 0)
                leaf.m_dead = true;
            else {
                leaf.m_dead = false;
                leaf.m_rollAngle = leaf.m_index % 2 * 180.0f;
                leaf.m_widthAlongLeaf.m_maxValue = leafWidth[row] / 100.0f;
                leaf.m_startingPoint = leafHeight[row] / stemHeights[row];
                leaf.m_branchingAngle = branchingAngle[row];
            }
        }

        for (auto& sorghumState : m_sorghumStates) {
            sorghumState.second.m_saved = false;
            int leafIndex = 0;
            for (auto& leafState : sorghumState.second.m_leaves) {
                leafState.m_saved = false;
                leafState.m_index = leafIndex;
                leafIndex++;
            }
            sorghumState.second.m_stem.m_saved = false;
            sorghumState.second.m_panicle.m_saved = false;
        }
        m_saved = false;
    }
    catch (std::exception e) {
        return false;
    }
    return true;
}

glm::vec3 SorghumStemState::GetPoint(float point) const {
    return m_direction * point * m_length;
}

int SorghumStatePair::GetLeafSize() const {
    if (m_left.m_leaves.size() <= m_right.m_leaves.size()) {
        return m_left.m_leaves.size() +
            glm::ceil((m_right.m_leaves.size() - m_left.m_leaves.size()) * m_a);
    }
    else
        return m_left.m_leaves.size();
}
float SorghumStatePair::GetStemLength() const {
    float leftLength, rightLength;
    switch ((StateMode)m_mode) {
    case StateMode::Default:
        leftLength = m_left.m_stem.m_length;
        rightLength = m_right.m_stem.m_length;
        break;
    case StateMode::CubicBezier:
        if (!m_left.m_stem.m_spline.m_curves.empty()) {
            leftLength = glm::distance(m_left.m_stem.m_spline.m_curves.front().m_p0,
                m_left.m_stem.m_spline.m_curves.back().m_p3);
        }
        else {
            leftLength = 0.0f;
        }
        if (!m_right.m_stem.m_spline.m_curves.empty()) {
            rightLength = glm::distance(m_right.m_stem.m_spline.m_curves.front().m_p0,
                m_right.m_stem.m_spline.m_curves.back().m_p3);
        }
        else {
            rightLength = 0.0f;
        }
        break;
    }
    return glm::mix(leftLength, rightLength, m_a);
}
glm::vec3 SorghumStatePair::GetStemDirection() const {
    glm::vec3 leftDir, rightDir;
    switch ((StateMode)m_mode) {
    case StateMode::Default:
        leftDir = glm::normalize(m_left.m_stem.m_direction);
        rightDir = glm::normalize(m_right.m_stem.m_direction);
        break;
    case StateMode::CubicBezier:
        if (!m_left.m_stem.m_spline.m_curves.empty()) {
            leftDir = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        else {
            leftDir = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        if (!m_right.m_stem.m_spline.m_curves.empty()) {
            rightDir = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        else {
            rightDir = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        break;
    }

    return glm::normalize(glm::mix(leftDir, rightDir, m_a));
}
glm::vec3 SorghumStatePair::GetStemPoint(float point) const {
    glm::vec3 leftPoint, rightPoint;
    switch ((StateMode)m_mode) {
    case StateMode::Default:
        leftPoint = glm::normalize(m_left.m_stem.m_direction) * point * m_left.m_stem.m_length;
        rightPoint = glm::normalize(m_right.m_stem.m_direction) * point * m_right.m_stem.m_length;
        break;
    case StateMode::CubicBezier:
        if (!m_left.m_stem.m_spline.m_curves.empty()) {
            leftPoint = m_left.m_stem.m_spline.EvaluatePointFromCurves(point);
        }
        else {
            leftPoint = glm::vec3(0.0f, 0.0f, 0.0f);
        }
        if (!m_right.m_stem.m_spline.m_curves.empty()) {
            rightPoint = m_right.m_stem.m_spline.EvaluatePointFromCurves(point);
        }
        else {
            rightPoint = glm::vec3(0.0f, 0.0f, 0.0f);
        }
        break;
    }

    return glm::mix(leftPoint, rightPoint, m_a);
}