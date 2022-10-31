//
// Created by lllll on 10/27/2022.
//

#include "BranchMeshGenerator.hpp"
#include "Curve.hpp"

using namespace EcoSysLab;

RingSegment::RingSegment(glm::vec3 startPosition, glm::vec3 endPosition, glm::vec3 startAxis,
                         glm::vec3 endAxis, float startRadius, float endRadius)
        : m_startPosition(startPosition),
          m_endPosition(endPosition),
          m_startAxis(startAxis),
          m_endAxis(endAxis),
          m_startRadius(startRadius),
          m_endRadius(endRadius) {
}

void RingSegment::AppendPoints(std::vector<Vertex> &vertices, glm::vec3 &normalDir, int step) {
    std::vector<Vertex> startRing;
    std::vector<Vertex> endRing;

    float angleStep = 360.0f / (float) (step);
    Vertex archetype;
    for (int i = 0; i < step; i++) {
        archetype.m_position = GetPoint(normalDir, angleStep * i, true);
        startRing.push_back(archetype);
    }
    for (int i = 0; i < step; i++) {
        archetype.m_position = GetPoint(normalDir, angleStep * i, false);
        endRing.push_back(archetype);
    }
    float textureXstep = 1.0f / step * 4;
    for (int i = 0; i < step - 1; i++) {
        float x = (i % step) * textureXstep;
        startRing[i].m_texCoord = glm::vec2(x, 0.0f);
        startRing[i + 1].m_texCoord = glm::vec2(x + textureXstep, 0.0f);
        endRing[i].m_texCoord = glm::vec2(x, 1.0f);
        endRing[i + 1].m_texCoord = glm::vec2(x + textureXstep, 1.0f);
        vertices.push_back(startRing[i]);
        vertices.push_back(startRing[i + 1]);
        vertices.push_back(endRing[i]);
        vertices.push_back(endRing[i + 1]);
        vertices.push_back(endRing[i]);
        vertices.push_back(startRing[i + 1]);
    }
    startRing[step - 1].m_texCoord = glm::vec2(1.0f - textureXstep, 0.0f);
    startRing[0].m_texCoord = glm::vec2(1.0f, 0.0f);
    endRing[step - 1].m_texCoord = glm::vec2(1.0f - textureXstep, 1.0f);
    endRing[0].m_texCoord = glm::vec2(1.0f, 1.0f);
    vertices.push_back(startRing[step - 1]);
    vertices.push_back(startRing[0]);
    vertices.push_back(endRing[step - 1]);
    vertices.push_back(endRing[0]);
    vertices.push_back(endRing[step - 1]);
    vertices.push_back(startRing[0]);
}

glm::vec3 RingSegment::GetPoint(glm::vec3 &normalDir, float angle, bool isStart) {
    glm::vec3 direction = glm::cross(normalDir, isStart ? this->m_startAxis : this->m_endAxis);
    direction = glm::rotate(direction, glm::radians(angle), isStart ? this->m_startAxis : this->m_endAxis);
    direction = glm::normalize(direction);
    const glm::vec3 position = (isStart ? m_startPosition : m_endPosition) + direction * (
            isStart ? m_startRadius : m_endRadius);
    return position;
}

void MeshGeneratorSettings::Save(const std::string &name, YAML::Emitter &out) {
    out << YAML::Key << name << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "m_resolution" << YAML::Value << m_resolution;
    out << YAML::Key << "m_subdivision" << YAML::Value << m_subdivision;
    out << YAML::Key << "m_vertexColorOnly" << YAML::Value << m_vertexColorOnly;
    out << YAML::Key << "m_enableFoliage" << YAML::Value << m_enableFoliage;
    out << YAML::Key << "m_enableBranch" << YAML::Value << m_enableBranch;
    out << YAML::Key << "m_smoothness" << YAML::Value << m_smoothness;
    out << YAML::Key << "m_overrideRadius" << YAML::Value << m_overrideRadius;
    out << YAML::Key << "m_boundaryRadius" << YAML::Value << m_radius;
    out << YAML::Key << "m_internodeLengthFactor" << YAML::Value << m_internodeLengthFactor;
    out << YAML::Key << "m_overrideVertexColor" << YAML::Value << m_overrideVertexColor;
    out << YAML::Key << "m_markJunctions" << YAML::Value << m_markJunctions;
    out << YAML::Key << "m_junctionUpperRatio" << YAML::Value << m_junctionUpperRatio;
    out << YAML::Key << "m_junctionLowerRatio" << YAML::Value << m_junctionLowerRatio;
    out << YAML::Key << "m_branchVertexColor" << YAML::Value << m_branchVertexColor;
    out << YAML::Key << "m_foliageVertexColor" << YAML::Value << m_foliageVertexColor;
    out << YAML::EndMap;
}

void MeshGeneratorSettings::Load(const std::string &name, const YAML::Node &in) {
    if (in[name]) {
        const auto &ms = in[name];
        if (ms["m_resolution"]) m_resolution = ms["m_resolution"].as<float>();
        if (ms["m_subdivision"]) m_subdivision = ms["m_subdivision"].as<float>();
        if (ms["m_vertexColorOnly"]) m_vertexColorOnly = ms["m_vertexColorOnly"].as<bool>();
        if (ms["m_enableFoliage"]) m_enableFoliage = ms["m_enableFoliage"].as<bool>();
        if (ms["m_enableBranch"]) m_enableBranch = ms["m_enableBranch"].as<bool>();
        if (ms["m_smoothness"]) m_smoothness = ms["m_smoothness"].as<bool>();
        if (ms["m_overrideRadius"]) m_overrideRadius = ms["m_overrideRadius"].as<bool>();
        if (ms["m_boundaryRadius"]) m_radius = ms["m_boundaryRadius"].as<float>();
        if (ms["m_internodeLengthFactor"]) m_internodeLengthFactor = ms["m_internodeLengthFactor"].as<float>();
        if (ms["m_overrideVertexColor"]) m_overrideVertexColor = ms["m_overrideVertexColor"].as<bool>();
        if (ms["m_markJunctions"]) m_markJunctions = ms["m_markJunctions"].as<bool>();
        if (ms["m_junctionUpperRatio"]) m_junctionUpperRatio = ms["m_junctionUpperRatio"].as<float>();
        if (ms["m_junctionLowerRatio"]) m_junctionLowerRatio = ms["m_junctionLowerRatio"].as<float>();
        if (ms["m_branchVertexColor"]) m_branchVertexColor = ms["m_branchVertexColor"].as<glm::vec3>();
        if (ms["m_foliageVertexColor"]) m_foliageVertexColor = ms["m_foliageVertexColor"].as<glm::vec3>();
    }
}

void MeshGeneratorSettings::OnInspect() {
    if (ImGui::TreeNodeEx("Mesh Generator settings")) {
        ImGui::DragFloat("Resolution", &m_resolution, 0.001f);
        ImGui::DragFloat("Subdivision", &m_subdivision, 0.001f);
        ImGui::Checkbox("Vertex color only", &m_vertexColorOnly);
        ImGui::Checkbox("Foliage", &m_enableFoliage);
        ImGui::Checkbox("Branch", &m_enableBranch);
        ImGui::Checkbox("Smoothness", &m_smoothness);
        if (!m_smoothness) {
            ImGui::DragFloat("Internode length factor", &m_internodeLengthFactor, 0.001f, 0.0f, 1.0f);
        }
        ImGui::Checkbox("Override radius", &m_overrideRadius);
        if (m_overrideRadius) ImGui::DragFloat("Radius", &m_radius);
        ImGui::Checkbox("Override vertex color", &m_overrideVertexColor);
        if (m_overrideVertexColor) {
            ImGui::ColorEdit3("Branch vertex color", &m_branchVertexColor.x);
            ImGui::ColorEdit3("Foliage vertex color", &m_foliageVertexColor.x);
        }
        ImGui::Checkbox("Mark Junctions", &m_markJunctions);
        if (m_markJunctions) {
            ImGui::DragFloat("Junction Lower Ratio", &m_junctionLowerRatio, 0.01f, 0.0f, 0.5f);
            ImGui::DragFloat("Junction Upper Ratio", &m_junctionUpperRatio, 0.01f, 0.0f, 0.5f);
        }
        ImGui::TreePop();
    }
}

void
BranchMeshGenerator::Generate(TreeSkeleton<BranchGrowthData, InternodeGrowthData> &treeSkeleton, std::vector<Vertex> &vertices,
                              std::vector<unsigned int> &indices, const MeshGeneratorSettings &settings) {
    int parentStep = -1;
    const auto &sortedInternodeList = treeSkeleton.RefSortedInternodeList();
    std::vector<std::vector<RingSegment>> ringsList;
    std::vector<int> steps;
    ringsList.resize(sortedInternodeList.size());
    steps.resize(sortedInternodeList.size());

    std::vector<std::shared_future<void>> results;
    Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i) {
        auto internodeHandle = sortedInternodeList[i];
        auto &internode = treeSkeleton.RefInternode(internodeHandle);
        auto &internodeInfo = internode.m_info;
        auto &rings = ringsList[i];
        rings.clear();

        glm::vec3 directionStart =
                internodeInfo.m_globalRotation * glm::vec3(0, 0, -1);
        glm::vec3 directionEnd = directionStart;
        glm::vec3 positionStart = internodeInfo.m_globalPosition;
        glm::vec3 positionEnd =
                positionStart + internodeInfo.m_length * settings.m_internodeLengthFactor * directionStart;
        float thicknessStart = internodeInfo.m_thickness;
        float thicknessEnd = internodeInfo.m_thickness;

        if (internode.m_parent != -1) {
            auto &parentInternode = treeSkeleton.RefInternode(internode.m_parent);
            thicknessStart = parentInternode.m_info.m_thickness;
            GlobalTransform parentRelativeGlobalTransform;
            directionStart =
                    parentInternode.m_info.m_globalRotation *
                    glm::vec3(0, 0, -1);
        }

        if (settings.m_overrideRadius) {
            thicknessStart = settings.m_radius;
            thicknessEnd = settings.m_radius;
        }
#pragma region Subdivision internode here.
        int step = thicknessStart / settings.m_resolution;
        if (step < 4)
            step = 4;
        if (step % 2 != 0)
            step++;
        steps[i] = step;
        int amount = static_cast<int>(0.5f +
                internodeInfo.m_length * settings.m_subdivision);
        if (amount % 2 != 0)
            amount++;
        BezierCurve curve = BezierCurve(
                positionStart,
                positionStart +
                (settings.m_smoothness ? internodeInfo.m_length / 3.0f : 0.0f) * directionStart,
                positionEnd -
                (settings.m_smoothness ? internodeInfo.m_length / 3.0f : 0.0f) * directionEnd,
                positionEnd);
        float posStep = 1.0f / static_cast<float>(amount);
        glm::vec3 dirStep = (directionEnd - directionStart) / static_cast<float>(amount);
        float radiusStep = (thicknessEnd - thicknessStart) /
                           static_cast<float>(amount);

        for (int ringIndex = 1; ringIndex < amount; ringIndex++) {
            float startThickness = static_cast<float>(ringIndex - 1) * radiusStep;
            float endThickness = static_cast<float>(ringIndex) * radiusStep;
            if (settings.m_smoothness) {
                rings.emplace_back(
                        curve.GetPoint(posStep * (ringIndex - 1)), curve.GetPoint(posStep * ringIndex),
                        directionStart + static_cast<float>(ringIndex - 1) * dirStep,
                        directionStart + static_cast<float>(ringIndex) * dirStep,
                        thicknessStart + startThickness, thicknessStart + endThickness);
            } else {
                rings.emplace_back(
                        curve.GetPoint(posStep * (ringIndex - 1)), curve.GetPoint(posStep * ringIndex),
                        directionEnd,
                        directionEnd,
                        thicknessStart + startThickness, thicknessStart + endThickness);
            }
        }
        if (amount > 1)
            rings.emplace_back(
                    curve.GetPoint(1.0f - posStep), positionEnd, directionEnd - dirStep,
                    directionEnd,
                    thicknessEnd - radiusStep,
                    thicknessEnd);
        else
            rings.emplace_back(positionStart, positionEnd,
                               directionStart, directionEnd, thicknessStart,
                               thicknessEnd);
#pragma endregion
    }, results);
    for (auto &i: results) i.wait();

    std::map<unsigned, glm::vec3> normals;
    for (int i = 0; i < sortedInternodeList.size(); i++) {
        auto internodeHandle = sortedInternodeList[i];
        auto &internode = treeSkeleton.RefInternode(internodeHandle);
        auto &internodeInfo = internode.m_info;
        auto parentInternodeHandle = internode.m_parent;
        glm::vec3 newNormalDir;
        if (parentInternodeHandle != -1) {
            newNormalDir = normals.at(parentInternodeHandle);
        } else {
            newNormalDir = internodeInfo.m_globalRotation * glm::vec3(1, 0, 0);
        }
        const glm::vec3 front = internodeInfo.m_globalRotation * glm::vec3(0.0f, 0.0f, -1.0f);
        newNormalDir = glm::cross(glm::cross(front, newNormalDir), front);
        normals[internodeHandle] = newNormalDir;
        auto &rings = ringsList[i];
        if (rings.empty()) {
            continue;
        }
        auto step = steps[i];
        // For stitching
        const int pStep = parentStep > 0 ? parentStep : step;
        parentStep = step;

        float angleStep = 360.0f / static_cast<float>(pStep);
        int vertexIndex = vertices.size();
        Vertex archetype;
        if (settings.m_overrideVertexColor) archetype.m_color = settings.m_branchVertexColor;
        //else archetype.m_color = branchColors.at(internodeHandle);

        float textureXStep = 1.0f / pStep * 4.0f;

        const auto startPosition = rings.at(0).m_startPosition;
        const auto endPosition = rings.back().m_endPosition;
        for (int i = 0; i < pStep; i++) {
            archetype.m_position =
                    rings.at(0).GetPoint(newNormalDir, angleStep * i, true);
            float distanceToStart = 0;
            float distanceToEnd = 1;
            const float x =
                    i < pStep / 2 ? i * textureXStep : (pStep - i) * textureXStep;
            archetype.m_texCoord = glm::vec2(x, 0.0f);
            vertices.push_back(archetype);
        }
        std::vector<float> angles;
        angles.resize(step);
        std::vector<float> pAngles;
        pAngles.resize(pStep);

        for (auto i = 0; i < pStep; i++) {
            pAngles[i] = angleStep * i;
        }
        angleStep = 360.0f / static_cast<float>(step);
        for (auto i = 0; i < step; i++) {
            angles[i] = angleStep * i;
        }

        std::vector<unsigned> pTarget;
        std::vector<unsigned> target;
        pTarget.resize(pStep);
        target.resize(step);
        for (int i = 0; i < pStep; i++) {
            // First we allocate nearest vertices for parent.
            auto minAngleDiff = 360.0f;
            for (auto j = 0; j < step; j++) {
                const float diff = glm::abs(pAngles[i] - angles[j]);
                if (diff < minAngleDiff) {
                    minAngleDiff = diff;
                    pTarget[i] = j;
                }
            }
        }
        for (int i = 0; i < step; i++) {
            // Second we allocate nearest vertices for child
            float minAngleDiff = 360.0f;
            for (int j = 0; j < pStep; j++) {
                const float diff = glm::abs(angles[i] - pAngles[j]);
                if (diff < minAngleDiff) {
                    minAngleDiff = diff;
                    target[i] = j;
                }
            }
        }
        for (int i = 0; i < pStep; i++) {
            if (pTarget[i] == pTarget[i == pStep - 1 ? 0 : i + 1]) {
                indices.push_back(vertexIndex + i);
                indices.push_back(vertexIndex + (i == pStep - 1 ? 0 : i + 1));
                indices.push_back(vertexIndex + pStep + pTarget[i]);
            } else {
                indices.push_back(vertexIndex + i);
                indices.push_back(vertexIndex + (i == pStep - 1 ? 0 : i + 1));
                indices.push_back(vertexIndex + pStep + pTarget[i]);

                indices.push_back(vertexIndex + pStep +
                                  pTarget[i == pStep - 1 ? 0 : i + 1]);
                indices.push_back(vertexIndex + pStep + pTarget[i]);
                indices.push_back(vertexIndex + (i == pStep - 1 ? 0 : i + 1));
            }
        }

        vertexIndex += pStep;
        textureXStep = 1.0f / step * 4.0f;
        int ringSize = rings.size();
        for (auto ringIndex = 0; ringIndex < ringSize; ringIndex++) {
            for (auto i = 0; i < step; i++) {
                archetype.m_position = rings.at(ringIndex).GetPoint(
                        newNormalDir, angleStep * i, false);
                float distanceToStart = glm::distance(
                        rings.at(ringIndex).m_endPosition, startPosition);
                float distanceToEnd = glm::distance(
                        rings.at(ringIndex).m_endPosition, endPosition);
                const auto x =
                        i < (step / 2) ? i * textureXStep : (step - i) * textureXStep;
                const auto y = ringIndex % 2 == 0 ? 1.0f : 0.0f;
                archetype.m_texCoord = glm::vec2(x, y);
                vertices.push_back(archetype);
            }
            if (ringIndex != 0) {
                for (int i = 0; i < step - 1; i++) {
                    // Down triangle
                    indices.push_back(vertexIndex + (ringIndex - 1) * step + i);
                    indices.push_back(vertexIndex + (ringIndex - 1) * step + i + 1);
                    indices.push_back(vertexIndex + (ringIndex) * step + i);
                    // Up triangle
                    indices.push_back(vertexIndex + (ringIndex) * step + i + 1);
                    indices.push_back(vertexIndex + (ringIndex) * step + i);
                    indices.push_back(vertexIndex + (ringIndex - 1) * step + i + 1);
                }
                // Down triangle
                indices.push_back(vertexIndex + (ringIndex - 1) * step + step - 1);
                indices.push_back(vertexIndex + (ringIndex - 1) * step);
                indices.push_back(vertexIndex + (ringIndex) * step + step - 1);
                // Up triangle
                indices.push_back(vertexIndex + (ringIndex) * step);
                indices.push_back(vertexIndex + (ringIndex) * step + step - 1);
                indices.push_back(vertexIndex + (ringIndex - 1) * step);
            }
        }
    }
}
