//
// Created by lllll on 10/21/2022.
//

#include "TreeModel/TreeModel.hpp"

using namespace EcoSysLab;

void ApplyTropism(const glm::vec3 &targetDir, float tropism, glm::vec3 &front, glm::vec3 &up) {
    const glm::vec3 dir = glm::normalize(targetDir);
    const float dotP = glm::abs(glm::dot(front, dir));
    if (dotP < 0.99f && dotP > -0.99f) {
        const glm::vec3 left = glm::cross(front, dir);
        const float maxAngle = glm::acos(dotP);
        const float rotateAngle = maxAngle * tropism;
        front = glm::normalize(
                glm::rotate(front, glm::min(maxAngle, rotateAngle), left));
        up = glm::normalize(glm::cross(glm::cross(front, up), front));
    }
}

void ApplyTropism(const glm::vec3 &targetDir, float tropism, glm::quat &rotation) {
    auto front = rotation * glm::vec3(0, 0, -1);
    auto up = rotation * glm::vec3(0, 1, 0);
    ApplyTropism(targetDir, tropism, front, up);
    rotation = glm::quatLookAt(front, up);
}

bool TreeModel::GrowShoots(float extendLength, InternodeHandle internodeHandle,
                           const TreeStructuralGrowthParameters &parameters, float &collectedInhibitor) {
    bool graphChanged = false;
    auto &skeleton = m_treeStructure.Skeleton();
    auto &internode = skeleton.RefInternode(internodeHandle);
    auto internodeLength = parameters.GetInternodeLength(internode);
    auto &internodeData = internode.m_data;
    auto &internodeInfo = internode.m_info;
    internodeInfo.m_length += extendLength;
    float extraLength = internodeInfo.m_length - internodeLength;
    auto &apicalBud = internodeData.m_buds.front();
    //If we need to add a new end node
    if (extraLength > 0) {
        graphChanged = true;
        apicalBud.m_status = BudStatus::Died;
        internodeInfo.m_length = internodeLength;
        auto desiredGlobalRotation = internodeInfo.m_globalRotation * apicalBud.m_localRotation;
        auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
        auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
        ApplyTropism(m_gravityDirection, parameters.GetGravitropism(internode), desiredGlobalFront,
                     desiredGlobalUp);
        ApplyTropism(internodeData.m_lightDirection, parameters.GetPhototropism(internode),
                     desiredGlobalFront, desiredGlobalUp);
        auto lateralBudCount = parameters.GetLateralBudCount(internode);
        //Allocate Lateral bud for current internode
        float turnAngle = glm::radians(360.0f / lateralBudCount);
        for (int i = 0; i < lateralBudCount; i++) {
            internodeData.m_buds.emplace_back();
            auto &lateralBud = internodeData.m_buds.back();
            lateralBud.m_type = BudType::Lateral;
            lateralBud.m_status = BudStatus::Dormant;
            lateralBud.m_localRotation = glm::vec3(
                    glm::radians(parameters.GetDesiredBranchingAngle(internode)), 0.0f,
                    i * turnAngle);
        }

        //Create new internode
        auto newInternodeHandle = skeleton.Extend(internodeHandle, false);
        auto &oldInternode = skeleton.RefInternode(internodeHandle);
        auto &newInternode = skeleton.RefInternode(newInternodeHandle);
        newInternode.m_data.Clear();
        newInternode.m_data.m_inhibitor = parameters.GetApicalDominanceBase(newInternode);
        newInternode.m_info.m_length = glm::clamp(extendLength, 0.0f, internodeLength);
        newInternode.m_info.m_thickness = parameters.GetEndNodeThickness(newInternode);
        newInternode.m_info.m_localRotation = newInternode.m_data.m_desiredLocalRotation =
                glm::inverse(oldInternode.m_info.m_globalRotation) *
                glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
        newInternode.m_info.m_globalRotation =
                oldInternode.m_info.m_globalRotation * newInternode.m_info.m_localRotation;
        //Allocate apical bud for new internode
        newInternode.m_data.m_buds.emplace_back();
        auto &newApicalBud = newInternode.m_data.m_buds.back();
        newApicalBud.m_type = BudType::Apical;
        newApicalBud.m_status = BudStatus::Dormant;
        newApicalBud.m_localRotation = glm::vec3(
                glm::radians(parameters.GetDesiredApicalAngle(newInternode)), 0.0f,
                parameters.GetDesiredRollAngle(newInternode));
        if (extraLength > internodeLength) {
            float childInhibitor = 0.0f;
            GrowShoots(extraLength - internodeLength, newInternodeHandle, parameters, childInhibitor);
            childInhibitor *= parameters.GetApicalDominanceDecrease(newInternode);
            collectedInhibitor += childInhibitor;
            skeleton.RefInternode(newInternodeHandle).m_data.m_inhibitor = childInhibitor;
        } else {
            collectedInhibitor += parameters.GetApicalDominanceBase(newInternode);
        }
    } else {
        //Otherwise, we add the inhibitor.
        collectedInhibitor += parameters.GetApicalDominanceBase(internode);
    }
    return graphChanged;
}

bool TreeModel::GrowInternode(InternodeHandle internodeHandle, const TreeStructuralGrowthParameters &parameters,
                              const GrowthNutrients &growthNutrients) {
    bool graphChanged = false;
    auto &skeleton = m_treeStructure.Skeleton();
    auto &buds = skeleton.RefInternode(internodeHandle).m_data.m_buds;
    {
        auto &internode = skeleton.RefInternode(internodeHandle);
        auto &internodeData = internode.m_data;
        internodeData.m_inhibitor = 0;
        for (const auto &childHandle: internode.RefChildHandles()) {
            internodeData.m_inhibitor += skeleton.RefInternode(childHandle).m_data.m_inhibitor *
                                         parameters.GetApicalDominanceDecrease(internode);
        }
    }
    for (auto &bud: buds) {
        auto &internode = skeleton.RefInternode(internodeHandle);
        auto &internodeData = internode.m_data;
        auto &internodeInfo = internode.m_info;
        if (bud.m_type == BudType::Apical && bud.m_status == BudStatus::Dormant) {
            if (parameters.GetApicalBudKillProbability(internode) > glm::linearRand(0.0f, 1.0f)) {
                bud.m_status = BudStatus::Died;
            } else {
                float waterReceived = bud.m_productiveResourceRequirement;
                auto growthRate = parameters.GetGrowthRate(internode);
                float elongateLength = waterReceived * parameters.GetInternodeLength(internode) * growthRate;
                float collectedInhibitor = 0.0f;
                auto dd = parameters.GetApicalDominanceDecrease(internode);
                graphChanged = GrowShoots(elongateLength, internodeHandle, parameters, collectedInhibitor);
                skeleton.RefInternode(internodeHandle).m_data.m_inhibitor += collectedInhibitor * dd;
            }
            //If apical bud is dormant, then there's no lateral bud at this stage. We should quit anyway.
            break;
        } else if (bud.m_type == BudType::Lateral) {
            if (bud.m_status == BudStatus::Dormant) {
                if (parameters.GetLateralBudKillProbability(internode) > glm::linearRand(0.0f, 1.0f)) {
                    bud.m_status = BudStatus::Died;
                } else {
                    float flushProbability = parameters.GetLateralBudFlushingProbability(internode) * parameters.GetGrowthRate(internode);
                    flushProbability /= (1.0f + internodeData.m_inhibitor);
                    if (flushProbability >= glm::linearRand(0.0f, 1.0f)) {
                        graphChanged = true;
                        bud.m_status = BudStatus::Flushed;
                        //Prepare information for new internode
                        auto desiredGlobalRotation = internodeInfo.m_globalRotation * bud.m_localRotation;
                        auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
                        auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
                        ApplyTropism(m_gravityDirection, parameters.GetGravitropism(internode), desiredGlobalFront,
                                     desiredGlobalUp);
                        ApplyTropism(internodeData.m_lightDirection, parameters.GetPhototropism(internode),
                                     desiredGlobalFront, desiredGlobalUp);
                        //Create new internode
                        auto newInternodeHandle = skeleton.Extend(internodeHandle, true);
                        auto &oldInternode = skeleton.RefInternode(internodeHandle);
                        auto &newInternode = skeleton.RefInternode(newInternodeHandle);
                        newInternode.m_data.Clear();
                        newInternode.m_info.m_length = 0.0f;
                        newInternode.m_info.m_thickness = parameters.GetEndNodeThickness(internode);
                        newInternode.m_info.m_localRotation = newInternode.m_data.m_desiredLocalRotation =
                                glm::inverse(oldInternode.m_info.m_globalRotation) *
                                glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
                        //Allocate apical bud
                        newInternode.m_data.m_buds.emplace_back();
                        auto &apicalBud = newInternode.m_data.m_buds.back();
                        apicalBud.m_type = BudType::Apical;
                        apicalBud.m_status = BudStatus::Dormant;
                        apicalBud.m_localRotation = glm::vec3(
                                glm::radians(parameters.GetDesiredApicalAngle(internode)), 0.0f,
                                parameters.GetDesiredRollAngle(internode));
                    }
                }
            }
        }
    }
    return graphChanged;
}

void TreeModel::CalculateSagging(InternodeHandle internodeHandle,
                                 const TreeStructuralGrowthParameters &parameters) {
    auto &skeleton = m_treeStructure.Skeleton();
    auto &internode = skeleton.RefInternode(internodeHandle);
    auto &internodeData = internode.m_data;
    auto &internodeInfo = internode.m_info;
    internodeData.m_childTotalBiomass = 0;

    if (!internode.IsEndNode()) {
        //If current node is not end node
        float maxDistanceToAnyBranchEnd = 0;
        float childThicknessCollection = 0.0f;
        for (const auto &i: internode.RefChildHandles()) {
            auto &childInternode = skeleton.RefInternode(i);
            internodeData.m_childTotalBiomass +=
                    childInternode.m_data.m_childTotalBiomass +
                    childInternode.m_info.m_thickness * childInternode.m_info.m_length;
            float childMaxDistanceToAnyBranchEnd =
                    childInternode.m_data.m_maxDistanceToAnyBranchEnd + childInternode.m_info.m_length;
            maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);

            childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
                                                 1.0f / parameters.GetThicknessControlFactor(internode));
        }
        internodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
        internodeData.m_sagging = parameters.GetSagging(internode);
        internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, glm::pow(childThicknessCollection,
                                                                                 parameters.GetThicknessControlFactor(
                                                                                         internode)));
    }
}

void TreeModel::CalculateResourceRequirement(InternodeHandle internodeHandle,
                                             const TreeStructuralGrowthParameters &parameters) {
    auto &skeleton = m_treeStructure.Skeleton();
    auto &internode = skeleton.RefInternode(internodeHandle);
    auto &internodeData = internode.m_data;
    auto &internodeInfo = internode.m_info;
    internodeData.m_productiveResourceRequirement = 0.0f;
    internodeData.m_descendentProductiveResourceRequirement = 0.0f;
    for (auto &bud: internodeData.m_buds) {
        if (bud.m_status == BudStatus::Died) {
            bud.m_baseResourceRequirement = 0.0f;
            bud.m_productiveResourceRequirement = 0.0f;
            continue;
        }
        switch (bud.m_type) {
            case BudType::Apical: {
                if (bud.m_status == BudStatus::Dormant) {
                    bud.m_baseResourceRequirement = parameters.GetShootBaseResourceRequirementFactor(
                            internode);
                    bud.m_productiveResourceRequirement = parameters.GetShootProductiveResourceRequirementFactor(
                            internode);
                }
            }
                break;
            case BudType::Leaf: {
                bud.m_baseResourceRequirement = parameters.GetLeafBaseResourceRequirementFactor(internode);
                bud.m_productiveResourceRequirement = parameters.GetLeafProductiveResourceRequirementFactor(
                        internode);
            }
                break;
            case BudType::Fruit: {
                bud.m_baseResourceRequirement = parameters.GetFruitBaseResourceRequirementFactor(internode);
                bud.m_productiveResourceRequirement = parameters.GetFruitProductiveResourceRequirementFactor(
                        internode);
            }
                break;
            case BudType::Lateral: {
                bud.m_baseResourceRequirement = 0.0f;
                bud.m_productiveResourceRequirement = 0.0f;
            }
                break;
        }

        internodeData.m_productiveResourceRequirement += bud.m_productiveResourceRequirement;
    }

    if (!internode.IsEndNode()) {
        //If current node is not end node
        for (const auto &i: internode.RefChildHandles()) {
            auto &childInternode = skeleton.RefInternode(i);
            internodeData.m_descendentProductiveResourceRequirement += childInternode.m_data.
                    m_productiveResourceRequirement;
        }
    }
}

bool TreeModel::Grow(const GrowthNutrients &growthNutrients, const TreeStructuralGrowthParameters &parameters) {
    auto &skeleton = m_treeStructure.Skeleton();
    bool structureChanged = false;
    bool graphChangedDuringGrowth = false;
#pragma region Low Branch Pruning
    skeleton.SortLists();
    if (!m_initialized) {
        Initialize(parameters);
        graphChangedDuringGrowth = true;
    }
    {
        const auto &sortedInternodeList = skeleton.RefSortedInternodeList();
        const auto maxDistance = skeleton.RefInternode(sortedInternodeList.front()).m_data.m_maxDistanceToAnyBranchEnd;
        for (const auto &internodeHandle: sortedInternodeList) {
            auto &internode = skeleton.RefInternode(internodeHandle);
            //Pruning here.
            if (internode.IsRecycled()) continue;
            const bool graphChanged = LowBranchPruning(maxDistance, internodeHandle, parameters);
            graphChangedDuringGrowth = graphChangedDuringGrowth || graphChanged;
        }

    }
#pragma endregion
#pragma region Grow
    if(graphChangedDuringGrowth) skeleton.SortLists();
    structureChanged = graphChangedDuringGrowth;
    graphChangedDuringGrowth = false;
    {
        const auto &sortedInternodeList = skeleton.RefSortedInternodeList();
        for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
            auto internodeHandle = *it;
            CalculateResourceRequirement(internodeHandle, parameters);
        }
        for (const auto &internodeHandle: sortedInternodeList) {
            auto &internode = skeleton.RefInternode(internodeHandle);
            auto &internodeData = internode.m_data;
            auto &internodeInfo = internode.m_info;
            if (internode.GetParentHandle() == -1) {
                internodeData.m_adjustedTotalProductiveWaterRequirement =
                        internodeData.m_productiveResourceRequirement +
                        internodeData.m_descendentProductiveResourceRequirement;
            }
            float apicalControl = parameters.GetApicalControl(internode);
            float totalChildResourceRequirement = 0.0f;
            for (const auto &i: internode.RefChildHandles()) {
                auto &childInternode = skeleton.RefInternode(i);
                auto &childInternodeData = childInternode.m_data;

                childInternodeData.m_adjustedTotalProductiveWaterRequirement =
                        (childInternodeData.m_adjustedTotalProductiveWaterRequirement +
                         childInternodeData.m_productiveResourceRequirement) /
                        internodeData.m_descendentProductiveResourceRequirement;
                childInternodeData.m_adjustedTotalProductiveWaterRequirement = glm::pow(
                        childInternodeData.m_adjustedTotalProductiveWaterRequirement, apicalControl);
                totalChildResourceRequirement += childInternodeData.m_adjustedTotalProductiveWaterRequirement;
            }
            for (const auto &i: internode.RefChildHandles()) {
                auto &childInternode = skeleton.RefInternode(i);
                auto &childInternodeData = childInternode.m_data;
                childInternodeData.m_adjustedTotalProductiveWaterRequirement =
                        childInternodeData.m_adjustedTotalProductiveWaterRequirement * 1.0f /
                        totalChildResourceRequirement * internodeData.m_descendentProductiveResourceRequirement;
                float resourceFactor = childInternodeData.m_descendentProductiveResourceRequirement +
                                       childInternodeData.m_productiveResourceRequirement;
                resourceFactor = childInternodeData.m_productiveResourceRequirement / resourceFactor;
                childInternodeData.m_productiveResourceRequirement *= resourceFactor;
                childInternodeData.m_descendentProductiveResourceRequirement *= resourceFactor;
                for (auto &bud: childInternodeData.m_buds) {
                    bud.m_productiveResourceRequirement *= resourceFactor;
                }
            }
        }
        for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
            const bool graphChanged = GrowInternode(*it, parameters, growthNutrients);
            graphChangedDuringGrowth = graphChangedDuringGrowth || graphChanged;
        }
    }
#pragma endregion
#pragma region Postprocess
    if(graphChangedDuringGrowth) skeleton.SortLists();
    structureChanged = graphChangedDuringGrowth;
    {
        skeleton.m_min = glm::vec3(FLT_MAX);
        skeleton.m_max = glm::vec3(FLT_MIN);

        const auto &sortedInternodeList = skeleton.RefSortedInternodeList();
        for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
            auto internodeHandle = *it;
            CalculateSagging(internodeHandle, parameters);
        }
        for (const auto &internodeHandle: sortedInternodeList) {
            auto &internode = skeleton.RefInternode(internodeHandle);
            auto &internodeData = internode.m_data;
            auto &internodeInfo = internode.m_info;
            if (internode.GetParentHandle() == -1) {
                internodeInfo.m_globalPosition = glm::vec3(0.0f);
                internodeInfo.m_localRotation = glm::vec3(0.0f);
                internodeInfo.m_globalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);

                internodeData.m_rootDistance = internodeInfo.m_length;
            } else {
                auto &parentInternode = skeleton.RefInternode(internode.GetParentHandle());
                internodeData.m_rootDistance = parentInternode.m_data.m_rootDistance + internodeInfo.m_length;
                internodeInfo.m_globalRotation =
                        parentInternode.m_info.m_globalRotation * internodeInfo.m_localRotation;
#pragma region Apply Sagging
                auto parentGlobalRotation = skeleton.RefInternode(internode.GetParentHandle()).m_info.m_globalRotation;
                internodeInfo.m_globalRotation = parentGlobalRotation * internodeData.m_desiredLocalRotation;
                auto front = internodeInfo.m_globalRotation * glm::vec3(0, 0, -1);
                auto up = internodeInfo.m_globalRotation * glm::vec3(0, 1, 0);
                float dotP = glm::abs(glm::dot(front, m_gravityDirection));
                ApplyTropism(-m_gravityDirection, internodeData.m_sagging * (1.0f - dotP), front, up);
                internodeInfo.m_globalRotation = glm::quatLookAt(front, up);
                internodeInfo.m_localRotation = glm::inverse(parentGlobalRotation) * internodeInfo.m_globalRotation;
#pragma endregion

                internodeInfo.m_globalPosition =
                        parentInternode.m_info.m_globalPosition + parentInternode.m_info.m_length *
                                                                  (parentInternode.m_info.m_globalRotation *
                                                                   glm::vec3(0, 0, -1));

            }
            skeleton.m_min = glm::min(skeleton.m_min, internodeInfo.m_globalPosition);
            skeleton.m_max = glm::max(skeleton.m_max, internodeInfo.m_globalPosition);
            const auto endPosition = internodeInfo.m_globalPosition + internodeInfo.m_length *
                                                                      (internodeInfo.m_globalRotation *
                                                                       glm::vec3(0, 0, -1));
            skeleton.m_min = glm::min(skeleton.m_min, endPosition);
            skeleton.m_max = glm::max(skeleton.m_max, endPosition);
        }
    }
    structureChanged = true;
    {
        const auto &sortedFlowList = skeleton.RefSortedFlowList();
        for (const auto &flowHandle: sortedFlowList) {
            auto &flow = skeleton.RefFlow(flowHandle);
            auto &flowData = flow.m_data;
            if (flow.GetParentHandle() == -1) {
                flowData.m_order = 0;
            } else {
                auto &parentFlow = skeleton.RefFlow(flow.GetParentHandle());
                if (flow.IsApical()) flowData.m_order = parentFlow.m_data.m_order;
                else flowData.m_order = parentFlow.m_data.m_order + 1;
            }
            for (const auto &internodeHandle: flow.RefInternodes()) {
                skeleton.RefInternode(internodeHandle).m_data.m_order = flowData.m_order;
            }
        }
        skeleton.CalculateFlows();
    }
#pragma endregion

    return structureChanged;
}

void TreeModel::Initialize(const TreeStructuralGrowthParameters &parameters) {
    auto &skeleton = m_treeStructure.Skeleton();
    auto &firstInternode = skeleton.RefInternode(0);
    firstInternode.m_info.m_thickness = parameters.GetEndNodeThickness(firstInternode);
    firstInternode.m_data.m_buds.emplace_back();
    auto &apicalBud = firstInternode.m_data.m_buds.back();
    apicalBud.m_type = BudType::Apical;
    apicalBud.m_status = BudStatus::Dormant;
    apicalBud.m_localRotation = glm::vec3(glm::radians(parameters.GetDesiredApicalAngle(firstInternode)), 0.0f,
                                          parameters.GetDesiredRollAngle(firstInternode));
    m_initialized = true;
}

void TreeModel::Clear() {
    m_treeStructure = {};
    m_initialized = false;
}

bool TreeModel::LowBranchPruning(float maxDistance, InternodeHandle internodeHandle,
                                 const TreeStructuralGrowthParameters &parameters) {
    auto &skeleton = m_treeStructure.Skeleton();
    auto &internode = skeleton.RefInternode(internodeHandle);
    //Pruning here.
    if (maxDistance > 5 && internode.m_data.m_order != 0 &&
        internode.m_data.m_rootDistance / maxDistance < parameters.GetLowBranchPruning(internode)) {
        skeleton.RecycleInternode(internodeHandle);
        return true;
    }
    return false;
}


TreeStructuralGrowthParameters::TreeStructuralGrowthParameters() {
    m_lateralBudCount = 2;
    m_branchingAngleMeanVariance = glm::vec2(30, 3);
    m_rollAngleMeanVariance = glm::vec2(120, 2);
    m_apicalAngleMeanVariance = glm::vec2(0, 4);
    m_gravitropism = -0.1f;
    m_phototropism = 0.05f;
    m_internodeLength = 1.0f;
    m_growthRate = 1.0f;
    m_endNodeThicknessAndControl = glm::vec2(0.01, 0.5);
    m_lateralBudFlushingProbability = 0.3f;
    m_apicalControlBaseDistFactor = {1.1f, 0.95f};
    m_apicalDominanceBaseAgeDist = glm::vec3(0.12, 1, 0.3);
    m_budKillProbabilityApicalLateral = glm::vec2(0.0, 0.03);
    m_lowBranchPruning = 0.2f;
    m_saggingFactorThicknessReductionMax = glm::vec3(6, 3, 0.5);

    m_baseResourceRequirementFactor = glm::vec3(1.0f);
    m_productiveResourceRequirementFactor = glm::vec3(1.0f);
}

int TreeStructuralGrowthParameters::GetLateralBudCount(const Internode<InternodeGrowthData> &internode) const {
    return m_lateralBudCount;
}

float TreeStructuralGrowthParameters::GetDesiredBranchingAngle(const Internode<InternodeGrowthData> &internode) const {
    return glm::gaussRand(
            m_branchingAngleMeanVariance.x,
            m_branchingAngleMeanVariance.y);
}

float TreeStructuralGrowthParameters::GetDesiredRollAngle(const Internode<InternodeGrowthData> &internode) const {
    return glm::gaussRand(
            m_rollAngleMeanVariance.x,
            m_rollAngleMeanVariance.y);
}

float TreeStructuralGrowthParameters::GetDesiredApicalAngle(const Internode<InternodeGrowthData> &internode) const {
    return glm::gaussRand(
            m_apicalAngleMeanVariance.x,
            m_apicalAngleMeanVariance.y);
}

float TreeStructuralGrowthParameters::GetGravitropism(const Internode<InternodeGrowthData> &internode) const {
    return m_gravitropism;
}

float TreeStructuralGrowthParameters::GetPhototropism(const Internode<InternodeGrowthData> &internode) const {
    return m_phototropism;
}

float TreeStructuralGrowthParameters::GetInternodeLength(const Internode<InternodeGrowthData> &internode) const {
    return m_internodeLength;
}

float TreeStructuralGrowthParameters::GetGrowthRate(const Internode<InternodeGrowthData> &internode) const {
    return m_growthRate;
}

float TreeStructuralGrowthParameters::GetEndNodeThickness(const Internode<InternodeGrowthData> &internode) const {
    return m_endNodeThicknessAndControl.x;
}

float TreeStructuralGrowthParameters::GetThicknessControlFactor(const Internode<InternodeGrowthData> &internode) const {
    return m_endNodeThicknessAndControl.y;
}

float TreeStructuralGrowthParameters::GetLateralBudFlushingProbability(
        const Internode<InternodeGrowthData> &internode) const {
    return m_lateralBudFlushingProbability;
}

float TreeStructuralGrowthParameters::GetApicalControl(const Internode<InternodeGrowthData> &internode) const {
    return glm::pow(m_apicalControlBaseDistFactor.x, glm::max(1.0f, 1.0f /
                                                                    internode.m_data.m_rootDistance *
                                                                    m_apicalControlBaseDistFactor.y));
}

float TreeStructuralGrowthParameters::GetApicalDominanceBase(const Internode<InternodeGrowthData> &internode) const {
    return m_apicalDominanceBaseAgeDist.x *
           glm::pow(
                   m_apicalDominanceBaseAgeDist.y,
                   internode.m_data.m_age);
}

float TreeStructuralGrowthParameters::GetApicalDominanceDecrease(
        const Internode<InternodeGrowthData> &internode) const {
    return m_apicalDominanceBaseAgeDist.z;
}

float
TreeStructuralGrowthParameters::GetApicalBudKillProbability(const Internode<InternodeGrowthData> &internode) const {
    return m_budKillProbabilityApicalLateral.x;
}

float
TreeStructuralGrowthParameters::GetLateralBudKillProbability(const Internode<InternodeGrowthData> &internode) const {
    return m_budKillProbabilityApicalLateral.y;
}

float TreeStructuralGrowthParameters::GetLowBranchPruning(const Internode<InternodeGrowthData> &internode) const {
    return m_lowBranchPruning;
}

bool TreeStructuralGrowthParameters::GetPruning(const Internode<InternodeGrowthData> &internode) const {
    return false;
}

float TreeStructuralGrowthParameters::GetSagging(const Internode<InternodeGrowthData> &internode) const {
    return glm::min(
            m_saggingFactorThicknessReductionMax.z,
            m_saggingFactorThicknessReductionMax.x *
            internode.m_data.m_childTotalBiomass /
            glm::pow(
                    internode.m_info.m_thickness /
                    m_endNodeThicknessAndControl.x,
                    m_saggingFactorThicknessReductionMax.y));
}

float TreeStructuralGrowthParameters::GetShootBaseResourceRequirementFactor(
        const Internode<InternodeGrowthData> &internode) const {
    return m_baseResourceRequirementFactor.x;
}

float TreeStructuralGrowthParameters::GetLeafBaseResourceRequirementFactor(
        const Internode<InternodeGrowthData> &internode) const {
    return m_baseResourceRequirementFactor.y;
}

float TreeStructuralGrowthParameters::GetFruitBaseResourceRequirementFactor(
        const Internode<InternodeGrowthData> &internode) const {
    return m_baseResourceRequirementFactor.z;
}

float TreeStructuralGrowthParameters::GetShootProductiveResourceRequirementFactor(
        const Internode<InternodeGrowthData> &internode) const {
    return m_productiveResourceRequirementFactor.x;
}

float TreeStructuralGrowthParameters::GetLeafProductiveResourceRequirementFactor(
        const Internode<InternodeGrowthData> &internode) const {
    return m_productiveResourceRequirementFactor.y;
}

float TreeStructuralGrowthParameters::GetFruitProductiveResourceRequirementFactor(
        const Internode<InternodeGrowthData> &internode) const {
    return m_productiveResourceRequirementFactor.z;
}

void InternodeGrowthData::Clear() {
    m_age = 0;
    m_inhibitor = 0;
    m_desiredLocalRotation = glm::vec3(0.0f);
    m_sagging = 0;

    m_maxDistanceToAnyBranchEnd = 0;
    m_level = 0;
    m_childTotalBiomass = 0;

    m_rootDistance = 0;

    m_productiveResourceRequirement = 0.0f;
    m_descendentProductiveResourceRequirement = 0.0f;
    m_adjustedTotalProductiveWaterRequirement = 0.0f;
    m_lightDirection = glm::vec3(0, 1, 0);
    m_lightIntensity = 1.0f;
    m_buds.clear();
}
