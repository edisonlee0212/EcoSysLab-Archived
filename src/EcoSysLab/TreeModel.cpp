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

void TreeModel::CollectInhibitor(InternodeHandle internodeHandle, const TreeStructuralGrowthParameters& parameters) {
    auto &internode = m_tree->RefInternode(internodeHandle);
    auto &internodeData = internode.m_data;
    if (internode.m_endNode) {
        //If current node is end node
        internodeData.m_inhibitor = 0;
    } else {
        //If current node is not end node
        for (const auto &i: internode.m_children) {
            auto &childInternode = m_tree->RefInternode(i);
            if (childInternode.m_endNode) {
                internodeData.m_inhibitor += parameters.m_apicalDominanceBaseAgeDist.x *
                                             glm::pow(
                                                     parameters.m_apicalDominanceBaseAgeDist.y,
                                                     childInternode.m_data.m_age);
            } else {
                internodeData.m_inhibitor +=
                        childInternode.m_data.m_inhibitor * parameters.m_apicalDominanceBaseAgeDist.z;
            };
        }
    }
}

void TreeModel::GrowInternode(InternodeHandle internodeHandle, const TreeStructuralGrowthParameters& parameters, const GrowthNutrients &growthNutrients) {
    auto &buds = m_tree->RefInternode(internodeHandle).m_data.m_buds;
    for (auto &bud: buds) {
        auto &internode = m_tree->RefInternode(internodeHandle);
        auto &internodeData = internode.m_data;
        switch (bud.m_type) {
            case BudType::Apical: {
                if (bud.m_status == BudStatus::Dormant) {
                    if (parameters.m_budKillProbabilityApicalLateral.x > glm::linearRand(0.0f, 1.0f)) {
                        bud.m_status = BudStatus::Died;
                    } else {
                        float waterReceived = growthNutrients.m_water * internodeData.m_apicalControl;
                        internode.m_length += waterReceived * parameters.m_growthRate;
                        if (internode.m_length > parameters.m_internodeLength) {
                            bud.m_status = BudStatus::Flushed;
                            //Prepare information for new internode.
                            float extraLength =
                                    (internode.m_length - parameters.m_internodeLength) / parameters.m_growthRate;
                            if (extraLength > parameters.m_internodeLength)
                                extraLength = parameters.m_internodeLength;
                            internode.m_length = parameters.m_internodeLength;
                            auto desiredGlobalRotation = internode.m_globalRotation * bud.m_localRotation;
                            auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
                            auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
                            ApplyTropism(m_gravityDirection, parameters.m_gravitropism, desiredGlobalFront,
                                         desiredGlobalUp);
                            ApplyTropism(internodeData.m_lightDirection, parameters.m_phototropism,
                                         desiredGlobalFront, desiredGlobalUp);

                            //Allocate Lateral bud for current internode
                            float turnAngle = glm::radians(360.0f / parameters.m_lateralBudCount);
                            for (int i = 0; i < parameters.m_lateralBudCount; i++) {
                                internodeData.m_buds.emplace_back();
                                auto &lateralBud = internodeData.m_buds.back();
                                lateralBud.m_type = BudType::LateralVegetative;
                                lateralBud.m_status = BudStatus::Dormant;
                                auto branchingAngle = glm::gaussRand(parameters.m_branchingAngleMeanVariance.x,
                                                                     parameters.m_branchingAngleMeanVariance.y);
                                lateralBud.m_localRotation = glm::vec3(glm::radians(branchingAngle), 0.0f,
                                                                       i * turnAngle);
                            }

                            //Create new internode
                            auto newInternodeHandle = m_tree->Extend(internodeHandle, false);
                            auto &oldInternode = m_tree->RefInternode(internodeHandle);
                            auto &newInternode = m_tree->RefInternode(newInternodeHandle);
                            newInternode.m_data.Clear();
                            newInternode.m_length = extraLength;
                            newInternode.m_thickness = parameters.m_endNodeThicknessAndControl.x;
                            newInternode.m_localRotation = newInternode.m_data.m_desiredLocalRotation =
                                    glm::inverse(oldInternode.m_globalRotation) *
                                    glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);

                            //Allocate apical bud for new internode
                            newInternode.m_data.m_buds.emplace_back();
                            auto &apicalBud = newInternode.m_data.m_buds.back();
                            apicalBud.m_type = BudType::Apical;
                            apicalBud.m_status = BudStatus::Dormant;
                            const auto rollAngle = glm::gaussRand(
                                    parameters.m_rollAngleMeanVariance.x,
                                    parameters.m_rollAngleMeanVariance.y);
                            const auto apicalAngle = glm::gaussRand(
                                    parameters.m_apicalAngleMeanVariance.x,
                                    parameters.m_apicalAngleMeanVariance.y);
                            apicalBud.m_localRotation = glm::vec3(glm::radians(apicalAngle), 0.0f, rollAngle);

                        }
                    }
                    //If apical bud is dormant, then there's no lateral bud at this stage. We should quit anyway.
                    return;
                }
            }
                break;
            case BudType::LateralVegetative: {
                if (bud.m_status == BudStatus::Dormant) {
                    if (parameters.m_budKillProbabilityApicalLateral.y > glm::linearRand(0.0f, 1.0f)) {
                        bud.m_status = BudStatus::Died;
                    } else {
                        bool flush = false;
                        float flushProbability = parameters.m_lateralBudFlushingProbability;
                        flushProbability /= (1.0f + internodeData.m_inhibitor);
                        if (flushProbability >= glm::linearRand(0.0f, 1.0f)) {
                            flush = true;
                        }
                        if (flush) {
                            bud.m_status = BudStatus::Flushed;
                            //Prepare information for new internode
                            auto desiredGlobalRotation = internode.m_globalRotation * bud.m_localRotation;
                            auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
                            auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
                            ApplyTropism(m_gravityDirection, parameters.m_gravitropism, desiredGlobalFront,
                                         desiredGlobalUp);
                            ApplyTropism(internodeData.m_lightDirection, parameters.m_phototropism,
                                         desiredGlobalFront, desiredGlobalUp);
                            //Create new internode
                            auto newInternodeHandle = m_tree->Extend(internodeHandle, true);
                            auto &oldInternode = m_tree->RefInternode(internodeHandle);
                            auto &newInternode = m_tree->RefInternode(newInternodeHandle);
                            newInternode.m_data.Clear();
                            newInternode.m_length = 0.0f;
                            newInternode.m_thickness = parameters.m_endNodeThicknessAndControl.x;
                            newInternode.m_localRotation = newInternode.m_data.m_desiredLocalRotation =
                                    glm::inverse(oldInternode.m_globalRotation) *
                                    glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
                            //Allocate apical bud
                            newInternode.m_data.m_buds.emplace_back();
                            auto &apicalBud = newInternode.m_data.m_buds.back();
                            apicalBud.m_type = BudType::Apical;
                            apicalBud.m_status = BudStatus::Dormant;
                            const auto rollAngle = glm::gaussRand(
                                    parameters.m_rollAngleMeanVariance.x,
                                    parameters.m_rollAngleMeanVariance.y);
                            const auto apicalAngle = glm::gaussRand(
                                    parameters.m_apicalAngleMeanVariance.x,
                                    parameters.m_apicalAngleMeanVariance.y);
                            apicalBud.m_localRotation = glm::vec3(glm::radians(apicalAngle), 0.0f, rollAngle);

                        }
                    }
                }
            }
                break;
            case BudType::LateralReproductive: {

            }
                break;
        }
    }
}

void TreeModel::CalculateSagging(InternodeHandle internodeHandle, const TreeStructuralGrowthParameters& parameters) {
    auto &internode = m_tree->RefInternode(internodeHandle);
    auto &internodeData = internode.m_data;
    internodeData.m_childTotalBiomass = 0;
    internodeData.m_decedentsAmount = 0;
    if (!internode.m_endNode) {
        //If current node is not end node
        float maxDistanceToAnyBranchEnd = 0;
        float childThicknessCollection = 0.0f;
        for (const auto &i: internode.m_children) {
            auto &childInternode = m_tree->RefInternode(i);
            internodeData.m_childTotalBiomass +=
                    childInternode.m_data.m_childTotalBiomass + childInternode.m_thickness * childInternode.m_length;
            internodeData.m_decedentsAmount += childInternode.m_data.m_decedentsAmount + 1;
            float childMaxDistanceToAnyBranchEnd =
                    childInternode.m_data.m_maxDistanceToAnyBranchEnd + childInternode.m_length;
            maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);

            childThicknessCollection += glm::pow(childInternode.m_thickness, 1.0f / parameters.m_endNodeThicknessAndControl.y);
        }
        internodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
        internodeData.m_sagging =
                glm::min(
                        parameters.m_saggingFactorThicknessReductionMax.z,
                        parameters.m_saggingFactorThicknessReductionMax.x *
                        internodeData.m_childTotalBiomass /
                        glm::pow(
                                internode.m_thickness /
                                parameters.m_endNodeThicknessAndControl.x,
                                parameters.m_saggingFactorThicknessReductionMax.y));
        internode.m_thickness = glm::pow(childThicknessCollection, parameters.m_endNodeThicknessAndControl.y);
    }
}

void TreeModel::Grow(const GrowthNutrients &growthNutrients, const TreeStructuralGrowthParameters& parameters) {
    if (!m_initialized) {
        return;
    }

    if (!m_tree) {
        return;
    }
#pragma region Preprocess
    m_tree->SortLists();
    {
        const auto &sortedInternodeList = m_tree->GetSortedInternodeList();
        const auto maxDistance = m_tree->RefInternode(sortedInternodeList.front()).m_data.m_maxDistanceToAnyBranchEnd;
        for (const auto &internodeHandle: sortedInternodeList) {
            auto &internode = m_tree->RefInternode(internodeHandle);
            //Pruning here.
            if (internode.m_recycled) continue;
            auto &branch = m_tree->RefBranch(internode.m_branchHandle);
            if(maxDistance > 5 && branch.m_data.m_order != 0 && (maxDistance - internode.m_data.m_maxDistanceToAnyBranchEnd) / maxDistance < parameters.m_lowBranchPruning){
                m_tree->PruneInternode(internodeHandle);
                continue;
            }
        }
    }
#pragma endregion
#pragma region Grow
    m_tree->SortLists();
    {
        const auto &sortedInternodeList = m_tree->GetSortedInternodeList();
        for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
            auto internodeHandle = *it;
            CollectInhibitor(internodeHandle, parameters);
            GrowInternode(internodeHandle, parameters, growthNutrients);
            CalculateSagging(internodeHandle, parameters);
        }

    }
#pragma endregion
#pragma region Postprocess
    m_tree->SortLists();
    {
        const auto &sortedInternodeList = m_tree->GetSortedInternodeList();

        for (const auto &internodeHandle: sortedInternodeList) {
            auto &internode = m_tree->RefInternode(internodeHandle);
            auto &internodeData = internode.m_data;
            if (internode.m_parent == -1) {
                internode.m_globalPosition = glm::vec3(0.0f);
                internode.m_localRotation = glm::vec3(0.0f);
                internode.m_globalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);

                internodeData.m_rootDistance = internode.m_length;
                internodeData.m_apicalControl = 1.0f;
            } else {
                auto &parentInternode = m_tree->RefInternode(internode.m_parent);
                internodeData.m_rootDistance = parentInternode.m_data.m_rootDistance + internode.m_length;

                internode.m_globalRotation = parentInternode.m_globalRotation * internode.m_localRotation;
                internode.m_globalPosition = parentInternode.m_globalPosition + parentInternode.m_length *
                                                                                (parentInternode.m_globalRotation *
                                                                                 glm::vec3(0, 0, -1));
#pragma region Apply Sagging
                auto parentGlobalRotation = m_tree->RefInternode(internode.m_parent).m_globalRotation;
                internode.m_globalRotation = parentGlobalRotation * internodeData.m_desiredLocalRotation;
                auto front = internode.m_globalRotation * glm::vec3(0, 0, -1);
                auto up = internode.m_globalRotation * glm::vec3(0, 1, 0);
                float dotP = glm::abs(glm::dot(front, m_gravityDirection));
                ApplyTropism(-m_gravityDirection, internodeData.m_sagging * (1.0f - dotP), front, up);
                internode.m_globalRotation = glm::quatLookAt(front, up);
                internode.m_localRotation = glm::inverse(parentGlobalRotation) * internode.m_globalRotation;
#pragma endregion
            }

            float apicalControl = glm::pow(parameters.m_apicalControlBaseDistFactor.x, glm::max(1.0f, 1.0f /
                                                                                                        internodeData.m_rootDistance *
                                                                                                        parameters.m_apicalControlBaseDistFactor.y));
            float totalApicalControl = 0.0f;
            for (const auto &i: internode.m_children) {
                auto &childInternode = m_tree->RefInternode(i);
                auto &childInternodeData = childInternode.m_data;
                childInternodeData.m_apicalControl = glm::pow(childInternodeData.m_decedentsAmount + 1,
                                                              apicalControl);
                totalApicalControl += childInternodeData.m_apicalControl;
            }
            for (const auto &i: internode.m_children) {
                auto &childInternode = m_tree->RefInternode(i);
                auto &childInternodeData = childInternode.m_data;
                childInternodeData.m_apicalControl =
                        internodeData.m_apicalControl * childInternodeData.m_apicalControl / totalApicalControl;
            }
        }
    }

    {
        const auto &sortedBranchList = m_tree->GetSortedBranchList();
        for (const auto &branchHandle: sortedBranchList) {
            auto &branch = m_tree->RefBranch(branchHandle);
            auto &branchData = branch.m_data;
            if (branch.m_parent == -1) {
                branchData.m_order = 0;
            } else {
                auto &parentBranch = m_tree->RefBranch(branch.m_parent);
                branchData.m_order = parentBranch.m_data.m_order + 1;
            }
        }
        m_tree->CalculateBranches();
    }
#pragma endregion
}

void TreeModel::Initialize(const TreeStructuralGrowthParameters& parameters) {
    m_tree = std::make_shared<TreeSkeleton<BranchData, InternodeData>>();
    auto &firstInternode = m_tree->RefInternode(0);
    firstInternode.m_data.m_buds.emplace_back();
    auto &apicalBud = firstInternode.m_data.m_buds.back();
    apicalBud.m_type = BudType::Apical;
    apicalBud.m_status = BudStatus::Dormant;
    const auto rollAngle = glm::gaussRand(
            parameters.m_rollAngleMeanVariance.x,
            parameters.m_rollAngleMeanVariance.y);
    const auto apicalAngle = glm::gaussRand(
            parameters.m_apicalAngleMeanVariance.x,
            parameters.m_apicalAngleMeanVariance.y);
    apicalBud.m_localRotation = glm::vec3(glm::radians(apicalAngle), 0.0f, rollAngle);
    m_initialized = true;
}

void TreeModel::Clear() {
    m_tree.reset();
    m_initialized = false;
}

bool TreeModel::IsInitialized() const {
    return m_initialized;
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
    m_apicalControlBaseDistFactor = {2.0f, 0.95f};
    m_apicalDominanceBaseAgeDist = glm::vec3(0.12, 1, 0.3);
    m_lateralBudFlushingLightingFactor = 0.0f;
    m_budKillProbabilityApicalLateral = glm::vec2(0.0, 0.03);
    m_randomPruningOrderProtection = 1;
    m_randomPruningBaseAgeMax = glm::vec3(-0.1, 0.007, 0.5);
    m_lowBranchPruning = 0.2f;
    m_saggingFactorThicknessReductionMax = glm::vec3(6, 3, 0.5);
}

void InternodeData::Clear() {
    m_age = 0;
    m_inhibitor = 0;
    m_desiredLocalRotation = glm::vec3(0.0f);
    m_sagging = 0;

    m_maxDistanceToAnyBranchEnd = 0;
    m_level = 0;
    m_childTotalBiomass = 0;

    m_rootDistance = 0;

    m_apicalControl = 0.0f;
    m_decedentsAmount = 0;
    m_lightDirection = glm::vec3(0, 1, 0);
    m_lightIntensity = 1.0f;
    m_buds.clear();
}
