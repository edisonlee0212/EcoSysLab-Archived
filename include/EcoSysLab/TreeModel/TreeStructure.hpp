#pragma once

#include "ecosyslab_export.h"

#define InternodeHandle int
#define FlowHandle int
using namespace UniEngine;
namespace EcoSysLab {
    struct InternodeInfo {
        glm::vec3 m_globalPosition = glm::vec3(0.0f);
        glm::quat m_globalRotation = glm::vec3(0.0f);

        float m_length = 0.0f;
        float m_thickness = 0.1f;
        glm::quat m_localRotation = glm::vec3(0.0f);
    };

    template<typename InternodeData>
    class Internode {
    public:
        InternodeData m_data;
        InternodeInfo m_info;
        bool m_endNode = true;
        bool m_recycled = false;
        InternodeHandle m_handle = -1;
        FlowHandle m_branchHandle = -1;

        InternodeHandle m_parent = -1;
        std::vector<InternodeHandle> m_children;

        std::vector<std::pair<int, InternodeHandle>> m_prunedChildren;

        explicit Internode(InternodeHandle handle);

    };

    struct FlowInfo {
        glm::vec3 m_globalStartPosition = glm::vec3(0.0f);
        glm::quat m_globalStartRotation = glm::vec3(0.0f);
        float m_startThickness = 0.0f;

        glm::vec3 m_globalEndPosition = glm::vec3(0.0f);
        glm::quat m_globalEndRotation = glm::vec3(0.0f);
        float m_endThickness = 0.0f;
    };

    template<typename FlowData>
    class Flow {
    public:
        FlowData m_data;
        FlowInfo m_info;

        bool m_recycled = false;
        FlowHandle m_handle = -1;

        std::vector<InternodeHandle> m_internodes;

        FlowHandle m_parent = -1;

        std::vector<FlowHandle> m_children;

        explicit Flow(FlowHandle handle);
    };


    template<typename FlowData, typename InternodeData>
    class TreeSkeleton {
        std::vector<Flow<FlowData>> m_flows;
        std::vector<Internode<InternodeData>> m_internodes;
        std::queue<InternodeHandle> m_internodePool;
        std::queue<FlowHandle> m_flowPool;

        int m_newVersion = 0;
        int m_version = -1;
        std::vector<InternodeHandle> m_sortedInternodeList;
        std::vector<FlowHandle> m_sortedFlowList;

        InternodeHandle AllocateInternode();

        void RecycleInternodeSingle(InternodeHandle handle);

        void RecycleFlowSingle(FlowHandle handle);

        FlowHandle AllocateFlow();

        void SetParentFlow(FlowHandle targetHandle, FlowHandle parentHandle);

        void DetachChildFlow(FlowHandle targetHandle, FlowHandle childHandle);

        void SetParentInternode(InternodeHandle targetHandle, InternodeHandle parentHandle);

        void DetachChildInternode(InternodeHandle targetHandle, InternodeHandle childHandle);


    public:
        void RecycleInternode(InternodeHandle handle);

        void RecycleFlow(FlowHandle handle);

        InternodeHandle Extend(InternodeHandle targetHandle, bool createNewBranch);

        [[nodiscard]] const std::vector<InternodeHandle> &RefSortedInternodeList() const;

        [[nodiscard]] const std::vector<FlowHandle> &RefSortedFlowList() const;

        void SortLists();

        TreeSkeleton();

        [[nodiscard]] int GetVersion() const;

        void CalculateBranches();

        Internode<InternodeData> &RefInternode(InternodeHandle handle);

        Flow<FlowData> &RefFlow(FlowHandle handle);

        const Internode<InternodeData> &PeekInternode(InternodeHandle handle) const;

        const Flow<FlowData> &PeekFlow(FlowHandle handle) const;

        glm::vec3 m_min = glm::vec3(0.0f);
        glm::vec3 m_max = glm::vec3(0.0f);
    };

    template<typename FlowData, typename InternodeData>
    class TreeStructure {
        TreeSkeleton<FlowData, InternodeData> m_skeleton;
        std::vector<TreeSkeleton<FlowData, InternodeData>> m_history;
    public:
        [[nodiscard]] TreeSkeleton<FlowData, InternodeData> &Skeleton();

        [[nodiscard]] const TreeSkeleton<FlowData, InternodeData> &Peek(int iteration) const;

        void Step();

        [[nodiscard]] int CurrentIteration() const;

        void Reverse(int iteration);
    };

    template<typename InternodeData>
    class ITreeGrowthParameters{
    public:
        virtual int GetLateralBudCount(const Internode<InternodeData>& internode) const = 0;
        virtual float GetDesiredBranchingAngle(const Internode<InternodeData>& internode) const = 0;
        virtual float GetDesiredRollAngle(const Internode<InternodeData>& internode) const = 0;
        virtual float GetDesiredApicalAngle(const Internode<InternodeData>& internode) const = 0;
        virtual float GetGravitropism(const Internode<InternodeData>& internode) const = 0;
        virtual float GetPhototropism(const Internode<InternodeData>& internode) const = 0;
        virtual float GetInternodeLength(const Internode<InternodeData>& internode) const = 0;
        virtual float GetGrowthRate(const Internode<InternodeData>& internode) const = 0;
        virtual float GetEndNodeThickness(const Internode<InternodeData>& internode) const = 0;
        virtual float GetThicknessControlFactor(const Internode<InternodeData>& internode) const = 0;
        virtual float GetLateralBudFlushingProbability(const Internode<InternodeData>& internode) const = 0;
        virtual float GetApicalControlBase(const Internode<InternodeData>& internode) const = 0;
        virtual float GetApicalDominanceBase(const Internode<InternodeData>& internode) const = 0;
        virtual float GetApicalDominanceDecrease(const Internode<InternodeData>& internode) const = 0;
        virtual float GetApicalBudKillProbability(const Internode<InternodeData>& internode) const = 0;
        virtual float GetLateralBudKillProbability(const Internode<InternodeData>& internode) const = 0;
        virtual bool GetPruning(const Internode<InternodeData>& internode) const = 0;
        virtual float GetLowBranchPruning(const Internode<InternodeData>& internode) const = 0;
        virtual float GetSagging(const Internode<InternodeData>& internode) const = 0;
    };

#pragma region
    template<typename FlowData, typename InternodeData>
    void TreeStructure<FlowData, InternodeData>::Step() {
        m_history.push_back(m_skeleton);
    }

    template<typename FlowData, typename InternodeData>
    void TreeStructure<FlowData, InternodeData>::Reverse(int iteration) {
        assert(iteration >= 0 && iteration < m_history.size());
        m_skeleton = m_history[iteration];
        m_history.erase((m_history.begin() + iteration), m_history.end());
    }

    template<typename FlowData, typename InternodeData>
    TreeSkeleton<FlowData, InternodeData> &TreeStructure<FlowData, InternodeData>::Skeleton() {
        return m_skeleton;
    }

    template<typename FlowData, typename InternodeData>
    const TreeSkeleton<FlowData, InternodeData> &TreeStructure<FlowData, InternodeData>::Peek(int iteration) const {
        assert(iteration <= m_history.size());
        if (iteration < 0) iteration = 0;
        if (iteration == m_history.size()) return m_skeleton;
        return m_history[iteration];
    }

    template<typename FlowData, typename InternodeData>
    int TreeStructure<FlowData, InternodeData>::CurrentIteration() const {
        return m_history.size();
    }

#pragma region TreeSkeleton
#pragma region Helper

    template<typename FlowData, typename InternodeData>
    Flow<FlowData> &TreeSkeleton<FlowData, InternodeData>::RefFlow(FlowHandle handle) {
        assert(handle >= 0 && handle < m_flows.size());
        return m_flows[handle];
    }

    template<typename FlowData, typename InternodeData>
    const Flow<FlowData> &TreeSkeleton<FlowData, InternodeData>::PeekFlow(FlowHandle handle) const {
        assert(handle >= 0 && handle < m_flows.size());
        return m_flows[handle];
    }

    template<typename FlowData, typename InternodeData>
    Internode<InternodeData> &TreeSkeleton<FlowData, InternodeData>::RefInternode(InternodeHandle handle) {
        assert(handle >= 0 && handle < m_internodes.size());
        return m_internodes[handle];
    }

    template<typename FlowData, typename InternodeData>
    const Internode<InternodeData> &
    TreeSkeleton<FlowData, InternodeData>::PeekInternode(InternodeHandle handle) const {
        assert(handle >= 0 && handle < m_internodes.size());
        return m_internodes[handle];
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::SortLists() {
        if (m_version == m_newVersion) return;
        m_version = m_newVersion;
        m_sortedFlowList.clear();
        std::queue<FlowHandle> branchWaitList;
        branchWaitList.push(0);
        while (!branchWaitList.empty()) {
            m_sortedFlowList.emplace_back(branchWaitList.front());
            branchWaitList.pop();
            for (const auto &i: m_flows[m_sortedFlowList.back()].m_children) {
                branchWaitList.push(i);
            }
        }

        m_sortedInternodeList.clear();
        std::queue<InternodeHandle> internodeWaitList;
        internodeWaitList.push(0);
        while (!internodeWaitList.empty()) {
            m_sortedInternodeList.emplace_back(internodeWaitList.front());
            internodeWaitList.pop();
            for (const auto &i: m_internodes[m_sortedInternodeList.back()].m_children) {
                internodeWaitList.push(i);
            }
        }
    }

    template<typename FlowData, typename InternodeData>
    const std::vector<FlowHandle> &TreeSkeleton<FlowData, InternodeData>::RefSortedFlowList() const {
        return m_sortedFlowList;
    }

    template<typename FlowData, typename InternodeData>
    const std::vector<InternodeHandle> &TreeSkeleton<FlowData, InternodeData>::RefSortedInternodeList() const {
        return m_sortedInternodeList;
    }

    template<typename FlowData, typename InternodeData>
    InternodeHandle
    TreeSkeleton<FlowData, InternodeData>::Extend(InternodeHandle targetHandle, bool createNewBranch) {
        assert(targetHandle < m_internodes.size());
        auto &targetInternode = m_internodes[targetHandle];
        assert(!targetInternode.m_recycled);
        assert(targetInternode.m_branchHandle < m_flows.size());
        auto &branch = m_flows[targetInternode.m_branchHandle];
        assert(!branch.m_recycled);
        auto newInternodeHandle = AllocateInternode();
        SetParentInternode(newInternodeHandle, targetHandle);
        auto &originalInternode = m_internodes[targetHandle];
        auto &newInternode = m_internodes[newInternodeHandle];

        if (createNewBranch) {
            auto newBranchHandle = AllocateFlow();
            auto &newBranch = m_flows[newBranchHandle];
            SetParentFlow(newBranchHandle, originalInternode.m_branchHandle);
            newInternode.m_branchHandle = newBranchHandle;
            newBranch.m_internodes.emplace_back(newInternodeHandle);
        } else {
            originalInternode.m_endNode = false;
            branch.m_internodes.emplace_back(newInternodeHandle);
            newInternode.m_branchHandle = originalInternode.m_branchHandle;


        }
        m_newVersion++;
        return newInternodeHandle;
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::RecycleFlow(FlowHandle handle) {
        assert(handle != 0);
        assert(!m_flows[handle].m_recycled);
        auto &branch = m_flows[handle];
        //Remove children
        auto children = branch.m_children;
        for (const auto &child: children) {
            RecycleFlow(child);
        }
        //Detach from parent
        if (branch.m_parent != -1) DetachChildFlow(branch.m_parent, handle);
        //Remove internodes
        if (!branch.m_internodes.empty()) {
            //Detach first internode from parent.
            auto internodes = branch.m_internodes;
            for (const auto &i: internodes) {
                RecycleInternodeSingle(i);
            }
        }
        RecycleFlowSingle(handle);
        m_newVersion++;
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::RecycleInternode(InternodeHandle handle) {
        assert(handle != 0);
        assert(!m_internodes[handle].m_recycled);
        auto &internode = m_internodes[handle];
        auto branchHandle = internode.m_branchHandle;
        auto &branch = m_flows[branchHandle];
        if (handle == branch.m_internodes[0]) {
            RecycleFlow(internode.m_branchHandle);
            return;
        }
        //Collect list of subsequent internodes
        std::vector<InternodeHandle> subsequentInternodes;
        while (branch.m_internodes.back() != handle) {
            subsequentInternodes.emplace_back(branch.m_internodes.back());
            branch.m_internodes.pop_back();
        }
        subsequentInternodes.emplace_back(branch.m_internodes.back());
        branch.m_internodes.pop_back();
        assert(!branch.m_internodes.empty());
        //Detach from parent
        if (internode.m_parent != -1) DetachChildInternode(internode.m_parent, handle);
        //From end node remove one by one.
        InternodeHandle prev = -1;
        for (const auto &i: subsequentInternodes) {
            auto children = m_internodes[i].m_children;
            for (const auto &childInternodeHandle: children) {
                if (childInternodeHandle == prev) continue;
                auto &child = m_internodes[childInternodeHandle];
                assert(!child.m_recycled);
                auto childBranchHandle = child.m_branchHandle;
                if (childBranchHandle != branchHandle) {
                    RecycleFlow(childBranchHandle);
                }
            }
            prev = i;
            RecycleInternodeSingle(i);

        }
        m_newVersion++;
    }

#pragma endregion
#pragma region Internal

    template<typename InternodeData>
    Internode<InternodeData>::Internode(int handle) {
        m_handle = handle;
        m_recycled = false;
        m_endNode = true;
        m_data = {};
        m_info = {};
    }

    template<typename FlowData>
    Flow<FlowData>::Flow(int handle) {
        m_handle = handle;
        m_recycled = false;
        m_data = {};
        m_info = {};
    }

    template<typename FlowData, typename InternodeData>
    TreeSkeleton<FlowData, InternodeData>::TreeSkeleton() {
        AllocateFlow();
        AllocateInternode();
        auto &rootBranch = m_flows[0];
        auto &rootInternode = m_internodes[0];
        rootInternode.m_branchHandle = 0;
        rootBranch.m_internodes.emplace_back(0);
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::DetachChildInternode(InternodeHandle targetHandle,
                                                                       InternodeHandle childHandle) {
        assert(targetHandle >= 0 && childHandle >= 0 && targetHandle < m_internodes.size() &&
               childHandle < m_internodes.size());
        auto &targetInternode = m_internodes[targetHandle];
        auto &childInternode = m_internodes[childHandle];
        assert(!targetInternode.m_recycled);
        assert(!childInternode.m_recycled);
        auto &children = targetInternode.m_children;
        for (int i = 0; i < children.size(); i++) {
            if (children[i] == childHandle) {
                children[i] = children.back();
                children.pop_back();
                childInternode.m_parent = -1;
                return;
            }
        }
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::SetParentInternode(InternodeHandle targetHandle,
                                                                     InternodeHandle parentHandle) {
        assert(targetHandle >= 0 && parentHandle >= 0 && targetHandle < m_internodes.size() &&
               parentHandle < m_internodes.size());
        auto &targetInternode = m_internodes[targetHandle];
        auto &parentInternode = m_internodes[parentHandle];
        assert(!targetInternode.m_recycled);
        assert(!parentInternode.m_recycled);
        targetInternode.m_parent = parentHandle;
        parentInternode.m_children.emplace_back(targetHandle);
    }

    template<typename FlowData, typename InternodeData>
    void
    TreeSkeleton<FlowData, InternodeData>::DetachChildFlow(FlowHandle targetHandle, FlowHandle childHandle) {
        assert(targetHandle >= 0 && childHandle >= 0 && targetHandle < m_flows.size() &&
               childHandle < m_flows.size());
        auto &targetBranch = m_flows[targetHandle];
        auto &childBranch = m_flows[childHandle];
        assert(!targetBranch.m_recycled);
        assert(!childBranch.m_recycled);

        if (!childBranch.m_internodes.empty()) {
            auto firstInternodeHandle = childBranch.m_internodes[0];
            auto &firstInternode = m_internodes[firstInternodeHandle];
            if (firstInternode.m_parent != -1)
                DetachChildInternode(firstInternode.m_parent, firstInternodeHandle);
        }

        auto &children = targetBranch.m_children;
        for (int i = 0; i < children.size(); i++) {
            if (children[i] == childHandle) {
                children[i] = children.back();
                children.pop_back();
                childBranch.m_parent = -1;
                return;
            }
        }
    }

    template<typename FlowData, typename InternodeData>
    void
    TreeSkeleton<FlowData, InternodeData>::SetParentFlow(FlowHandle targetHandle, FlowHandle parentHandle) {
        assert(targetHandle >= 0 && parentHandle >= 0 && targetHandle < m_flows.size() &&
               parentHandle < m_flows.size());
        auto &targetBranch = m_flows[targetHandle];
        auto &parentBranch = m_flows[parentHandle];
        assert(!targetBranch.m_recycled);
        assert(!parentBranch.m_recycled);
        targetBranch.m_parent = parentHandle;
        parentBranch.m_children.emplace_back(targetHandle);
    }

    template<typename FlowData, typename InternodeData>
    FlowHandle TreeSkeleton<FlowData, InternodeData>::AllocateFlow() {
        if (m_flowPool.empty()) {
            auto newBranch = m_flows.emplace_back(m_flows.size());
            return newBranch.m_handle;
        }
        auto handle = m_flowPool.front();
        m_flowPool.pop();
        auto &branch = m_flows[handle];
        branch.m_recycled = false;
        return handle;
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::RecycleFlowSingle(FlowHandle handle) {
        assert(!m_flows[handle].m_recycled);
        auto &branch = m_flows[handle];
        branch.m_parent = -1;
        branch.m_children.clear();
        branch.m_internodes.clear();

        branch.m_data = {};
        branch.m_info = {};

        branch.m_recycled = true;
        m_flowPool.emplace(handle);
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::RecycleInternodeSingle(InternodeHandle handle) {
        assert(!m_internodes[handle].m_recycled);
        auto &internode = m_internodes[handle];
        internode.m_parent = -1;
        internode.m_branchHandle = -1;
        internode.m_endNode = true;
        internode.m_children.clear();

        internode.m_data = {};
        internode.m_info = {};

        internode.m_recycled = true;
        m_internodePool.emplace(handle);
    }

    template<typename FlowData, typename InternodeData>
    InternodeHandle TreeSkeleton<FlowData, InternodeData>::AllocateInternode() {
        if (m_internodePool.empty()) {
            auto newInternode = m_internodes.emplace_back(m_internodes.size());
            return newInternode.m_handle;
        }
        auto handle = m_internodePool.front();
        m_internodePool.pop();
        auto &internode = m_internodes[handle];
        internode.m_recycled = false;
        return handle;
    }

    template<typename FlowData, typename InternodeData>
    int TreeSkeleton<FlowData, InternodeData>::GetVersion() const {
        return m_version;
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::CalculateBranches() {
        const auto &sortedBranchList = RefSortedFlowList();
        for (const auto &branchHandle: sortedBranchList) {
            auto &branch = m_flows[branchHandle];
            auto &firstInternode = m_internodes[branch.m_internodes.front()];
            auto &lastInternode = m_internodes[branch.m_internodes.back()];
            branch.m_info.m_startThickness = firstInternode.m_info.m_thickness;
            branch.m_info.m_globalStartPosition = firstInternode.m_info.m_globalPosition;
            branch.m_info.m_globalStartRotation = firstInternode.m_info.m_localRotation;

            branch.m_info.m_endThickness = lastInternode.m_info.m_thickness;
            branch.m_info.m_globalEndPosition = lastInternode.m_info.m_globalPosition +
                                                lastInternode.m_info.m_length *
                                                (lastInternode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
            branch.m_info.m_globalEndRotation = lastInternode.m_info.m_globalRotation;
        }
    }


#pragma endregion
#pragma endregion
}