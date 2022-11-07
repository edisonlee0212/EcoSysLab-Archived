#pragma once

#include "ecosyslab_export.h"

#define InternodeHandle int
#define FlowHandle int
using namespace UniEngine;
namespace EcoSysLab {
#pragma region Structural Info
    struct InternodeInfo {
        glm::vec3 m_globalPosition = glm::vec3(0.0f);
        glm::quat m_globalRotation = glm::vec3(0.0f);

        float m_length = 0.0f;
        float m_thickness = 0.1f;
        glm::quat m_localRotation = glm::vec3(0.0f);

        std::vector<glm::mat4> m_leaves;
    };

    struct FlowInfo {
        glm::vec3 m_globalStartPosition = glm::vec3(0.0f);
        glm::quat m_globalStartRotation = glm::vec3(0.0f);
        float m_startThickness = 0.0f;

        glm::vec3 m_globalEndPosition = glm::vec3(0.0f);
        glm::quat m_globalEndRotation = glm::vec3(0.0f);
        float m_endThickness = 0.0f;
    };
#pragma endregion

    template<typename InternodeData>
    class Internode {
#pragma region Private

        template<typename FD>
        friend
        class Flow;

        template<typename FD, typename ID>
        friend
        class TreeSkeleton;

        bool m_endNode = true;
        bool m_recycled = false;
        InternodeHandle m_handle = -1;
        FlowHandle m_flowHandle = -1;
        InternodeHandle m_parentHandle = -1;
        std::vector<InternodeHandle> m_childHandles;
#pragma endregion
    public:
        InternodeData m_data;
        /**
         * The structural information of current internode.
         */
        InternodeInfo m_info;

        /**
         * Whether this internode is the end node.
         * @return True if this is end node, false else wise.
         */
        [[nodiscard]] bool IsEndNode() const;

        /**
         * Whether this internode is recycled (removed).
         * @return True if this internode is recycled (removed), false else wise.
         */
        [[nodiscard]] bool IsRecycled() const;

        /**
         * Get the handle of self.
         * @return InternodeHandle of current internode.
         */
        [[nodiscard]] InternodeHandle GetHandle() const;

        /**
         * Get the handle of parent.
         * @return InternodeHandle of parent internode.
         */
        [[nodiscard]] InternodeHandle GetParentHandle() const;

        /**
         * Get the handle to belonged flow.
         * @return FlowHandle of belonged flow.
         */
        [[nodiscard]] FlowHandle GetFlowHandle() const;

        /**
         * Access the children by their handles.
         * @return The list of handles.
         */
        [[nodiscard]] const std::vector<InternodeHandle> &RefChildHandles() const;

        explicit Internode(InternodeHandle handle);
    };

    template<typename FlowData>
    class Flow {
#pragma region Private

        template<typename FD, typename ID>
        friend
        class TreeSkeleton;

        bool m_recycled = false;
        FlowHandle m_handle = -1;
        std::vector<InternodeHandle> m_internodes;
        FlowHandle m_parentHandle = -1;
        std::vector<FlowHandle> m_childHandles;
        bool m_apical = false;
#pragma endregion
    public:
        FlowData m_data;
        FlowInfo m_info;

        /**
         * Whether this flow is recycled (removed).
         * @return True if this flow is recycled (removed), false else wise.
         */
        [[nodiscard]] bool IsRecycled() const;

        /**
         * Whether this flow is extended from an apical bud. The apical flow will have the same order as parent flow.
         * @return True if this flow is from apical bud.
         */
        [[nodiscard]] bool IsApical() const;

        /**
         * Get the handle of self.
         * @return FlowHandle of current flow.
         */
        [[nodiscard]] FlowHandle GetHandle() const;

        /**
         * Get the handle of parent.
         * @return FlowHandle of parent flow.
         */
        [[nodiscard]] FlowHandle GetParentHandle() const;

        /**
         * Access the children by their handles.
         * @return The list of handles.
         */
        [[nodiscard]] const std::vector<FlowHandle> &RefChildHandles() const;

        /**
         * Access the internodes that belongs to this flow.
         * @return The list of handles.
         */
        [[nodiscard]] const std::vector<InternodeHandle> &RefInternodes() const;

        explicit Flow(FlowHandle handle);
    };

    template<typename FlowData, typename InternodeData>
    class TreeSkeleton {
#pragma region Private
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

#pragma endregion
    public:
        /**
         * Recycle (Remove) an internode, the descendents of this internode will also be recycled. The relevant flow will also be removed/restructured.
         * @param handle The handle of the internode to be removed. Must be valid (non-zero and the internode should not be recycled prior to this operation).
         */
        void RecycleInternode(InternodeHandle handle);
        /**
         * Recycle (Remove) an flow, the descendents of this flow will also be recycled. The relevant internode will also be removed/restructured.
         * @param handle The handle of the flow to be removed. Must be valid (non-zero and the flow should not be recycled prior to this operation).
         */
        void RecycleFlow(FlowHandle handle);

        /**
         * Branch/prolong internode during growth process. The flow structure will also be updated.
         * @param targetHandle The handle of the internode to branch/prolong
         * @param branching True if branching, false if prolong. During branching, 2 new flows will be generated.
         * @return The handle of new internode.
         */
        InternodeHandle Extend(InternodeHandle targetHandle, bool branching);

        /**
         *
         * @return
         */
        [[nodiscard]] const std::vector<InternodeHandle> &RefSortedInternodeList() const;

        /**
         *
         * @return
         */
        [[nodiscard]] const std::vector<FlowHandle> &RefSortedFlowList() const;

        /**
         *
         */
        void SortLists();

        TreeSkeleton();

        /**
         *
         * @return
         */
        [[nodiscard]] int GetVersion() const;

        /**
         *
         */
        void CalculateBranches();

        /**
         *
         * @param handle
         * @return
         */
        Internode<InternodeData> &RefInternode(InternodeHandle handle);

        /**
         *
         * @param handle
         * @return
         */
        Flow<FlowData> &RefFlow(FlowHandle handle);

        /**
         *
         * @param handle
         * @return
         */
        const Internode<InternodeData> &PeekInternode(InternodeHandle handle) const;

        /**
         *
         * @param handle
         * @return
         */
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
    class ITreeGrowthParameters {
    public:
        virtual int GetLateralBudCount(const Internode<InternodeData> &internode) const = 0;

        virtual float GetDesiredBranchingAngle(const Internode<InternodeData> &internode) const = 0;

        virtual float GetDesiredRollAngle(const Internode<InternodeData> &internode) const = 0;

        virtual float GetDesiredApicalAngle(const Internode<InternodeData> &internode) const = 0;

        virtual float GetGravitropism(const Internode<InternodeData> &internode) const = 0;

        virtual float GetPhototropism(const Internode<InternodeData> &internode) const = 0;

        virtual float GetInternodeLength(const Internode<InternodeData> &internode) const = 0;

        virtual float GetGrowthRate(const Internode<InternodeData> &internode) const = 0;

        virtual float GetEndNodeThickness(const Internode<InternodeData> &internode) const = 0;

        virtual float GetThicknessControlFactor(const Internode<InternodeData> &internode) const = 0;

        virtual float GetLateralBudFlushingProbability(const Internode<InternodeData> &internode) const = 0;

        virtual float GetApicalControlBase(const Internode<InternodeData> &internode) const = 0;

        virtual float GetApicalDominanceBase(const Internode<InternodeData> &internode) const = 0;

        virtual float GetApicalDominanceDecrease(const Internode<InternodeData> &internode) const = 0;

        virtual float GetApicalBudKillProbability(const Internode<InternodeData> &internode) const = 0;

        virtual float GetLateralBudKillProbability(const Internode<InternodeData> &internode) const = 0;

        virtual bool GetPruning(const Internode<InternodeData> &internode) const = 0;

        virtual float GetLowBranchPruning(const Internode<InternodeData> &internode) const = 0;

        virtual float GetSagging(const Internode<InternodeData> &internode) const = 0;
    };

#pragma region TreeStructure

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

#pragma endregion
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
        std::queue<FlowHandle> flowWaitList;
        flowWaitList.push(0);
        while (!flowWaitList.empty()) {
            m_sortedFlowList.emplace_back(flowWaitList.front());
            flowWaitList.pop();
            for (const auto &i: m_flows[m_sortedFlowList.back()].m_childHandles) {
                flowWaitList.push(i);
            }
        }

        m_sortedInternodeList.clear();
        std::queue<InternodeHandle> internodeWaitList;
        internodeWaitList.push(0);
        while (!internodeWaitList.empty()) {
            m_sortedInternodeList.emplace_back(internodeWaitList.front());
            internodeWaitList.pop();
            for (const auto &i: m_internodes[m_sortedInternodeList.back()].m_childHandles) {
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
    TreeSkeleton<FlowData, InternodeData>::Extend(InternodeHandle targetHandle, bool branching) {
        assert(targetHandle < m_internodes.size());
        auto &targetInternode = m_internodes[targetHandle];
        assert(!targetInternode.m_recycled);
        assert(targetInternode.m_flowHandle < m_flows.size());
        auto &flow = m_flows[targetInternode.m_flowHandle];
        assert(!flow.m_recycled);
        auto newInternodeHandle = AllocateInternode();
        SetParentInternode(newInternodeHandle, targetHandle);
        auto &originalInternode = m_internodes[targetHandle];
        auto &newInternode = m_internodes[newInternodeHandle];

        if (branching) {
            auto newFlowHandle = AllocateFlow();
            auto &newFlow = m_flows[newFlowHandle];
            SetParentFlow(newFlowHandle, originalInternode.m_flowHandle);
            newInternode.m_flowHandle = newFlowHandle;
            newFlow.m_internodes.emplace_back(newInternodeHandle);
            newFlow.m_apical = false;
            if (targetHandle != m_flows[originalInternode.m_flowHandle].m_internodes.front()) {
                auto extendedFlowHandle = AllocateFlow();
                auto &extendedFlow = m_flows[extendedFlowHandle];
                extendedFlow.m_apical = true;
                SetParentFlow(extendedFlowHandle, originalInternode.m_flowHandle);
                //Find target internode.
                auto &originalFlow = m_flows[originalInternode.m_flowHandle];
                for (auto r = originalFlow.m_internodes.begin(); r != originalFlow.m_internodes.end(); r++) {
                    if (*r == targetHandle) {
                        extendedFlow.m_internodes.insert(extendedFlow.m_internodes.end(), r,
                                                         originalFlow.m_internodes.end());
                        originalFlow.m_internodes.erase(r, originalFlow.m_internodes.end());
                        break;
                    }
                }
                for (const auto &extractedInternodeHandle: extendedFlow.m_internodes) {
                    auto &extractedInternode = m_internodes[extractedInternodeHandle];
                    extractedInternode.m_flowHandle = extendedFlowHandle;
                }
            }
        } else {
            originalInternode.m_endNode = false;
            flow.m_internodes.emplace_back(newInternodeHandle);
            newInternode.m_flowHandle = originalInternode.m_flowHandle;
        }
        m_newVersion++;
        return newInternodeHandle;
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::RecycleFlow(FlowHandle handle) {
        assert(handle != 0);
        assert(!m_flows[handle].m_recycled);
        auto &flow = m_flows[handle];
        //Remove children
        auto children = flow.m_childHandles;
        for (const auto &child: children) {
            RecycleFlow(child);
        }
        //Detach from parent
        if (flow.m_parentHandle != -1) DetachChildFlow(flow.m_parentHandle, handle);
        //Remove internodes
        if (!flow.m_internodes.empty()) {
            //Detach first internode from parent.
            auto internodes = flow.m_internodes;
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
        auto flowHandle = internode.m_flowHandle;
        auto &flow = m_flows[flowHandle];
        if (handle == flow.m_internodes[0]) {
            RecycleFlow(internode.m_flowHandle);
            return;
        }
        //Collect list of subsequent internodes
        std::vector<InternodeHandle> subsequentInternodes;
        while (flow.m_internodes.back() != handle) {
            subsequentInternodes.emplace_back(flow.m_internodes.back());
            flow.m_internodes.pop_back();
        }
        subsequentInternodes.emplace_back(flow.m_internodes.back());
        flow.m_internodes.pop_back();
        assert(!flow.m_internodes.empty());
        //Detach from parent
        if (internode.m_parentHandle != -1) DetachChildInternode(internode.m_parentHandle, handle);
        //From end node remove one by one.
        InternodeHandle prev = -1;
        for (const auto &i: subsequentInternodes) {
            auto children = m_internodes[i].m_childHandles;
            for (const auto &childInternodeHandle: children) {
                if (childInternodeHandle == prev) continue;
                auto &child = m_internodes[childInternodeHandle];
                assert(!child.m_recycled);
                auto childBranchHandle = child.m_flowHandle;
                if (childBranchHandle != flowHandle) {
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

    template<typename InternodeData>
    bool Internode<InternodeData>::IsEndNode() const {
        return m_endNode;
    }

    template<typename InternodeData>
    bool Internode<InternodeData>::IsRecycled() const {
        return m_recycled;
    }

    template<typename InternodeData>
    InternodeHandle Internode<InternodeData>::GetHandle() const {
        return m_handle;
    }

    template<typename InternodeData>
    InternodeHandle Internode<InternodeData>::GetParentHandle() const {
        return m_parentHandle;
    }

    template<typename InternodeData>
    FlowHandle Internode<InternodeData>::GetFlowHandle() const {
        return m_flowHandle;
    }

    template<typename InternodeData>
    const std::vector<InternodeHandle> &Internode<InternodeData>::RefChildHandles() const {
        return m_childHandles;
    }

    template<typename FlowData>
    Flow<FlowData>::Flow(int handle) {
        m_handle = handle;
        m_recycled = false;
        m_data = {};
        m_info = {};
        m_apical = false;
    }

    template<typename FlowData>
    const std::vector<InternodeHandle> &Flow<FlowData>::RefInternodes() const {
        return m_internodes;
    }

    template<typename FlowData>
    FlowHandle Flow<FlowData>::GetParentHandle() const {
        return m_parentHandle;
    }

    template<typename FlowData>
    const std::vector<FlowHandle> &Flow<FlowData>::RefChildHandles() const {
        return m_childHandles;
    }

    template<typename FlowData>
    bool Flow<FlowData>::IsRecycled() const {
        return m_recycled;
    }

    template<typename FlowData>
    FlowHandle Flow<FlowData>::GetHandle() const { return m_handle; }

    template<typename FlowData>
    bool Flow<FlowData>::IsApical() const {
        return m_apical;
    }

    template<typename FlowData, typename InternodeData>
    TreeSkeleton<FlowData, InternodeData>::TreeSkeleton() {
        AllocateFlow();
        AllocateInternode();
        auto &rootBranch = m_flows[0];
        auto &rootInternode = m_internodes[0];
        rootInternode.m_flowHandle = 0;
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
        auto &children = targetInternode.m_childHandles;
        for (int i = 0; i < children.size(); i++) {
            if (children[i] == childHandle) {
                children[i] = children.back();
                children.pop_back();
                childInternode.m_parentHandle = -1;
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
        targetInternode.m_parentHandle = parentHandle;
        parentInternode.m_childHandles.emplace_back(targetHandle);
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
            if (firstInternode.m_parentHandle != -1)
                DetachChildInternode(firstInternode.m_parentHandle, firstInternodeHandle);
        }

        auto &children = targetBranch.m_childHandles;
        for (int i = 0; i < children.size(); i++) {
            if (children[i] == childHandle) {
                children[i] = children.back();
                children.pop_back();
                childBranch.m_parentHandle = -1;
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
        targetBranch.m_parentHandle = parentHandle;
        parentBranch.m_childHandles.emplace_back(targetHandle);
    }

    template<typename FlowData, typename InternodeData>
    FlowHandle TreeSkeleton<FlowData, InternodeData>::AllocateFlow() {
        if (m_flowPool.empty()) {
            auto newBranch = m_flows.emplace_back(m_flows.size());
            return newBranch.m_handle;
        }
        auto handle = m_flowPool.front();
        m_flowPool.pop();
        auto &flow = m_flows[handle];
        flow.m_recycled = false;
        return handle;
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::RecycleFlowSingle(FlowHandle handle) {
        assert(!m_flows[handle].m_recycled);
        auto &flow = m_flows[handle];
        flow.m_parentHandle = -1;
        flow.m_childHandles.clear();
        flow.m_internodes.clear();

        flow.m_data = {};
        flow.m_info = {};

        flow.m_recycled = true;
        flow.m_apical = false;
        m_flowPool.emplace(handle);
    }

    template<typename FlowData, typename InternodeData>
    void TreeSkeleton<FlowData, InternodeData>::RecycleInternodeSingle(InternodeHandle handle) {
        assert(!m_internodes[handle].m_recycled);
        auto &internode = m_internodes[handle];
        internode.m_parentHandle = -1;
        internode.m_flowHandle = -1;
        internode.m_endNode = true;
        internode.m_childHandles.clear();

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
        for (const auto &flowHandle: sortedBranchList) {
            auto &flow = m_flows[flowHandle];
            auto &firstInternode = m_internodes[flow.m_internodes.front()];
            auto &lastInternode = m_internodes[flow.m_internodes.back()];
            flow.m_info.m_startThickness = firstInternode.m_info.m_thickness;
            flow.m_info.m_globalStartPosition = firstInternode.m_info.m_globalPosition;
            flow.m_info.m_globalStartRotation = firstInternode.m_info.m_localRotation;

            flow.m_info.m_endThickness = lastInternode.m_info.m_thickness;
            flow.m_info.m_globalEndPosition = lastInternode.m_info.m_globalPosition +
                                              lastInternode.m_info.m_length *
                                              (lastInternode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
            flow.m_info.m_globalEndRotation = lastInternode.m_info.m_globalRotation;
        }
    }

#pragma endregion
#pragma endregion
}