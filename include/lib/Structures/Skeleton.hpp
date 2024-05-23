#pragma once


namespace EcoSysLab {
	typedef int SkeletonNodeHandle;
	typedef int SkeletonFlowHandle;

#pragma region Structural Info
	struct SkeletonNodeWound
	{
		bool m_apical = false;
		glm::quat m_localRotation = glm::vec3(0.f);
		float m_thickness = 0.f;
		float m_healing = 0.f;
	};

	struct SkeletonNodeInfo {
		bool m_locked = false;
		/**
		 * \brief The global position at the start of the node.
		 */
		glm::vec3 m_globalPosition = glm::vec3(0.0f);
		/**
		 * \brief The global rotation at the start of the node.
		 */
		glm::quat m_globalRotation = glm::vec3(0.0f);

		float m_length = 0.0f;
		float m_thickness = 0.1f;
		float m_rootDistance = 0.0f;
		float m_endDistance = 0.0f;
		int m_chainIndex = 0;
		glm::quat m_regulatedGlobalRotation = glm::vec3(0.0f);
		std::vector<SkeletonNodeWound> m_wounds;

		float m_leaves = 1.f;
		float m_fruits = 1.f;
		glm::vec4 m_color = glm::vec4(1.0f);

		int m_clusterIndex = 0;

		[[nodiscard]] glm::vec3 GetGlobalEndPosition() const;
		[[nodiscard]] glm::vec3 GetGlobalDirection() const;
	};

	inline glm::vec3 SkeletonNodeInfo::GetGlobalEndPosition() const
	{
		return m_globalPosition + glm::normalize(m_globalRotation * glm::vec3(0, 0, -1)) * m_length;
	}

	inline glm::vec3 SkeletonNodeInfo::GetGlobalDirection() const
	{
		return glm::normalize(m_globalRotation * glm::vec3(0, 0, -1));
	}
	struct SkeletonFlowInfo {
		glm::vec3 m_globalStartPosition = glm::vec3(0.0f);
		glm::quat m_globalStartRotation = glm::vec3(0.0f);
		float m_startThickness = 0.0f;

		glm::vec3 m_globalEndPosition = glm::vec3(0.0f);
		glm::quat m_globalEndRotation = glm::vec3(0.0f);
		float m_endThickness = 0.0f;

		float m_flowLength = 0.0f;
	};
#pragma endregion

	template<typename SkeletonNodeData>
	class SkeletonNode {
		template<typename FD>
		friend class SkeletonFlow;

		template<typename SD, typename FD, typename ID>
		friend class Skeleton;

		template<typename SD, typename FD, typename ID>
		friend class SkeletonSerializer;

		bool m_endNode = true;
		bool m_recycled = false;
		SkeletonNodeHandle m_handle = -1;
		SkeletonFlowHandle m_flowHandle = -1;
		SkeletonNodeHandle m_parentHandle = -1;
		std::vector<SkeletonNodeHandle> m_childHandles;
		bool m_apical = true;
		int m_index = -1;
	public:
		SkeletonNodeData m_data;
		/**
		 * The structural information of current node.
		 */
		SkeletonNodeInfo m_info;

		/**
		 * Whether this node is the end node.
		 * @return True if this is end node, false else wise.
		 */
		[[nodiscard]] bool IsEndNode() const;

		/**
		 * Whether this node is recycled (removed).
		 * @return True if this node is recycled (removed), false else wise.
		 */
		[[nodiscard]] bool IsRecycled() const;
		/**
		 * Whether this node is apical.
		 * @return True if this node is apical, false else wise.
		 */
		[[nodiscard]] bool IsApical() const;
		/**
		 * Get the handle of self.
		 * @return NodeHandle of current node.
		 */
		[[nodiscard]] SkeletonNodeHandle GetHandle() const;

		/**
		 * Get the handle of parent.
		 * @return NodeHandle of parent node.
		 */
		[[nodiscard]] SkeletonNodeHandle GetParentHandle() const;

		/**
		 * Get the handle to belonged flow.
		 * @return FlowHandle of belonged flow.
		 */
		[[nodiscard]] SkeletonFlowHandle GetFlowHandle() const;

		/**
		 * Access the children by their handles.
		 * @return The list of handles.
		 */
		[[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekChildHandles() const;

		/**
		 * Access the children by their handles. Allow modification. Potentially break the skeleton structure!
		 * @return The list of handles.
		 */
		[[nodiscard]] std::vector<SkeletonNodeHandle>& UnsafeRefChildHandles();
		SkeletonNode() = default;
		SkeletonNode(SkeletonNodeHandle handle);

		[[nodiscard]] int GetIndex() const;
	};

	template<typename SkeletonFlowData>
	class SkeletonFlow {
		template<typename SD, typename FD, typename ID>
		friend class Skeleton;

		template<typename SD, typename FD, typename ID>
		friend class SkeletonSerializer;

		bool m_recycled = false;
		SkeletonFlowHandle m_handle = -1;
		std::vector<SkeletonNodeHandle> m_nodes;
		SkeletonFlowHandle m_parentHandle = -1;
		std::vector<SkeletonFlowHandle> m_childHandles;
		bool m_apical = false;
		int m_index = -1;
	public:
		SkeletonFlowData m_data;
		SkeletonFlowInfo m_info;

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
		[[nodiscard]] SkeletonFlowHandle GetHandle() const;

		/**
		 * Get the handle of parent.
		 * @return FlowHandle of parent flow.
		 */
		[[nodiscard]] SkeletonFlowHandle GetParentHandle() const;

		/**
		 * Access the children by their handles.
		 * @return The list of handles.
		 */
		[[nodiscard]] const std::vector<SkeletonFlowHandle>& PeekChildHandles() const;

		/**
		 * Access the nodes that belongs to this flow.
		 * @return The list of handles.
		 */
		[[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekNodeHandles() const;
		SkeletonFlow() = default;
		explicit SkeletonFlow(SkeletonFlowHandle handle);

		[[nodiscard]] int GetIndex() const;
	};

	struct SkeletonClusterSettings
	{
		
	};

	template<typename SkeletonData, typename FlowData, typename NodeData>
	class Skeleton {
		template<typename SD, typename FD, typename ID>
		friend class Skeleton;

		template<typename SD, typename FD, typename ID>
		friend class SkeletonSerializer;

		std::vector<SkeletonFlow<FlowData>> m_flows;
		std::vector<SkeletonNode<NodeData>> m_nodes;
		std::queue<SkeletonNodeHandle> m_nodePool;
		std::queue<SkeletonFlowHandle> m_flowPool;

		int m_newVersion = 0;
		int m_version = -1;
		std::vector<SkeletonNodeHandle> m_sortedNodeList;
		std::vector<SkeletonFlowHandle> m_sortedFlowList;

		SkeletonNodeHandle AllocateNode();

		void RecycleNodeSingle(SkeletonNodeHandle handle, const std::function<void(SkeletonNodeHandle)>& nodeHandler);

		void RecycleFlowSingle(SkeletonFlowHandle handle, const std::function<void(SkeletonFlowHandle)>& flowHandler);

		SkeletonFlowHandle AllocateFlow();

		void SetParentFlow(SkeletonFlowHandle targetHandle, SkeletonFlowHandle parentHandle);

		void DetachChildFlow(SkeletonFlowHandle targetHandle, SkeletonFlowHandle childHandle);

		void SetParentNode(SkeletonNodeHandle targetHandle, SkeletonNodeHandle parentHandle);

		void DetachChildNode(SkeletonNodeHandle targetHandle, SkeletonNodeHandle childHandle);

		int m_maxNodeIndex = -1;
		int m_maxFlowIndex = -1;

		std::vector<SkeletonNodeHandle> m_baseNodeList;

		void RefreshBaseNodeList();

	public:

		void CalculateClusters(const SkeletonClusterSettings& clusterSettings);

		template<typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
		void Clone(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& srcSkeleton);

		[[nodiscard]] int GetMaxNodeIndex() const;
		[[nodiscard]] int GetMaxFlowIndex() const;
		SkeletonData m_data;

		void CalculateDistance();
		void CalculateRegulatedGlobalRotation();
		/**
		 * Recycle (Remove) a node, the descendants of this node will also be recycled. The relevant flow will also be removed/restructured.
		 * @param handle The handle of the node to be removed. Must be valid (non-zero and the node should not be recycled prior to this operation).
		 * @param flowHandler Function to be called right before a flow in recycled.
		 * @param nodeHandler Function to be called right before a node in recycled.
		 */
		void RecycleNode(SkeletonNodeHandle handle,
			const std::function<void(SkeletonFlowHandle)>& flowHandler,
			const std::function<void(SkeletonNodeHandle)>& nodeHandler);

		/**
		 * Recycle (Remove) a flow, the descendants of this flow will also be recycled. The relevant node will also be removed/restructured.
		 * @param handle The handle of the flow to be removed. Must be valid (non-zero and the flow should not be recycled prior to this operation).
		 * @param flowHandler Function to be called right before a flow in recycled.
		 * @param nodeHandler Function to be called right before a node in recycled.
		 */
		void RecycleFlow(SkeletonFlowHandle handle,
			const std::function<void(SkeletonFlowHandle)>& flowHandler,
			const std::function<void(SkeletonNodeHandle)>& nodeHandler);

		/**
		 * Branch/prolong node during growth process. The flow structure will also be updated.
		 * @param targetHandle The handle of the node to branch/prolong
		 * @param branching True if branching, false if prolong. During branching, 2 new flows will be generated.
		 * @return The handle of new node.
		 */
		[[nodiscard]] SkeletonNodeHandle Extend(SkeletonNodeHandle targetHandle, bool branching);

		/**
		 * To retrieve a list of handles of all nodes contained within the tree.
		 * @return The list of handles of nodes sorted from root to ends.
		 */
		[[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekBaseNodeList();

		/**
		 * To retrieve a list of handles of all nodes contained within the tree.
		 * @return The list of handles of nodes sorted from root to ends.
		 */
		[[nodiscard]] const std::vector<SkeletonNodeHandle>& PeekSortedNodeList() const;

		[[nodiscard]] std::vector<SkeletonNodeHandle> GetSubTree(SkeletonNodeHandle baseNodeHandle) const;
		[[nodiscard]] std::vector<SkeletonNodeHandle> GetChainToRoot(SkeletonNodeHandle endNodeHandle) const;

		[[nodiscard]] std::vector<SkeletonNodeHandle> GetNodeListBaseIndex(unsigned baseIndex) const;
		/**
		 * To retrieve a list of handles of all flows contained within the tree.
		 * @return The list of handles of flows sorted from root to ends.
		 */
		[[nodiscard]] const std::vector<SkeletonFlowHandle>& PeekSortedFlowList() const;

		[[nodiscard]] std::vector<SkeletonFlow<FlowData>>& RefRawFlows();

		[[nodiscard]] std::vector<SkeletonNode<NodeData>>& RefRawNodes();

		[[nodiscard]] const std::vector<SkeletonFlow<FlowData>>& PeekRawFlows() const;

		[[nodiscard]] const std::vector<SkeletonNode<NodeData>>& PeekRawNodes() const;

		/**
		 *  Force the structure to sort the node and flow list.
		 *  \n!!You MUST call this after you prune the tree or altered the tree structure manually!!
		 */
		void SortLists();

		Skeleton(unsigned initialNodeCount = 1);

		/**
		 * Get the structural version of the tree. The version will change when the tree structure changes.
		 * @return The version
		 */
		[[nodiscard]] int GetVersion() const;

		/**
		 * Calculate the structural information of the flows.
		 */
		void CalculateFlows();

		/**
		 * Retrieve a modifiable reference to the node with the handle.
		 * @param handle The handle to the target node.
		 * @return The modifiable reference to the node.
		 */
		SkeletonNode<NodeData>& RefNode(SkeletonNodeHandle handle);

		/**
		 * Retrieve a modifiable reference to the flow with the handle.
		 * @param handle The handle to the target flow.
		 * @return The modifiable reference to the flow.
		 */
		SkeletonFlow<FlowData>& RefFlow(SkeletonFlowHandle handle);

		/**
		 * Retrieve a non-modifiable reference to the node with the handle.
		 * @param handle The handle to the target node.
		 * @return The non-modifiable reference to the node.
		 */
		[[nodiscard]] const SkeletonNode<NodeData>& PeekNode(SkeletonNodeHandle handle) const;

		/**
		 * Retrieve a non-modifiable reference to the flow with the handle.
		 * @param handle The handle to the target flow.
		 * @return The non-modifiable reference to the flow.
		 */
		[[nodiscard]] const SkeletonFlow<FlowData>& PeekFlow(SkeletonFlowHandle handle) const;

		/**
		 * The min value of the bounding box of current tree structure.
		 */
		glm::vec3 m_min = glm::vec3(0.0f);

		/**
		 * The max value of the bounding box of current tree structure.
		 */
		glm::vec3 m_max = glm::vec3(0.0f);
	};

	struct BaseSkeletonData {};
	struct BaseFlowData {};
	struct BaseNodeData {};

	typedef Skeleton<BaseSkeletonData, BaseFlowData, BaseNodeData> BaseSkeleton;

#pragma region TreeSkeleton
#pragma region Helper

	template<typename SkeletonData, typename FlowData, typename NodeData>
	SkeletonFlow<FlowData>& Skeleton<SkeletonData, FlowData, NodeData>::RefFlow(SkeletonFlowHandle handle) {
		assert(handle >= 0 && handle < m_flows.size());
		return m_flows[handle];
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	const SkeletonFlow<FlowData>& Skeleton<SkeletonData, FlowData, NodeData>::PeekFlow(SkeletonFlowHandle handle) const {
		assert(handle >= 0 && handle < m_flows.size());
		return m_flows[handle];
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	SkeletonNode<NodeData>&
		Skeleton<SkeletonData, FlowData, NodeData>::RefNode(SkeletonNodeHandle handle) {
		assert(handle >= 0 && handle < m_nodes.size());
		return m_nodes[handle];
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	const SkeletonNode<NodeData>&
		Skeleton<SkeletonData, FlowData, NodeData>::PeekNode(SkeletonNodeHandle handle) const {
		assert(handle >= 0 && handle < m_nodes.size());
		return m_nodes[handle];
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::SortLists() {
		if (m_version == m_newVersion) return;
		if (m_nodes.empty()) return;
		m_version = m_newVersion;
		m_sortedFlowList.clear();
		m_sortedNodeList.clear();
		RefreshBaseNodeList();
		std::queue<SkeletonFlowHandle> flowWaitList;
		std::queue<SkeletonNodeHandle> nodeWaitList;

		for (const auto& baseNodeHandle : m_baseNodeList)
		{
			nodeWaitList.push(baseNodeHandle);
			flowWaitList.push(m_nodes[baseNodeHandle].m_flowHandle);
		}

		while (!flowWaitList.empty()) {
			m_sortedFlowList.emplace_back(flowWaitList.front());
			flowWaitList.pop();
			for (const auto& i : m_flows[m_sortedFlowList.back()].m_childHandles) {
				assert(!m_flows[i].m_recycled);
				flowWaitList.push(i);
			}

		}

		while (!nodeWaitList.empty()) {
			m_sortedNodeList.emplace_back(nodeWaitList.front());
			nodeWaitList.pop();
			for (const auto& i : m_nodes[m_sortedNodeList.back()].m_childHandles) {
				assert(!m_nodes[i].m_recycled);
				nodeWaitList.push(i);
			}
		}
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	const std::vector<SkeletonFlowHandle>& Skeleton<SkeletonData, FlowData, NodeData>::PeekSortedFlowList() const {
		return m_sortedFlowList;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	const std::vector<SkeletonNodeHandle>&
		Skeleton<SkeletonData, FlowData, NodeData>::PeekSortedNodeList() const {
		return m_sortedNodeList;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	std::vector<SkeletonNodeHandle> Skeleton<SkeletonData, FlowData, NodeData>::GetSubTree(const SkeletonNodeHandle baseNodeHandle) const
	{
		std::vector<SkeletonNodeHandle> retVal{};
		std::queue<SkeletonNodeHandle> nodeHandles;
		nodeHandles.push(baseNodeHandle);
		while (!nodeHandles.empty())
		{
			auto nextNodeHandle = nodeHandles.front();
			retVal.emplace_back(nodeHandles.front());
			nodeHandles.pop();
			for (const auto& childHandle : m_nodes[nextNodeHandle].m_childHandles)
			{
				nodeHandles.push(childHandle);
			}
		}
		return retVal;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	std::vector<SkeletonNodeHandle> Skeleton<SkeletonData, FlowData, NodeData>::GetChainToRoot(
		const SkeletonNodeHandle endNodeHandle) const
	{
		std::vector<SkeletonNodeHandle> retVal{};
		SkeletonNodeHandle walker = endNodeHandle;
		while(walker != -1)
		{
			retVal.emplace_back(walker);
			walker = m_nodes[walker].m_parentHandle;
		}
		return retVal;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	std::vector<SkeletonNodeHandle> Skeleton<SkeletonData, FlowData, NodeData>::GetNodeListBaseIndex(unsigned baseIndex) const
	{
		std::vector<SkeletonNodeHandle> retVal{};
		for (const auto& i : m_sortedNodeList)
		{
			if (m_nodes[i].m_index >= baseIndex) retVal.push_back(i);
		}
		return retVal;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	SkeletonNodeHandle
		Skeleton<SkeletonData, FlowData, NodeData>::Extend(SkeletonNodeHandle targetHandle, const bool branching) {
		assert(targetHandle < m_nodes.size());
		auto& targetNode = m_nodes[targetHandle];
		assert(!targetNode.m_recycled);
		assert(targetNode.m_flowHandle < m_flows.size());
		auto& flow = m_flows[targetNode.m_flowHandle];
		assert(!flow.m_recycled);
		auto newNodeHandle = AllocateNode();
		SetParentNode(newNodeHandle, targetHandle);
		auto& originalNode = m_nodes[targetHandle];
		auto& newNode = m_nodes[newNodeHandle];
		originalNode.m_endNode = false;
		if (branching) {
			auto newFlowHandle = AllocateFlow();
			auto& newFlow = m_flows[newFlowHandle];

			newNode.m_flowHandle = newFlowHandle;
			newNode.m_apical = false;
			newFlow.m_nodes.emplace_back(newNodeHandle);
			newFlow.m_apical = false;
			if (targetHandle != m_flows[originalNode.m_flowHandle].m_nodes.back()) {
				auto extendedFlowHandle = AllocateFlow();
				auto& extendedFlow = m_flows[extendedFlowHandle];
				extendedFlow.m_apical = true;
				//Find target node.
				auto& originalFlow = m_flows[originalNode.m_flowHandle];
				for (auto r = originalFlow.m_nodes.begin(); r != originalFlow.m_nodes.end(); ++r) {
					if (*r == targetHandle) {
						extendedFlow.m_nodes.insert(extendedFlow.m_nodes.end(), r + 1,
							originalFlow.m_nodes.end());
						originalFlow.m_nodes.erase(r + 1, originalFlow.m_nodes.end());
						break;
					}
				}
				for (const auto& extractedNodeHandle : extendedFlow.m_nodes) {
					auto& extractedNode = m_nodes[extractedNodeHandle];
					extractedNode.m_flowHandle = extendedFlowHandle;
				}
				extendedFlow.m_childHandles = originalFlow.m_childHandles;
				originalFlow.m_childHandles.clear();
				for (const auto& childFlowHandle : extendedFlow.m_childHandles) {
					m_flows[childFlowHandle].m_parentHandle = extendedFlowHandle;
				}
				SetParentFlow(extendedFlowHandle, originalNode.m_flowHandle);
			}
			SetParentFlow(newFlowHandle, originalNode.m_flowHandle);
		}
		else {
			flow.m_nodes.emplace_back(newNodeHandle);
			newNode.m_flowHandle = originalNode.m_flowHandle;
			newNode.m_apical = true;
		}
		m_newVersion++;
		return newNodeHandle;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	const std::vector<SkeletonNodeHandle>& Skeleton<SkeletonData, FlowData, NodeData>::PeekBaseNodeList()
	{
		RefreshBaseNodeList();
		return m_baseNodeList;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::RecycleFlow(SkeletonFlowHandle handle,
		const std::function<void(SkeletonFlowHandle)>& flowHandler,
		const std::function<void(SkeletonNodeHandle)>& nodeHandler) {
		assert(handle != 0);
		assert(!m_flows[handle].m_recycled);
		auto& flow = m_flows[handle];
		//Remove children
		auto children = flow.m_childHandles;
		for (const auto& child : children) {
			if (m_flows[child].m_recycled) continue;
			RecycleFlow(child, flowHandler, nodeHandler);
		}
		//Detach from parent
		auto parentHandle = flow.m_parentHandle;
		if (parentHandle != -1) DetachChildFlow(parentHandle, handle);
		//Remove node
		if (!flow.m_nodes.empty()) {
			//Detach first node from parent.
			auto nodes = flow.m_nodes;
			for (const auto& i : nodes) {
				RecycleNodeSingle(i, nodeHandler);
			}
		}
		RecycleFlowSingle(handle, flowHandler);
		m_newVersion++;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::RecycleNode(SkeletonNodeHandle handle,
		const std::function<void(SkeletonFlowHandle)>& flowHandler,
		const std::function<void(SkeletonNodeHandle)>& nodeHandler) {
		assert(handle != 0);
		assert(!m_nodes[handle].m_recycled);
		auto& node = m_nodes[handle];
		auto flowHandle = node.m_flowHandle;
		auto& flow = m_flows[flowHandle];
		if (handle == flow.m_nodes[0]) {
			auto parentFlowHandle = flow.m_parentHandle;

			RecycleFlow(node.m_flowHandle, flowHandler, nodeHandler);
			if (parentFlowHandle != -1) {
				//Connect parent branch with the only apical child flow.
				auto& parentFlow = m_flows[parentFlowHandle];
				if (parentFlow.m_childHandles.size() == 1) {
					auto childHandle = parentFlow.m_childHandles[0];
					auto& childFlow = m_flows[childHandle];
					if (childFlow.m_apical) {
						for (const auto& nodeHandle : childFlow.m_nodes) {
							m_nodes[nodeHandle].m_flowHandle = parentFlowHandle;
						}
						for (const auto& grandChildFlowHandle : childFlow.m_childHandles) {
							m_flows[grandChildFlowHandle].m_parentHandle = parentFlowHandle;
						}
						parentFlow.m_nodes.insert(parentFlow.m_nodes.end(), childFlow.m_nodes.begin(),
							childFlow.m_nodes.end());
						parentFlow.m_childHandles.clear();
						parentFlow.m_childHandles.insert(parentFlow.m_childHandles.end(),
							childFlow.m_childHandles.begin(),
							childFlow.m_childHandles.end());
						RecycleFlowSingle(childHandle, flowHandler);
					}
				}
			}
			return;
		}
		//Collect list of subsequent nodes
		std::vector<SkeletonNodeHandle> subsequentNodes;
		while (flow.m_nodes.back() != handle) {
			subsequentNodes.emplace_back(flow.m_nodes.back());
			flow.m_nodes.pop_back();
		}
		subsequentNodes.emplace_back(flow.m_nodes.back());
		flow.m_nodes.pop_back();
		assert(!flow.m_nodes.empty());
		//Detach from parent
		if (node.m_parentHandle != -1) DetachChildNode(node.m_parentHandle, handle);
		//From end node remove one by one.
		SkeletonNodeHandle prev = -1;
		for (const auto& i : subsequentNodes) {
			auto children = m_nodes[i].m_childHandles;
			for (const auto& childNodeHandle : children) {
				if (childNodeHandle == prev) continue;
				auto& child = m_nodes[childNodeHandle];
				assert(!child.m_recycled);
				auto childBranchHandle = child.m_flowHandle;
				if (childBranchHandle != flowHandle) {
					RecycleFlow(childBranchHandle, flowHandler, nodeHandler);
				}
			}
			prev = i;
			RecycleNodeSingle(i, nodeHandler);

		}
		m_newVersion++;
	}

#pragma endregion
#pragma region Internal

	template<typename NodeData>
	SkeletonNode<NodeData>::SkeletonNode(const SkeletonNodeHandle handle) {
		m_handle = handle;
		m_recycled = false;
		m_endNode = true;
		m_data = {};
		m_info = {};
		m_index = -1;
	}

	template<typename NodeData>
	bool SkeletonNode<NodeData>::IsEndNode() const {
		return m_endNode;
	}

	template<typename NodeData>
	bool SkeletonNode<NodeData>::IsRecycled() const {
		return m_recycled;
	}

	template <typename NodeData>
	bool SkeletonNode<NodeData>::IsApical() const
	{
		return m_apical;
	}

	template<typename NodeData>
	SkeletonNodeHandle SkeletonNode<NodeData>::GetHandle() const {
		return m_handle;
	}

	template<typename NodeData>
	SkeletonNodeHandle SkeletonNode<NodeData>::GetParentHandle() const {
		return m_parentHandle;
	}

	template<typename NodeData>
	SkeletonFlowHandle SkeletonNode<NodeData>::GetFlowHandle() const {
		return m_flowHandle;
	}

	template<typename NodeData>
	const std::vector<SkeletonNodeHandle>& SkeletonNode<NodeData>::PeekChildHandles() const {
		return m_childHandles;
	}

	template <typename NodeData>
	std::vector<SkeletonNodeHandle>& SkeletonNode<NodeData>::UnsafeRefChildHandles()
	{
		return m_childHandles;
	}

	template <typename NodeData>
	int SkeletonNode<NodeData>::GetIndex() const
	{
		return m_index;
	}

	template<typename FlowData>
	SkeletonFlow<FlowData>::SkeletonFlow(const SkeletonFlowHandle handle) {
		m_handle = handle;
		m_recycled = false;
		m_data = {};
		m_info = {};
		m_apical = false;
		m_index = -1;
	}

	template <typename FlowData>
	int SkeletonFlow<FlowData>::GetIndex() const
	{
		return m_index;
	}

	template<typename FlowData>
	const std::vector<SkeletonNodeHandle>& SkeletonFlow<FlowData>::PeekNodeHandles() const {
		return m_nodes;
	}

	template<typename FlowData>
	SkeletonFlowHandle SkeletonFlow<FlowData>::GetParentHandle() const {
		return m_parentHandle;
	}

	template<typename FlowData>
	const std::vector<SkeletonFlowHandle>& SkeletonFlow<FlowData>::PeekChildHandles() const {
		return m_childHandles;
	}

	template<typename FlowData>
	bool SkeletonFlow<FlowData>::IsRecycled() const {
		return m_recycled;
	}

	template<typename FlowData>
	SkeletonFlowHandle SkeletonFlow<FlowData>::GetHandle() const { return m_handle; }

	template<typename FlowData>
	bool SkeletonFlow<FlowData>::IsApical() const {
		return m_apical;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	Skeleton<SkeletonData, FlowData, NodeData>::Skeleton(const unsigned initialNodeCount) {
		m_maxNodeIndex = -1;
		m_maxFlowIndex = -1;
		for (int i = 0; i < initialNodeCount; i++) {
			auto flowHandle = AllocateFlow();
			auto nodeHandle = AllocateNode();
			auto& rootFlow = m_flows[flowHandle];
			auto& rootNode = m_nodes[nodeHandle];
			rootNode.m_flowHandle = flowHandle;
			rootFlow.m_nodes.emplace_back(nodeHandle);
			m_baseNodeList.emplace_back(nodeHandle);
		}
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::DetachChildNode(SkeletonNodeHandle targetHandle,
		SkeletonNodeHandle childHandle) {
		assert(targetHandle >= 0 && childHandle >= 0 && targetHandle < m_nodes.size() &&
			childHandle < m_nodes.size());
		auto& targetNode = m_nodes[targetHandle];
		auto& childNode = m_nodes[childHandle];
		assert(!targetNode.m_recycled);
		assert(!childNode.m_recycled);
		auto& children = targetNode.m_childHandles;
		for (int i = 0; i < children.size(); i++) {
			if (children[i] == childHandle) {
				children[i] = children.back();
				children.pop_back();
				childNode.m_parentHandle = -1;
				if (children.empty()) targetNode.m_endNode = true;
				return;
			}
		}
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::RefreshBaseNodeList()
	{
		std::vector<SkeletonNodeHandle> temp;
		for (const auto& i : m_baseNodeList) if (!m_nodes[i].m_recycled && m_nodes[i].m_parentHandle == -1) temp.emplace_back(i);
		m_baseNodeList = temp;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::CalculateClusters(const SkeletonClusterSettings& clusterSettings)
	{

	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::Clone(
		const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& srcSkeleton)
	{
		m_data = {};
		m_flowPool = srcSkeleton.m_flowPool;
		m_nodePool = srcSkeleton.m_nodePool;
		m_sortedNodeList = srcSkeleton.m_sortedNodeList;
		m_sortedFlowList = srcSkeleton.m_sortedFlowList;

		m_nodes.resize(srcSkeleton.m_nodes.size());
		for (int i = 0; i < srcSkeleton.m_nodes.size(); i++)
		{
			m_nodes[i].m_info = srcSkeleton.m_nodes[i].m_info;

			m_nodes[i].m_endNode = srcSkeleton.m_nodes[i].m_endNode;
			m_nodes[i].m_recycled = srcSkeleton.m_nodes[i].m_recycled;
			m_nodes[i].m_handle = srcSkeleton.m_nodes[i].m_handle;
			m_nodes[i].m_flowHandle = srcSkeleton.m_nodes[i].m_flowHandle;
			m_nodes[i].m_parentHandle = srcSkeleton.m_nodes[i].m_parentHandle;
			m_nodes[i].m_childHandles = srcSkeleton.m_nodes[i].m_childHandles;
			m_nodes[i].m_apical = srcSkeleton.m_nodes[i].m_apical;
			m_nodes[i].m_index = srcSkeleton.m_nodes[i].m_index;
		}

		m_flows.resize(srcSkeleton.m_flows.size());
		for (int i = 0; i < srcSkeleton.m_flows.size(); i++)
		{
			m_flows[i].m_info = srcSkeleton.m_flows[i].m_info;

			m_flows[i].m_recycled = srcSkeleton.m_flows[i].m_recycled;
			m_flows[i].m_handle = srcSkeleton.m_flows[i].m_handle;
			m_flows[i].m_nodes = srcSkeleton.m_flows[i].m_nodes;
			m_flows[i].m_parentHandle = srcSkeleton.m_flows[i].m_parentHandle;
			m_flows[i].m_apical = srcSkeleton.m_flows[i].m_apical;
		}
		m_maxNodeIndex = srcSkeleton.m_maxNodeIndex;
		m_maxFlowIndex = srcSkeleton.m_maxFlowIndex;
		m_newVersion = srcSkeleton.m_newVersion;
		m_version = srcSkeleton.m_version;
		m_min = srcSkeleton.m_min;
		m_max = srcSkeleton.m_max;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	int Skeleton<SkeletonData, FlowData, NodeData>::GetMaxNodeIndex() const
	{
		return m_maxNodeIndex;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	int Skeleton<SkeletonData, FlowData, NodeData>::GetMaxFlowIndex() const
	{
		return m_maxFlowIndex;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::CalculateDistance()
	{
		for (const auto& nodeHandle : m_sortedNodeList) {
			auto& node = m_nodes[nodeHandle];
			auto& nodeInfo = node.m_info;
			if (node.GetParentHandle() == -1) {
				nodeInfo.m_rootDistance = nodeInfo.m_length;
				nodeInfo.m_chainIndex = 0;
			}
			else {
				const auto& parentInternode = m_nodes[node.GetParentHandle()];
				nodeInfo.m_rootDistance = parentInternode.m_info.m_rootDistance + nodeInfo.m_length;

				if(node.IsApical())
				{
					node.m_info.m_chainIndex = parentInternode.m_info.m_chainIndex + 1;
				}else
				{
					node.m_info.m_chainIndex = 0;
				}
			}
		}
		for (auto it = m_sortedNodeList.rbegin(); it != m_sortedNodeList.rend(); ++it) {
			auto& node = m_nodes[*it];
			float maxDistanceToAnyBranchEnd = 0;
			node.m_info.m_endDistance = 0;
			for (const auto& i : node.PeekChildHandles())
			{
				const auto& childNode = m_nodes[i];
				const float childMaxDistanceToAnyBranchEnd =
					childNode.m_info.m_endDistance +
					childNode.m_info.m_length;
				maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);
			}
			node.m_info.m_endDistance = maxDistanceToAnyBranchEnd;
		}
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::CalculateRegulatedGlobalRotation()
	{
		m_min = glm::vec3(FLT_MAX);
		m_max = glm::vec3(-FLT_MAX);
		for (const auto& nodeHandle : m_sortedNodeList) {
			auto& node = m_nodes[nodeHandle];
			auto& nodeInfo = node.m_info;
			m_min = glm::min(m_min, node.m_info.m_globalPosition);
			m_min = glm::min(m_min, node.m_info.GetGlobalEndPosition());
			m_max = glm::max(m_max, node.m_info.m_globalPosition);
			m_max = glm::max(m_max, node.m_info.GetGlobalEndPosition());
			if (node.m_parentHandle != -1) {
				auto& parentInfo = m_nodes[node.m_parentHandle].m_info;
				auto front = nodeInfo.m_globalRotation * glm::vec3(0, 0, -1);
				auto parentRegulatedUp = parentInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
				auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentRegulatedUp), front));
				nodeInfo.m_regulatedGlobalRotation = glm::quatLookAt(front, regulatedUp);
			}
			else
			{
				nodeInfo.m_regulatedGlobalRotation = nodeInfo.m_globalRotation;
			}
		}
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::SetParentNode(SkeletonNodeHandle targetHandle,
		SkeletonNodeHandle parentHandle) {
		assert(targetHandle >= 0 && parentHandle >= 0 && targetHandle < m_nodes.size() &&
			parentHandle < m_nodes.size());
		auto& targetNode = m_nodes[targetHandle];
		auto& parentNode = m_nodes[parentHandle];
		assert(!targetNode.m_recycled);
		assert(!parentNode.m_recycled);
		targetNode.m_parentHandle = parentHandle;
		parentNode.m_childHandles.emplace_back(targetHandle);
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void
		Skeleton<SkeletonData, FlowData, NodeData>::DetachChildFlow(SkeletonFlowHandle targetHandle,
			SkeletonFlowHandle childHandle) {
		assert(targetHandle >= 0 && childHandle >= 0 && targetHandle < m_flows.size() &&
			childHandle < m_flows.size());
		auto& targetBranch = m_flows[targetHandle];
		auto& childBranch = m_flows[childHandle];
		assert(!targetBranch.m_recycled);
		assert(!childBranch.m_recycled);

		if (!childBranch.m_nodes.empty()) {
			auto firstNodeHandle = childBranch.m_nodes[0];
			auto& firstNode = m_nodes[firstNodeHandle];
			if (firstNode.m_parentHandle != -1)
				DetachChildNode(firstNode.m_parentHandle, firstNodeHandle);
		}

		auto& children = targetBranch.m_childHandles;
		for (int i = 0; i < children.size(); i++) {
			if (children[i] == childHandle) {
				children[i] = children.back();
				children.pop_back();
				childBranch.m_parentHandle = -1;
				return;
			}
		}
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void
		Skeleton<SkeletonData, FlowData, NodeData>::SetParentFlow(SkeletonFlowHandle targetHandle,
			SkeletonFlowHandle parentHandle) {
		assert(targetHandle >= 0 && parentHandle >= 0 && targetHandle < m_flows.size() &&
			parentHandle < m_flows.size());
		auto& targetBranch = m_flows[targetHandle];
		auto& parentBranch = m_flows[parentHandle];
		assert(!targetBranch.m_recycled);
		assert(!parentBranch.m_recycled);
		targetBranch.m_parentHandle = parentHandle;
		parentBranch.m_childHandles.emplace_back(targetHandle);
	}



	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::RecycleFlowSingle(SkeletonFlowHandle handle, const std::function<void(SkeletonFlowHandle)>& flowHandler) {
		assert(!m_flows[handle].m_recycled);
		auto& flow = m_flows[handle];
		flowHandler(handle);
		flow.m_parentHandle = -1;
		flow.m_childHandles.clear();
		flow.m_nodes.clear();

		flow.m_data = {};
		flow.m_info = {};

		flow.m_recycled = true;
		flow.m_apical = false;
		m_flowPool.emplace(handle);
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::RecycleNodeSingle(SkeletonNodeHandle handle, const std::function<void(SkeletonNodeHandle)>& nodeHandler) {
		assert(!m_nodes[handle].m_recycled);
		auto& node = m_nodes[handle];
		nodeHandler(handle);
		node.m_parentHandle = -1;
		node.m_flowHandle = -1;
		node.m_endNode = true;
		node.m_childHandles.clear();

		node.m_data = {};
		node.m_info = {};
		node.m_info.m_locked = false;
		node.m_info.m_wounds.clear();

		node.m_recycled = true;
		m_nodePool.emplace(handle);
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	SkeletonFlowHandle Skeleton<SkeletonData, FlowData, NodeData>::AllocateFlow() {
		m_maxFlowIndex++;
		if (m_flowPool.empty()) {
			m_flows.emplace_back(m_flows.size());
			m_flows.back().m_index = m_maxFlowIndex;
			return m_flows.back().m_handle;
		}
		auto handle = m_flowPool.front();
		m_flowPool.pop();
		auto& flow = m_flows[handle];
		flow.m_recycled = false;
		flow.m_index = m_maxFlowIndex;
		return handle;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	SkeletonNodeHandle Skeleton<SkeletonData, FlowData, NodeData>::AllocateNode() {
		m_maxNodeIndex++;
		if (m_nodePool.empty()) {
			m_nodes.emplace_back(m_nodes.size());
			m_nodes.back().m_index = m_maxNodeIndex;
			return m_nodes.back().m_handle;
		}
		auto handle = m_nodePool.front();
		m_nodePool.pop();
		auto& node = m_nodes[handle];
		node.m_recycled = false;
		node.m_index = m_maxNodeIndex;
		return handle;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	int Skeleton<SkeletonData, FlowData, NodeData>::GetVersion() const {
		return m_version;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void Skeleton<SkeletonData, FlowData, NodeData>::CalculateFlows() {
		for (const auto& flowHandle : m_sortedFlowList) {
			auto& flow = m_flows[flowHandle];
			auto& firstNode = m_nodes[flow.m_nodes.front()];
			auto& lastNode = m_nodes[flow.m_nodes.back()];
			flow.m_info.m_startThickness = firstNode.m_info.m_thickness;
			flow.m_info.m_globalStartPosition = firstNode.m_info.m_globalPosition;
			flow.m_info.m_globalStartRotation = firstNode.m_info.m_globalRotation;

			flow.m_info.m_endThickness = lastNode.m_info.m_thickness;
			flow.m_info.m_globalEndPosition = lastNode.m_info.m_globalPosition +
				lastNode.m_info.m_length *
				(lastNode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			flow.m_info.m_globalEndRotation = lastNode.m_info.m_globalRotation;

			flow.m_info.m_flowLength = 0.0f;
			for (const auto& nodeHandle : flow.m_nodes)
			{
				flow.m_info.m_flowLength += m_nodes[nodeHandle].m_info.m_length;
			}

		}
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	std::vector<SkeletonFlow<FlowData>>& Skeleton<SkeletonData, FlowData, NodeData>::RefRawFlows() {
		return m_flows;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	std::vector<SkeletonNode<NodeData>>&
		Skeleton<SkeletonData, FlowData, NodeData>::RefRawNodes() {
		return m_nodes;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	const std::vector<SkeletonFlow<FlowData>>& Skeleton<SkeletonData, FlowData, NodeData>::PeekRawFlows() const
	{
		return m_flows;
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	const std::vector<SkeletonNode<NodeData>>& Skeleton<SkeletonData, FlowData, NodeData>::PeekRawNodes() const
	{
		return m_nodes;
	}

#pragma endregion
#pragma endregion
}