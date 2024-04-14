#pragma once


namespace EvoEngine {
	typedef int NodeGraphNodeHandle;
	typedef int NodeGraphInputPinHandle;
	typedef int NodeGraphOutputPinHandle;
	typedef int NodeGraphLinkHandle;

	template<typename NodeGraphInputPinData>
	class NodeGraphInputPin
	{
		template<typename ID, typename OD, typename ND, typename LD>
		friend class NodeGraph;
		NodeGraphNodeHandle m_nodeHandle = -1;
		bool m_recycled = true;
		NodeGraphInputPinHandle m_handle = -1;

		NodeGraphLinkHandle m_linkHandle = -1;
	public:
		NodeGraphInputPinData m_data{};
		NodeGraphInputPin() = default;
		NodeGraphInputPin(NodeGraphInputPinHandle handle, NodeGraphNodeHandle nodeHandle);
	};

	template<typename NodeGraphOutputPinData>
	class NodeGraphOutputPin
	{
		template<typename ID, typename OD, typename ND, typename LD>
		friend class NodeGraph;
		NodeGraphNodeHandle m_nodeHandle = -1;
		bool m_recycled = true;
		NodeGraphOutputPinHandle m_handle = -1;

		std::vector<NodeGraphLinkHandle> m_linkHandles{};
	public:
		NodeGraphOutputPinData m_data{};
		NodeGraphOutputPin() = default;
		NodeGraphOutputPin(NodeGraphOutputPinHandle handle, NodeGraphNodeHandle nodeHandle);
	};

	template<typename NodeGraphNodeData>
	class NodeGraphNode
	{
		template<typename ID, typename OD, typename ND, typename LD>
		friend class NodeGraph;
		NodeGraphNodeHandle m_handle = -1;
		bool m_recycled = true;
		

		std::vector<NodeGraphInputPinHandle> m_inputPinHandles;
		NodeGraphOutputPinHandle m_outputPinHandle;
	public:
		NodeGraphNodeData m_data{};
		NodeGraphNode() = default;
		NodeGraphNode(NodeGraphNodeHandle handle);
	};

	template<typename NodeGraphLinkData>
	class NodeGraphLink
	{
		NodeGraphOutputPinHandle m_start = -1;
		NodeGraphInputPinHandle m_end = -1;
		NodeGraphLinkHandle m_handle = -1;
		bool m_recycled = true;

		template<typename ID, typename OD, typename ND, typename LD>
		friend class NodeGraph;
	public:
		NodeGraphLinkData m_data{};
		NodeGraphLink() = default;
		NodeGraphLink(NodeGraphLinkHandle handle, NodeGraphOutputPinHandle outputPinHandle, NodeGraphInputPinHandle inputPinHandle);
	};

	template<typename ID, typename OD, typename ND, typename LD>
	class NodeGraph
	{
		std::vector<NodeGraphInputPin<ID>> m_inputPins;
		std::vector<NodeGraphOutputPin<OD>> m_outputPins;
		std::vector<NodeGraphNode<ND>> m_nodes;
		
		std::vector<NodeGraphLink<LD>> m_links;

		std::queue<NodeGraphInputPinHandle> m_inputPinPool;
		std::queue<NodeGraphOutputPinHandle> m_outputPinPool;
		std::queue<NodeGraphNodeHandle> m_nodePool;
		std::queue<NodeGraphLinkHandle> m_linkPool;

		NodeGraphInputPinHandle AllocateInputPin(NodeGraphNodeHandle nodeHandle);
		NodeGraphOutputPinHandle AllocateOutputPin(NodeGraphNodeHandle nodeHandle);
		
		void RecycleOutputPin(NodeGraphOutputPinHandle handle);
		void RecycleInputPin(NodeGraphInputPinHandle handle);
	public:
		
		NodeGraphLinkHandle AllocateLink(NodeGraphOutputPinHandle startHandle, NodeGraphInputPinHandle endHandle);
		void RecycleLink(NodeGraphLinkHandle handle);


		NodeGraphNodeHandle AllocateNode(size_t inputPinCount, bool hasOutput);
		void RecycleNode(NodeGraphNodeHandle handle);

		bool OnInspect(
			const std::string& title,
			const std::shared_ptr<EditorLayer>& editorLayer,
			const std::function<void(ImVec2 clickPos)>& nodeEditorPopupGui,
			const std::function<void(NodeGraphInputPinHandle inputPinHandle)>& nodeInputPinGui,
			const std::function<void(NodeGraphOutputPinHandle outputPinHandle)>& nodeOutputPinGui,
			const std::function<void(NodeGraphOutputPinHandle startHandle, NodeGraphInputPinHandle endHandle)>& linkCreateHandler,
			const std::function<void(NodeGraphLinkHandle linkHandle)>& linkDestroyHandler,
			const std::function<void(NodeGraphNodeHandle nodeHandle, NodeGraphLinkHandle linkHandle, NodeGraphInputPinHandle inputPinHandle, NodeGraphOutputPinHandle outputPinHandle)>& hoverHandler,
			const std::function<void(const std::vector<NodeGraphNodeHandle>& selectedNodeHandles, const std::vector<NodeGraphLinkHandle>& selectedLinkHandles)>& selectionHandler
		);
	};

	template <typename NodeGraphInputPinData>
	NodeGraphInputPin<NodeGraphInputPinData>::NodeGraphInputPin(const NodeGraphInputPinHandle handle, const NodeGraphNodeHandle nodeHandle)
	{
		m_nodeHandle = nodeHandle;
		m_linkHandle = -1;
		m_handle = handle;
		m_recycled = false;
		m_data = {};
	}

	template <typename NodeGraphOutputPinData>
	NodeGraphOutputPin<NodeGraphOutputPinData>::NodeGraphOutputPin(const NodeGraphOutputPinHandle handle, const NodeGraphNodeHandle nodeHandle)
	{
		m_nodeHandle = nodeHandle;
		m_linkHandles.clear();
		m_handle = handle;
		m_recycled = false;
		m_data = {};
	}

	template <typename NodeGraphNodeData>
	NodeGraphNode<NodeGraphNodeData>::NodeGraphNode(const NodeGraphNodeHandle handle)
	{
		m_inputPinHandles.clear();
		m_outputPinHandle = -1;
		m_handle = handle;
		m_recycled = false;
		m_data = {};
	}

	template <typename NodeGraphLinkData>
	NodeGraphLink<NodeGraphLinkData>::NodeGraphLink(const NodeGraphLinkHandle handle,
		const NodeGraphOutputPinHandle outputPinHandle, const NodeGraphInputPinHandle inputPinHandle)
	{
		m_handle = handle;
		m_recycled = false;
		m_data = {};

		m_start = outputPinHandle;
		m_end = inputPinHandle;
	}

	template <typename ID, typename OD, typename ND, typename LD>
	NodeGraphInputPinHandle NodeGraph<ID, OD, ND, LD>::AllocateInputPin(const NodeGraphNodeHandle nodeHandle)
	{
		if (m_inputPinPool.empty()) {
			m_inputPins.emplace_back(m_inputPins.size(), nodeHandle);
			return m_inputPins.back().m_handle;
		}
		auto handle = m_inputPinPool.front();
		m_inputPinPool.pop();
		auto& inputPin = m_inputPins[handle];
		inputPin.m_linkHandle = -1;
		inputPin.m_nodeHandle = nodeHandle;
		inputPin.m_recycled = false;
		return handle;
	}

	template <typename ID, typename OD, typename ND, typename LD>
	NodeGraphOutputPinHandle NodeGraph<ID, OD, ND, LD>::AllocateOutputPin(const NodeGraphNodeHandle nodeHandle)
	{
		if (m_outputPinPool.empty()) {
			m_outputPins.emplace_back(m_outputPins.size(), nodeHandle);
			return m_outputPins.back().m_handle;
		}
		auto handle = m_outputPinPool.front();
		m_outputPinPool.pop();
		auto& outputPin = m_outputPins[handle];
		outputPin.m_linkHandles.clear();
		outputPin.m_nodeHandle = nodeHandle;
		outputPin.m_recycled = false;
		return handle;
	}

	template <typename ID, typename OD, typename ND, typename LD>
	NodeGraphLinkHandle NodeGraph<ID, OD, ND, LD>::AllocateLink(const NodeGraphOutputPinHandle startHandle,
		const NodeGraphInputPinHandle endHandle)
	{
		if (m_linkPool.empty()) {
			m_links.emplace_back(m_links.size(), startHandle, endHandle);
			return m_links.back().m_handle;
		}
		auto handle = m_linkPool.front();
		m_linkPool.pop();
		auto& link = m_links[handle];
		link.m_start = startHandle;
		link.m_end = endHandle;
		link.m_recycled = false;
		return handle;
	}

	template <typename ID, typename OD, typename ND, typename LD>
	void NodeGraph<ID, OD, ND, LD>::RecycleLink(const NodeGraphLinkHandle handle)
	{
		assert(!m_links[handle].m_recycled);
		auto& link = m_links[handle];
		link.m_data = {};
		auto& outputPinLinkHandles = m_outputPins[link.m_start].m_linkHandles;
		for(int i = 0; i < outputPinLinkHandles.size(); i++)
		{
			if(outputPinLinkHandles[i] == handle)
			{
				outputPinLinkHandles[i] = outputPinLinkHandles.back();
				outputPinLinkHandles.pop_back();
				break;
			}
		}
		m_inputPins[link.m_end].m_linkHandle = -1;
		link.m_recycled = true;
		m_linkPool.emplace(handle);
	}

	template <typename ID, typename OD, typename ND, typename LD>
	void NodeGraph<ID, OD, ND, LD>::RecycleOutputPin(NodeGraphOutputPinHandle handle)
	{
		assert(!m_outputPins[handle].m_recycled);
		auto& outputPin = m_outputPins[handle];
		outputPin.m_data = {};

		auto linkHandles = outputPin.m_linkHandles;
		for(const auto& i : linkHandles) RecycleLink(i);
		assert(outputPin.m_linkHandles.empty());
		outputPin.m_recycled = true;
		m_outputPinPool.emplace(handle);
	}

	template <typename ID, typename OD, typename ND, typename LD>
	void NodeGraph<ID, OD, ND, LD>::RecycleInputPin(NodeGraphInputPinHandle handle)
	{
		assert(!m_inputPins[handle].m_recycled);
		auto& inputPin = m_inputPins[handle];
		inputPin.m_data = {};
		if (inputPin.m_linkHandle != -1)
		{
			RecycleLink(inputPin.m_linkHandle);
			inputPin.m_linkHandle = -1;
		}
		inputPin.m_recycled = true;
		m_inputPinPool.emplace(handle);
	}

	template <typename ID, typename OD, typename ND, typename LD>
	NodeGraphNodeHandle NodeGraph<ID, OD, ND, LD>::AllocateNode(const size_t inputPinCount, bool hasOutput)
	{
		NodeGraphNodeHandle newNodeHandle;
		if (m_nodePool.empty()) {
			m_nodes.emplace_back(m_nodes.size());
			newNodeHandle = m_nodes.back().m_handle;
		}else
		{
			newNodeHandle = m_nodePool.front();
			m_nodePool.pop();
		}
		auto& node = m_nodes[newNodeHandle];
		node.m_data = {};
		node.m_inputPinHandles.clear();
		node.m_outputPinHandle = -1;
		for(int i = 0; i < inputPinCount; i++)
		{
			node.m_inputPinHandles.emplace_back(AllocateInputPin(newNodeHandle));
		}
		if(hasOutput) node.m_outputPinHandle = AllocateOutputPin(newNodeHandle);
		node.m_recycled = false;
		return newNodeHandle;
	}

	template <typename ID, typename OD, typename ND, typename LD>
	void NodeGraph<ID, OD, ND, LD>::RecycleNode(const NodeGraphNodeHandle handle)
	{
		assert(!m_nodes[handle].m_recycled);
		auto& node = m_nodes[handle];
		node.m_data = {};
		for(const auto& i : node.m_inputPinHandles)
		{
			RecycleInputPin(i);
		}
		if(node.m_outputPinHandle != -1) RecycleOutputPin(node.m_outputPinHandle);
		node.m_recycled = true;
		m_nodePool.emplace(handle);
	}

	template <typename ID, typename OD, typename ND, typename LD>
	bool NodeGraph<ID, OD, ND, LD>::OnInspect(
		const std::string& title,
		const std::shared_ptr<EditorLayer>& editorLayer,
		const std::function<void(ImVec2 clickPos)>& nodeEditorPopupGui,
		const std::function<void(NodeGraphInputPinHandle inputPinHandle)>& nodeInputPinGui,
		const std::function<void(NodeGraphOutputPinHandle outputPinHandle)>& nodeOutputPinGui,
		const std::function<void(NodeGraphOutputPinHandle startHandle, NodeGraphInputPinHandle endHandle)>& linkCreateHandler,
		const std::function<void(NodeGraphLinkHandle linkHandle)>& linkDestroyHandler,
		const std::function<void(NodeGraphNodeHandle nodeHandle, NodeGraphLinkHandle linkHandle, NodeGraphInputPinHandle inputPinHandle, NodeGraphOutputPinHandle outputPinHandle)>& hoverHandler,
		const std::function<void(const std::vector<NodeGraphNodeHandle>& selectedNodeHandles, const std::vector<NodeGraphLinkHandle>& selectedLinkHandles)>& selectionHandler
		)
	{
		ImNodesIO& io = ImNodes::GetIO();
		io.LinkDetachWithModifierClick.Modifier = &ImGui::GetIO().KeyAlt;
		io.MultipleSelectModifier.Modifier = &ImGui::GetIO().KeyCtrl;

		ImGui::Begin(title.c_str());

		ImNodes::BeginNodeEditor();

		// Handle new nodes
		// These are driven by the user, so we place this code before rendering the nodes
		
			const bool openPopup = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
				ImNodes::IsEditorHovered() && editorLayer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == KeyActionType::Press;

			if (!ImGui::IsAnyItemHovered() && openPopup)
			{
				ImGui::OpenPopup((title + "_editor_menu").c_str());
			}
			ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.f, 8.f));
			if (ImGui::BeginPopup((title + "_editor_menu").c_str()))
			{
				const ImVec2 clickPos = ImGui::GetMousePosOnOpeningCurrentPopup();
				nodeEditorPopupGui(clickPos);
				ImGui::EndPopup();
			}
			ImGui::PopStyleVar();
		


		for (const auto& node : m_nodes)
		{
			if (node.m_recycled) continue;


			ImNodes::BeginNode(node.m_handle);
			ImNodes::BeginNodeTitleBar();
			ImGui::TextUnformatted("Node");
			ImNodes::EndNodeTitleBar();
			
			for(const auto inputPinHandle : node.m_inputPinHandles)
			{
				ImNodes::BeginInputAttribute(inputPinHandle + (1 << 16), ImNodesPinShape_QuadFilled);
				// in between Begin|EndAttribute calls, you can call ImGui
				// UI functions
				nodeInputPinGui(inputPinHandle);
				ImNodes::EndInputAttribute();
			}


			if(node.m_outputPinHandle != -1){
				ImNodes::BeginOutputAttribute(node.m_outputPinHandle + (1 << 17));
				// in between Begin|EndAttribute calls, you can call ImGui
				// UI functions
				nodeOutputPinGui(node.m_outputPinHandle);
				ImNodes::EndOutputAttribute();
			}
			
			ImNodes::EndNode();
		}

		for(const auto& link : m_links)
		{
			if (link.m_recycled) continue;
			ImNodes::Link(link.m_handle, link.m_start + (1 << 17), link.m_end + (1 << 16));
		}

		ImNodes::EndNodeEditor();
		{
			NodeGraphOutputPinHandle outputHandle;
			NodeGraphInputPinHandle inputHandle;
			if (ImNodes::IsLinkCreated(&outputHandle, &inputHandle))
			{
				linkCreateHandler(outputHandle - (1 << 17), inputHandle - (1 << 16));
			}
		}
		{
			NodeGraphLinkHandle linkHandle;
			if (ImNodes::IsLinkDestroyed(&linkHandle))
			{
				linkDestroyHandler(linkHandle);
			}
		}
		NodeGraphNodeHandle hoveredNodeHandle = -1;
		NodeGraphLinkHandle hoveredLinkHandle = -1;
		NodeGraphInputPinHandle hoveredInputPinHandle = -1;
		NodeGraphOutputPinHandle hoveredOutputPinHandle = -1;
		std::vector<NodeGraphNodeHandle> selectedNodes;
		std::vector<NodeGraphLinkHandle> selectedLinks;
		int id = -1;
		if (ImNodes::IsNodeHovered(&id))
		{
			hoveredNodeHandle = id;
		}
		if (ImNodes::IsLinkHovered(&id))
		{
			hoveredLinkHandle = id;
		}
		if (ImNodes::IsPinHovered(&id))
		{
			if (id < (1 << 17)) hoveredInputPinHandle = id - (1 << 16);
			else hoveredOutputPinHandle = id - (1 << 17);
		}
		hoverHandler(hoveredNodeHandle, hoveredLinkHandle, hoveredInputPinHandle, hoveredOutputPinHandle);
		const int numSelectedNodes = ImNodes::NumSelectedNodes();
		if (numSelectedNodes > 0)
		{
			selectedNodes.resize(numSelectedNodes);
			ImNodes::GetSelectedNodes(selectedNodes.data());
		}
		const int numSelectedLinks = ImNodes::NumSelectedLinks();
		if (numSelectedLinks > 0)
		{
			selectedLinks.resize(numSelectedLinks);
			ImNodes::GetSelectedLinks(selectedLinks.data());
		}
		selectionHandler(selectedNodes, selectedLinks);
		ImGui::End();
		return false;
	}
}
