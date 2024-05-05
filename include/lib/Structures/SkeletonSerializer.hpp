#pragma once

#include "Skeleton.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	template<typename SkeletonData, typename FlowData, typename NodeData>
	class SkeletonSerializer {
	public:
		static void Serialize(YAML::Emitter& out, const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			const std::function<void(YAML::Emitter& nodeOut, const NodeData& nodeData)>& nodeFunc,
			const std::function<void(YAML::Emitter& flowOut, const FlowData& flowData)>& flowFunc,
			const std::function<void(YAML::Emitter& skeletonOut, const SkeletonData& skeletonData)>& skeletonFunc);

		static void Deserialize(const YAML::Node& in, Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			const std::function<void(const YAML::Node& nodeIn, NodeData& nodeData)>& nodeFunc,
			const std::function<void(const YAML::Node& flowIn, FlowData& flowData)>& flowFunc,
			const std::function<void(const YAML::Node& skeletonIn, SkeletonData& skeletonData)>& skeletonFunc);
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void SkeletonSerializer<SkeletonData, FlowData, NodeData>::Serialize(YAML::Emitter& out,
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		const std::function<void(YAML::Emitter& nodeOut, const NodeData& nodeData)>& nodeFunc,
		const std::function<void(YAML::Emitter& flowOut, const FlowData& flowData)>& flowFunc,
		const std::function<void(YAML::Emitter& skeletonOut, const SkeletonData& skeletonData)>& skeletonFunc)
	{
		out << YAML::Key << "m_maxNodeIndex" << YAML::Value << skeleton.m_maxNodeIndex;
		out << YAML::Key << "m_maxFlowIndex" << YAML::Value << skeleton.m_maxFlowIndex;
		out << YAML::Key << "m_newVersion" << YAML::Value << skeleton.m_newVersion;
		out << YAML::Key << "m_min" << YAML::Value << skeleton.m_min;
		out << YAML::Key << "m_max" << YAML::Value << skeleton.m_max;

		
		const auto nodeSize = skeleton.m_nodes.size();
		auto nodeRecycledList = std::vector<int>(nodeSize);
		auto nodeFlowHandleList = std::vector<SkeletonFlowHandle>(nodeSize);
		auto nodeParentHandleList = std::vector<SkeletonNodeHandle>(nodeSize);
		auto nodeApicalList = std::vector<int>(nodeSize);
		auto nodeIndexList = std::vector<int>(nodeSize);

		auto infoGlobalPositionList = std::vector<glm::vec3>(nodeSize);
		auto infoGlobalRotationList = std::vector<glm::quat>(nodeSize);
		auto infoLengthList = std::vector<float>(nodeSize);
		auto infoThicknessList = std::vector<float>(nodeSize);
		auto infoColorList = std::vector<glm::vec4>(nodeSize);
		auto infoLockedList = std::vector<int>(nodeSize);

		auto infoLeavesList = std::vector<int>(nodeSize);
		auto infoFruitsList = std::vector<int>(nodeSize);
		for (int nodeIndex = 0; nodeIndex < nodeSize; nodeIndex++)
		{
			const auto& node = skeleton.m_nodes[nodeIndex];
			nodeRecycledList[nodeIndex] = node.m_recycled ? 1 : 0;
			nodeFlowHandleList[nodeIndex] = node.m_flowHandle;
			nodeParentHandleList[nodeIndex] = node.m_parentHandle;
			nodeApicalList[nodeIndex] = node.m_apical ? 1 : 0;
			nodeIndexList[nodeIndex] = node.m_index;

			infoGlobalPositionList[nodeIndex] = node.m_info.m_globalPosition;
			infoGlobalRotationList[nodeIndex] = node.m_info.m_globalRotation;
			infoLengthList[nodeIndex] = node.m_info.m_length;
			infoThicknessList[nodeIndex] = node.m_info.m_thickness;
			infoColorList[nodeIndex] = node.m_info.m_color;
			infoLockedList[nodeIndex] = node.m_info.m_locked ? 1 : 0;

			infoLeavesList[nodeIndex] = node.m_info.m_leaves;
			infoFruitsList[nodeIndex] = node.m_info.m_fruits;
		}
		out << YAML::Key << "m_nodes.m_recycled" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(nodeRecycledList.data()), nodeRecycledList.size() * sizeof(int));
		out << YAML::Key << "m_nodes.m_flowHandle" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(nodeFlowHandleList.data()), nodeFlowHandleList.size() * sizeof(SkeletonFlowHandle));
		out << YAML::Key << "m_nodes.m_parentHandle" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(nodeParentHandleList.data()), nodeParentHandleList.size() * sizeof(SkeletonNodeHandle));
		out << YAML::Key << "m_nodes.m_apical" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(nodeApicalList.data()), nodeApicalList.size() * sizeof(int));
		out << YAML::Key << "m_nodes.m_index" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(nodeIndexList.data()), nodeIndexList.size() * sizeof(int));

		out << YAML::Key << "m_nodes.m_info.m_globalPosition" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoGlobalPositionList.data()), infoGlobalPositionList.size() * sizeof(glm::vec3));
		out << YAML::Key << "m_nodes.m_info.m_globalRotation" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoGlobalRotationList.data()), infoGlobalRotationList.size() * sizeof(glm::quat));
		out << YAML::Key << "m_nodes.m_info.m_length" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoLengthList.data()), infoLengthList.size() * sizeof(float));
		out << YAML::Key << "m_nodes.m_info.m_thickness" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoThicknessList.data()), infoThicknessList.size() * sizeof(float));
		out << YAML::Key << "m_nodes.m_info.m_color" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoColorList.data()), infoColorList.size() * sizeof(glm::vec4));
		out << YAML::Key << "m_nodes.m_info.m_locked" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoLockedList.data()), infoLockedList.size() * sizeof(int));

		out << YAML::Key << "m_nodes.m_info.m_leaves" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoLeavesList.data()), infoLeavesList.size() * sizeof(float));
		out << YAML::Key << "m_nodes.m_info.m_fruits" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(infoFruitsList.data()), infoFruitsList.size() * sizeof(float));

		out << YAML::Key << "m_nodes.m_info" << YAML::Value << YAML::BeginSeq;
		for (size_t nodeIndex = 0; nodeIndex < nodeSize; nodeIndex++)
		{
			const auto& node = skeleton.m_nodes[nodeIndex];
			out << YAML::BeginMap;
			{
				if (!node.m_info.m_wounds.empty()) {
					out << YAML::Key << "m_wounds" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(node.m_info.m_wounds.data()), node.m_info.m_wounds.size() * sizeof(SkeletonNodeWound));
				}
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;

		out << YAML::Key << "m_nodes.m_data" << YAML::Value << YAML::BeginSeq;
		for (size_t nodeIndex = 0; nodeIndex < nodeSize; nodeIndex++)
		{
			const auto& node = skeleton.m_nodes[nodeIndex];
			out << YAML::BeginMap;
			{
				nodeFunc(out, node.m_data);
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;


		const auto flowSize = skeleton.m_flows.size();
		auto flowRecycledList = std::vector<int>(flowSize);
		auto flowParentHandleList = std::vector<SkeletonFlowHandle>(flowSize);
		auto flowApicalList = std::vector<int>(flowSize);
		auto flowIndexList = std::vector<int>(flowSize);
		for (int flowIndex = 0; flowIndex < flowSize; flowIndex++)
		{
			const auto& flow = skeleton.m_flows[flowIndex];
			flowRecycledList[flowIndex] = flow.m_recycled ? 1 : 0;
			flowParentHandleList[flowIndex] = flow.m_parentHandle;
			flowApicalList[flowIndex] = flow.m_apical ? 1 : 0;
			flowIndexList[flowIndex] = flow.m_index;
		}
		out << YAML::Key << "m_flows.m_recycled" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(flowRecycledList.data()), flowRecycledList.size() * sizeof(int));
		out << YAML::Key << "m_flows.m_parentHandle" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(flowParentHandleList.data()), flowParentHandleList.size() * sizeof(SkeletonFlowHandle));
		out << YAML::Key << "m_flows.m_apical" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(flowApicalList.data()), flowApicalList.size() * sizeof(int));
		out << YAML::Key << "m_flows.m_index" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(flowIndexList.data()), flowIndexList.size() * sizeof(int));

		out << YAML::Key << "m_flows" << YAML::Value << YAML::BeginSeq;
		for (int flowIndex = 0; flowIndex < flowSize; flowIndex++)
		{
			const auto& flow = skeleton.m_flows[flowIndex];
			out << YAML::BeginMap;
			{
				if (!flow.m_nodes.empty()) {
					out << YAML::Key << "m_nodes" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(flow.m_nodes.data()), flow.m_nodes.size() * sizeof(SkeletonNodeHandle));
				}
				out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
				{
					flowFunc(out, flow.m_data);
				}
				out << YAML::EndMap;
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;

		out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
		skeletonFunc(out, skeleton.m_data);
		out << YAML::EndMap;

	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void SkeletonSerializer<SkeletonData, FlowData, NodeData>::Deserialize(
		const YAML::Node& in, Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		const std::function<void(const YAML::Node& nodeIn, NodeData& nodeData)>& nodeFunc,
		const std::function<void(const YAML::Node& flowIn, FlowData& flowData)>& flowFunc,
		const std::function<void(const YAML::Node& skeletonIn, SkeletonData& skeletonData)>& skeletonFunc)
	{
		if (in["m_maxNodeIndex"]) skeleton.m_maxNodeIndex = in["m_maxNodeIndex"].as<int>();
		if (in["m_maxFlowIndex"]) skeleton.m_maxFlowIndex = in["m_maxFlowIndex"].as<int>();
		if (in["m_newVersion"]) skeleton.m_newVersion = in["m_newVersion"].as<int>();
		skeleton.m_version = -1;
		if (in["m_min"]) skeleton.m_min = in["m_min"].as<glm::vec3>();
		if (in["m_max"]) skeleton.m_max = in["m_max"].as<glm::vec3>();
		
		if(in["m_nodes.m_recycled"])
		{
			auto nodeRecycledList = std::vector<int>();
			const auto data = in["m_nodes.m_recycled"].as<YAML::Binary>();
			nodeRecycledList.resize(data.size() / sizeof(int));
			std::memcpy(nodeRecycledList.data(), data.data(), data.size());

			skeleton.m_nodes.resize(nodeRecycledList.size());
			for(size_t i = 0; i < nodeRecycledList.size(); i++)
			{
				skeleton.m_nodes[i].m_recycled = nodeRecycledList[i] == 1;
			}
		}

		if (in["m_nodes.m_flowHandle"])
		{
			auto nodeFlowHandleList = std::vector<SkeletonFlowHandle>();
			const auto data = in["m_nodes.m_flowHandle"].as<YAML::Binary>();
			nodeFlowHandleList.resize(data.size() / sizeof(SkeletonFlowHandle));
			std::memcpy(nodeFlowHandleList.data(), data.data(), data.size());

			for (size_t i = 0; i < nodeFlowHandleList.size(); i++)
			{
				skeleton.m_nodes[i].m_flowHandle = nodeFlowHandleList[i];
			}
		}

		if (in["m_nodes.m_parentHandle"])
		{
			auto nodeParentHandleList = std::vector<SkeletonNodeHandle>();
			const auto data = in["m_nodes.m_parentHandle"].as<YAML::Binary>();
			nodeParentHandleList.resize(data.size() / sizeof(SkeletonFlowHandle));
			std::memcpy(nodeParentHandleList.data(), data.data(), data.size());

			for (size_t i = 0; i < nodeParentHandleList.size(); i++)
			{
				skeleton.m_nodes[i].m_parentHandle = nodeParentHandleList[i];
			}
		}

		if (in["m_nodes.m_apical"])
		{
			auto nodeApicalList = std::vector<int>();
			const auto data = in["m_nodes.m_apical"].as<YAML::Binary>();
			nodeApicalList.resize(data.size() / sizeof(int));
			std::memcpy(nodeApicalList.data(), data.data(), data.size());

			for (size_t i = 0; i < nodeApicalList.size(); i++)
			{
				skeleton.m_nodes[i].m_apical = nodeApicalList[i] == 1;
			}
		}

		if (in["m_nodes.m_index"])
		{
			auto nodeIndexList = std::vector<int>();
			const auto data = in["m_nodes.m_index"].as<YAML::Binary>();
			nodeIndexList.resize(data.size() / sizeof(int));
			std::memcpy(nodeIndexList.data(), data.data(), data.size());

			for (size_t i = 0; i < nodeIndexList.size(); i++)
			{
				skeleton.m_nodes[i].m_index = nodeIndexList[i];
			}
		}

		if (in["m_nodes.m_info.m_globalPosition"])
		{
			auto infoGlobalPositionList = std::vector<glm::vec3>();
			const auto data = in["m_nodes.m_info.m_globalPosition"].as<YAML::Binary>();
			infoGlobalPositionList.resize(data.size() / sizeof(glm::vec3));
			std::memcpy(infoGlobalPositionList.data(), data.data(), data.size());

			for (size_t i = 0; i < infoGlobalPositionList.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_globalPosition = infoGlobalPositionList[i];
			}
		}

		if (in["m_nodes.m_info.m_globalRotation"])
		{
			auto infoGlobalRotationList = std::vector<glm::quat>();
			const auto data = in["m_nodes.m_info.m_globalRotation"].as<YAML::Binary>();
			infoGlobalRotationList.resize(data.size() / sizeof(glm::quat));
			std::memcpy(infoGlobalRotationList.data(), data.data(), data.size());

			for (size_t i = 0; i < infoGlobalRotationList.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_globalRotation = infoGlobalRotationList[i];
			}
		}

		if (in["m_nodes.m_info.m_length"])
		{
			auto infoLengthList = std::vector<float>();
			const auto data = in["m_nodes.m_info.m_length"].as<YAML::Binary>();
			infoLengthList.resize(data.size() / sizeof(float));
			std::memcpy(infoLengthList.data(), data.data(), data.size());

			for (size_t i = 0; i < infoLengthList.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_length = infoLengthList[i];
			}
		}

		if (in["m_nodes.m_info.m_thickness"])
		{
			auto list = std::vector<float>();
			const auto data = in["m_nodes.m_info.m_thickness"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(float));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_thickness = list[i];
			}
		}

		if (in["m_nodes.m_info.m_color"])
		{
			auto list = std::vector<glm::vec4>();
			const auto data = in["m_nodes.m_info.m_color"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec4));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_color = list[i];
			}
		}

		if (in["m_nodes.m_info.m_locked"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_nodes.m_info.m_locked"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_locked = list[i] == 1;
			}
		}

		if (in["m_nodes.m_info.m_leaves"])
		{
			auto list = std::vector<float>();
			const auto data = in["m_nodes.m_info.m_leaves"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(float));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_leaves = list[i];
			}
		}

		if (in["m_nodes.m_info.m_fruits"])
		{
			auto list = std::vector<float>();
			const auto data = in["m_nodes.m_info.m_fruits"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(float));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				skeleton.m_nodes[i].m_info.m_fruits = list[i];
			}
		}

		if(in["m_nodes.m_info"])
		{
			const auto& inNodes = in["m_nodes.m_info"];
			SkeletonNodeHandle nodeHandle = 0;
			for (const auto& inNodeInfo : inNodes) {
				if(inNodeInfo["m_wounds"])
				{
					auto& node = skeleton.m_nodes[nodeHandle];
					const auto data = inNodeInfo["m_wounds"].as<YAML::Binary>();
					node.m_info.m_wounds.resize(data.size() / sizeof(SkeletonNodeWound));
					std::memcpy(node.m_info.m_wounds.data(), data.data(), data.size());
				}
			}
		}

		if (in["m_nodes.m_data"]) {
			const auto& inNodes = in["m_nodes.m_data"];
			SkeletonNodeHandle nodeHandle = 0;
			for (const auto& inNodeData : inNodes) {
				auto& node = skeleton.m_nodes[nodeHandle];
				node.m_handle = nodeHandle;
				nodeFunc(inNodeData, node.m_data);
				nodeHandle++;
			}
		}


		if (in["m_flows.m_recycled"])
		{
			auto flowRecycledList = std::vector<int>();
			const auto data = in["m_flows.m_recycled"].as<YAML::Binary>();
			flowRecycledList.resize(data.size() / sizeof(int));
			std::memcpy(flowRecycledList.data(), data.data(), data.size());

			skeleton.m_flows.resize(flowRecycledList.size());
			for (size_t i = 0; i < flowRecycledList.size(); i++)
			{
				skeleton.m_flows[i].m_recycled = flowRecycledList[i] == 1;
			}
		}

		if (in["m_flows.m_parentHandle"])
		{
			auto flowParentHandleList = std::vector<SkeletonFlowHandle>();
			const auto data = in["m_flows.m_parentHandle"].as<YAML::Binary>();
			flowParentHandleList.resize(data.size() / sizeof(SkeletonFlowHandle));
			std::memcpy(flowParentHandleList.data(), data.data(), data.size());

			for (size_t i = 0; i < flowParentHandleList.size(); i++)
			{
				skeleton.m_flows[i].m_parentHandle = flowParentHandleList[i];
			}
		}

		if (in["m_flows.m_apical"])
		{
			auto flowApicalList = std::vector<int>();
			const auto data = in["m_flows.m_apical"].as<YAML::Binary>();
			flowApicalList.resize(data.size() / sizeof(int));
			std::memcpy(flowApicalList.data(), data.data(), data.size());

			for (size_t i = 0; i < flowApicalList.size(); i++)
			{
				skeleton.m_flows[i].m_apical = flowApicalList[i] == 1;
			}
		}

		if (in["m_flows.m_index"])
		{
			auto flowIndexList = std::vector<int>();
			const auto data = in["m_flows.m_index"].as<YAML::Binary>();
			flowIndexList.resize(data.size() / sizeof(int));
			std::memcpy(flowIndexList.data(), data.data(), data.size());

			for (size_t i = 0; i < flowIndexList.size(); i++)
			{
				skeleton.m_flows[i].m_index = flowIndexList[i];
			}
		}

		if (in["m_flows"]) {
			const auto& inFlows = in["m_flows"];
			SkeletonFlowHandle flowHandle = 0;
			for (const auto& inFlow : inFlows) {
				auto& flow = skeleton.m_flows[flowHandle];
				flow.m_handle = flowHandle;
				if (inFlow["m_nodes"])
				{
					const auto nodes = inFlow["m_nodes"].as<YAML::Binary>();
					flow.m_nodes.resize(nodes.size() / sizeof(SkeletonNodeHandle));
					std::memcpy(flow.m_nodes.data(), nodes.data(), nodes.size());
				}
				if (inFlow["m_data"])
				{
					const auto& inFlowData = inFlow["m_data"];
					flowFunc(inFlowData, flow.m_data);
				}
				flowHandle++;
			}
		}
		skeleton.m_nodePool = {};
		skeleton.m_flowPool = {};
		for(const auto& node : skeleton.m_nodes)
		{
			if(node.m_recycled)
			{
				skeleton.m_nodePool.emplace(node.m_handle);
			}else if(node.m_parentHandle != -1)
			{
				skeleton.m_nodes[node.m_parentHandle].m_childHandles.emplace_back(node.m_handle);
			}
		}
		for (auto& node : skeleton.m_nodes)
		{
			node.m_endNode = node.m_childHandles.empty();
		}
		for (const auto& flow : skeleton.m_flows)
		{
			if (flow.m_recycled)
			{
				skeleton.m_flowPool.emplace(flow.m_handle);
			}
			else if (flow.m_parentHandle != -1)
			{
				skeleton.m_flows[flow.m_parentHandle].m_childHandles.emplace_back(flow.m_handle);
			}
		}
		skeleton.SortLists();

		skeleton.CalculateDistance();
		skeleton.CalculateFlows();
		skeleton.CalculateRegulatedGlobalRotation();

		if (in["m_data"]) skeletonFunc(in["m_data"], skeleton.m_data);
	}
}
