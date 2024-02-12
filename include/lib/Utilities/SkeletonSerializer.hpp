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

		out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
		skeletonFunc(out, skeleton.m_data);
		out << YAML::EndMap;

		out << YAML::Key << "m_nodes" << YAML::Value << YAML::BeginSeq;
		for (int nodeIndex = 0; nodeIndex < skeleton.m_nodes.size(); nodeIndex++)
		{
			const auto& node = skeleton.m_nodes[nodeIndex];
			out << YAML::BeginMap;
			{
				out << YAML::Key << "m_recycled" << YAML::Value << node.m_recycled;
				out << YAML::Key << "m_flowHandle" << YAML::Value << node.m_flowHandle;
				out << YAML::Key << "m_parentHandle" << YAML::Value << node.m_parentHandle;
				out << YAML::Key << "m_apical" << YAML::Value << node.m_apical;
				out << YAML::Key << "m_index" << YAML::Value << node.m_index;

				out << YAML::Key << "m_childHandles" << YAML::Value << YAML::BeginSeq;
				for (const auto& nodeHandle : node.m_childHandles)
				{
					out << nodeHandle;
				}
				out << YAML::EndSeq;

				out << YAML::Key << "m_info" << YAML::Value << YAML::BeginMap;
				{
					out << YAML::Key << "m_globalPosition" << YAML::Value << node.m_info.m_globalPosition;
					out << YAML::Key << "m_globalRotation" << YAML::Value << node.m_info.m_globalRotation;
					out << YAML::Key << "m_length" << YAML::Value << node.m_info.m_length;
					out << YAML::Key << "m_thickness" << YAML::Value << node.m_info.m_thickness;
					out << YAML::Key << "m_color" << YAML::Value << node.m_info.m_color;
				}
				out << YAML::EndMap;
				
				out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
				{
					nodeFunc(out, node.m_data);
				}
				out << YAML::EndMap;
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;

		out << YAML::Key << "m_flows" << YAML::Value << YAML::BeginSeq;
		for (int flowIndex = 0; flowIndex < skeleton.m_flows.size(); flowIndex++)
		{
			const auto& flow = skeleton.m_flows[flowIndex];
			out << YAML::BeginMap;
			{
				out << YAML::Key << "m_recycled" << YAML::Value << flow.m_recycled;
				out << YAML::Key << "m_parentHandle" << YAML::Value << flow.m_parentHandle;
				out << YAML::Key << "m_apical" << YAML::Value << flow.m_apical;
				out << YAML::Key << "m_index" << YAML::Value << flow.m_index;
				out << YAML::Key << "m_nodes" << YAML::Value << YAML::BeginSeq;
				for(const auto& nodeHandle : flow.m_nodes)
				{
					out << nodeHandle;
				}
				out << YAML::EndSeq;

				out << YAML::Key << "m_childHandles" << YAML::Value << YAML::BeginSeq;
				for (const auto& flowHandle : flow.m_childHandles)
				{
					out << flowHandle;
				}
				out << YAML::EndSeq;

				out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
				{
					flowFunc(out, flow.m_data);
				}
				out << YAML::EndMap;
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
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

		if(in["m_data"]) skeletonFunc(in["m_data"], skeleton.m_data);
		std::multimap<int, NodeHandle> sortedNodes;
		if (in["m_nodes"]) {
			skeleton.m_nodes.clear();
			const auto& inNodes = in["m_nodes"];
			NodeHandle nodeHandle = 0;
			for (const auto& inNode : inNodes) {
				skeleton.m_nodes.emplace_back();
				auto& node = skeleton.m_nodes.back();
				node.m_handle = nodeHandle;
				if (inNode["m_recycled"]) node.m_recycled = inNode["m_recycled"].as<bool>();
				if (inNode["m_endNode"]) node.m_endNode = inNode["m_endNode"].as<bool>();
				if (inNode["m_flowHandle"]) node.m_flowHandle = inNode["m_flowHandle"].as<FlowHandle>();
				if (inNode["m_parentHandle"]) node.m_parentHandle = inNode["m_parentHandle"].as<NodeHandle>();
				if (inNode["m_index"]) node.m_index = inNode["m_index"].as<int>();
				sortedNodes.insert({ node.m_index, nodeHandle });
				if (inNode["m_apical"]) node.m_apical = inNode["m_apical"].as<bool>();
				if (inNode["m_info"])
				{
					const auto& inNodeInfo = inNode["m_info"];
					if (inNodeInfo["m_globalPosition"]) node.m_info.m_globalPosition = inNodeInfo["m_globalPosition"].as<glm::vec3>();
					if (inNodeInfo["m_globalRotation"]) node.m_info.m_globalRotation = inNodeInfo["m_globalRotation"].as<glm::quat>();
					node.m_info.m_globalDirection = glm::normalize(node.m_info.m_globalRotation * glm::vec3(0, 0, -1));
					if (inNodeInfo["m_length"]) node.m_info.m_length = inNodeInfo["m_length"].as<float>();
					if (inNodeInfo["m_thickness"]) node.m_info.m_thickness = inNodeInfo["m_thickness"].as<float>();
					if (inNodeInfo["m_color"]) node.m_info.m_color = inNodeInfo["m_color"].as<glm::vec4>();
				}
				if (inNode["m_childHandles"])
				{
					const auto& inNodeChildHandles = inNode["m_childHandles"];
					for (const auto& inNodeNode : inNodeChildHandles)
					{
						node.m_childHandles.emplace_back(inNodeNode.as<NodeHandle>());
					}
				}
				if(inNode["m_data"])
				{
					const auto& inNodeData = inNode["m_data"];
					nodeFunc(inNodeData, node.m_data);
				}
				nodeHandle++;
			}
		}
		std::multimap<int, FlowHandle> sortedFlows;
		if (in["m_flows"]) {
			skeleton.m_flows.clear();
			const auto& inFlows = in["m_flows"];
			FlowHandle flowHandle = 0;
			for (const auto& inFlow : inFlows) {
				skeleton.m_flows.emplace_back();
				auto& flow = skeleton.m_flows.back();
				flow.m_handle = flowHandle;
				if (inFlow["m_recycled"]) flow.m_recycled = inFlow["m_recycled"].as<bool>();
				if (inFlow["m_parentHandle"]) flow.m_parentHandle = inFlow["m_parentHandle"].as<FlowHandle>();
				if (inFlow["m_index"]) flow.m_index = inFlow["m_index"].as<int>();
				sortedFlows.insert({ flow.m_index, flowHandle });
				if (inFlow["m_apical"]) flow.m_apical = inFlow["m_apical"].as<bool>();
				if (inFlow["m_nodes"])
				{
					const auto& inFlowNodes= inFlow["m_nodes"];
					for (const auto& inFlowNode : inFlowNodes)
					{
						flow.m_nodes.emplace_back(inFlowNode.as<NodeHandle>());
					}
				}

				if (inFlow["m_childHandles"])
				{
					const auto& inFlowChildHandles = inFlow["m_childHandles"];
					for (const auto& inFlowNode : inFlowChildHandles)
					{
						flow.m_childHandles.emplace_back(inFlowNode.as<FlowHandle>());
					}
				}
				if (inFlow["m_data"])
				{
					const auto& inFlowData = inFlow["m_data"];
					flowFunc(inFlowData, flow.m_data);
				}
				flowHandle++;
			}
		}

		for(const auto& pair : sortedNodes)
		{
			const auto& node = skeleton.m_nodes[pair.second];
			if(node.m_recycled)
			{
				skeleton.m_nodePool.emplace(pair.second);
			}
			
		}
		for (const auto& pair : sortedFlows)
		{
			const auto& flow = skeleton.m_flows[pair.second];
			if (flow.m_recycled)
			{
				skeleton.m_flowPool.emplace(pair.second);
			}
		}
		skeleton.SortLists();
		skeleton.CalculateDistance();
		skeleton.CalculateFlows();
		skeleton.CalculateRegulatedGlobalRotation();
	}
}
