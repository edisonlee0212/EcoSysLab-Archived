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
				out << YAML::Key << "R" << YAML::Value << node.m_recycled;
				out << YAML::Key << "F" << YAML::Value << node.m_flowHandle;
				out << YAML::Key << "P" << YAML::Value << node.m_parentHandle;
				out << YAML::Key << "A" << YAML::Value << node.m_apical;
				out << YAML::Key << "I" << YAML::Value << node.m_index;

				out << YAML::Key << "IF" << YAML::Value << YAML::BeginMap;
				{
					out << YAML::Key << "GP" << YAML::Value << node.m_info.m_globalPosition;
					out << YAML::Key << "GR" << YAML::Value << node.m_info.m_globalRotation;
					out << YAML::Key << "L" << YAML::Value << node.m_info.m_length;
					out << YAML::Key << "T" << YAML::Value << node.m_info.m_thickness;
					out << YAML::Key << "C" << YAML::Value << node.m_info.m_color;
				}
				out << YAML::EndMap;
				
				out << YAML::Key << "D" << YAML::Value << YAML::BeginMap;
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
				out << YAML::Key << "R" << YAML::Value << flow.m_recycled;
				out << YAML::Key << "P" << YAML::Value << flow.m_parentHandle;
				out << YAML::Key << "A" << YAML::Value << flow.m_apical;
				out << YAML::Key << "I" << YAML::Value << flow.m_index;
				out << YAML::Key << "N" << YAML::Value << YAML::BeginSeq;
				for(const auto& nodeHandle : flow.m_nodes)
				{
					out << nodeHandle;
				}
				out << YAML::EndSeq;

				out << YAML::Key << "D" << YAML::Value << YAML::BeginMap;
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
		if (in["m_nodes"]) {
			skeleton.m_nodes.clear();
			const auto& inNodes = in["m_nodes"];
			NodeHandle nodeHandle = 0;
			for (const auto& inNode : inNodes) {
				skeleton.m_nodes.emplace_back();
				auto& node = skeleton.m_nodes.back();
				node.m_handle = nodeHandle;
				if (inNode["R"]) node.m_recycled = inNode["R"].as<bool>();
				if (inNode["F"]) node.m_flowHandle = inNode["F"].as<FlowHandle>();
				if (inNode["P"]) node.m_parentHandle = inNode["P"].as<NodeHandle>();
				if (inNode["I"]) node.m_index = inNode["I"].as<int>();
				if (inNode["A"]) node.m_apical = inNode["A"].as<bool>();
				if (inNode["IF"])
				{
					const auto& inNodeInfo = inNode["IF"];
					if (inNodeInfo["GP"]) node.m_info.m_globalPosition = inNodeInfo["GP"].as<glm::vec3>();
					if (inNodeInfo["GR"]) node.m_info.m_globalRotation = inNodeInfo["GR"].as<glm::quat>();
					if (inNodeInfo["L"]) node.m_info.m_length = inNodeInfo["L"].as<float>();
					if (inNodeInfo["T"]) node.m_info.m_thickness = inNodeInfo["T"].as<float>();
					if (inNodeInfo["C"]) node.m_info.m_color = inNodeInfo["C"].as<glm::vec4>();
				}
				if(inNode["D"])
				{
					const auto& inNodeData = inNode["D"];
					nodeFunc(inNodeData, node.m_data);
				}
				nodeHandle++;
			}
		}
		if (in["m_flows"]) {
			skeleton.m_flows.clear();
			const auto& inFlows = in["m_flows"];
			FlowHandle flowHandle = 0;
			for (const auto& inFlow : inFlows) {
				skeleton.m_flows.emplace_back();
				auto& flow = skeleton.m_flows.back();
				flow.m_handle = flowHandle;
				if (inFlow["R"]) flow.m_recycled = inFlow["R"].as<bool>();
				if (inFlow["P"]) flow.m_parentHandle = inFlow["P"].as<FlowHandle>();
				if (inFlow["I"]) flow.m_index = inFlow["I"].as<int>();
				if (inFlow["A"]) flow.m_apical = inFlow["A"].as<bool>();
				if (inFlow["N"])
				{
					const auto& inFlowNodes= inFlow["N"];
					for (const auto& inFlowNode : inFlowNodes)
					{
						flow.m_nodes.emplace_back(inFlowNode.as<NodeHandle>());
					}
				}
				if (inFlow["D"])
				{
					const auto& inFlowData = inFlow["D"];
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
		
	}
}
