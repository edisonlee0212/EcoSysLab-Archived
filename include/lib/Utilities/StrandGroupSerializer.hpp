#pragma once

#include "StrandGroup.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	template<typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	class StrandGroupSerializer {
	public:
		static void Serialize(YAML::Emitter& out, const StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strandGroup,
			const std::function<void(YAML::Emitter& strandSegmentOut, const StrandSegmentData& strandSegmentData)>& strandSegmentFunc,
			const std::function<void(YAML::Emitter& strandOut, const StrandData& strandData)>& strandFunc,
			const std::function<void(YAML::Emitter& groupOut, const StrandGroupData& groupData)>& groupFunc);

		static void Deserialize(const YAML::Node& in, StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strandGroup,
			const std::function<void(const YAML::Node& strandSegmentIn, StrandSegmentData& segmentData)>& strandSegmentFunc,
			const std::function<void(const YAML::Node& strandIn, StrandData& strandData)>& strandFunc,
			const std::function<void(const YAML::Node& groupIn, StrandGroupData& groupData)>& groupFunc);
	};

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	void StrandGroupSerializer<StrandGroupData, StrandData, StrandSegmentData>::Serialize(YAML::Emitter& out,
		const StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strandGroup,
		const std::function<void(YAML::Emitter& strandSegmentOut, const StrandSegmentData& strandSegmentData)>& strandSegmentFunc,
		const std::function<void(YAML::Emitter& strandOut, const StrandData& strandData)>& strandFunc,
		const std::function<void(YAML::Emitter& groupOut, const StrandGroupData& groupData)>& groupFunc)
	{
		out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
		groupFunc(out, strandGroup.m_data);
		out << YAML::EndMap;

		out << YAML::Key << "m_strands" << YAML::Value << YAML::BeginSeq;
		for (const auto& strand : strandGroup.m_strands)
		{
			out << YAML::BeginMap;
			{
				out << YAML::Key << "R" << YAML::Value << strand.m_recycled;
				out << YAML::Key << "SSH" << YAML::Value << YAML::BeginSeq;
				{
					for (const auto strandSegmentHandle : strand.m_strandSegmentHandles)
					{
						out << strandSegmentHandle;
					}
				}
				out << YAML::EndSeq;
				out << YAML::Key << "IF" << YAML::Value << YAML::BeginMap;
				{
					out << YAML::Key << "C" << YAML::Value << strand.m_info.m_color;
					out << YAML::Key << "BGP" << YAML::Value << strand.m_info.m_baseInfo.m_globalPosition;
					out << YAML::Key << "BGR" << YAML::Value << strand.m_info.m_baseInfo.m_globalRotation;
					out << YAML::Key << "BT" << YAML::Value << strand.m_info.m_baseInfo.m_thickness;
					out << YAML::Key << "BC" << YAML::Value << strand.m_info.m_baseInfo.m_color;
					out << YAML::Key << "IB" << YAML::Value << strand.m_info.m_baseInfo.m_isBoundary;
				}
				out << YAML::EndMap;
				out << YAML::Key << "D" << YAML::Value << YAML::BeginMap;
				{
					strandFunc(out, strand.m_data);
				}
				out << YAML::EndMap;
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;

		out << YAML::Key << "m_strandSegments" << YAML::Value << YAML::BeginSeq;
		for (const auto& strandSegment : strandGroup.m_strandSegments)
		{
			out << YAML::BeginMap;
			{
				out << YAML::Key << "E" << YAML::Value << strandSegment.m_endSegment;
				out << YAML::Key << "R" << YAML::Value << strandSegment.m_recycled;
				out << YAML::Key << "P" << YAML::Value << strandSegment.m_prevHandle;
				out << YAML::Key << "N" << YAML::Value << strandSegment.m_nextHandle;
				out << YAML::Key << "S" << YAML::Value << strandSegment.m_strandHandle;
				out << YAML::Key << "I" << YAML::Value << strandSegment.m_index;
				out << YAML::Key << "IF" << YAML::Value << YAML::BeginMap;
				{
					out << YAML::Key << "GP" << YAML::Value << strandSegment.m_info.m_globalPosition;
					out << YAML::Key << "GR" << YAML::Value << strandSegment.m_info.m_globalRotation;
					out << YAML::Key << "T" << YAML::Value << strandSegment.m_info.m_thickness;
					out << YAML::Key << "C" << YAML::Value << strandSegment.m_info.m_color;
					out << YAML::Key << "B" << YAML::Value << strandSegment.m_info.m_isBoundary;
				}
				out << YAML::EndMap;
				out << YAML::Key << "D" << YAML::Value << YAML::BeginMap;
				{
					strandSegmentFunc(out, strandSegment.m_data);
				}
				out << YAML::EndMap;
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	void StrandGroupSerializer<StrandGroupData, StrandData, StrandSegmentData>::Deserialize(const YAML::Node& in,
		StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strandGroup,
		const std::function<void(const YAML::Node& segmentIn, StrandSegmentData& segmentData)>& strandSegmentFunc,
		const std::function<void(const YAML::Node& strandIn, StrandData& strandData)>& strandFunc,
		const std::function<void(const YAML::Node& groupIn, StrandGroupData& groupData)>& groupFunc)
	{
		if (in["m_data"]) groupFunc(in["m_data"], strandGroup.m_data);

		if(in["m_strands"])
		{
			strandGroup.m_strands.clear();
			const auto& inStrands = in["m_strands"];
			StrandHandle strandHandle = 0;
			for(const auto& inStrand : inStrands)
			{
				strandGroup.m_strands.emplace_back();
				auto& strand = strandGroup.m_strands.back();
				strand.m_handle = strandHandle;
				if (inStrand["R"]) strand.m_recycled = inStrand["R"].as<bool>();
				if (inStrand["SSH"])
				{
					const auto& inStrandSegmentHandles = inStrand["SSH"];
					for(const auto& inStrandSegmentHandle : inStrandSegmentHandles)
					{
						strand.m_strandSegmentHandles.emplace_back(inStrandSegmentHandle.as<StrandSegmentHandle>());
					}
				}
				if (inStrand["IF"])
				{
					const auto& inStrandInfo = inStrand["IF"];
					if (inStrandInfo["C"]) strand.m_info.m_color = inStrandInfo["C"].as<glm::vec4>();
					if (inStrandInfo["BGP"]) strand.m_info.m_baseInfo.m_globalPosition = inStrandInfo["BGP"].as<glm::vec3>();
					if (inStrandInfo["BGR"]) strand.m_info.m_baseInfo.m_globalRotation = inStrandInfo["BGR"].as<glm::quat>();
					if (inStrandInfo["BT"]) strand.m_info.m_baseInfo.m_thickness = inStrandInfo["BT"].as<float>();
					if (inStrandInfo["BC"]) strand.m_info.m_baseInfo.m_color = inStrandInfo["BC"].as<glm::vec4>();
					if (inStrandInfo["IB"]) strand.m_info.m_baseInfo.m_isBoundary = inStrandInfo["IB"].as<bool>();
				}
				if (inStrand["D"])
				{
					const auto& inStrandData = inStrand["D"];
					strandFunc(inStrandData, strand.m_data);
				}
				strandHandle++;
			}
		}
		if (in["m_strandSegments"])
		{
			strandGroup.m_strandSegments.clear();
			const auto& inStrandSegments = in["m_strandSegments"];
			StrandSegmentHandle strandSegmentHandle = 0;
			for (const auto& inStrandSegment : inStrandSegments)
			{
				strandGroup.m_strandSegments.emplace_back();
				auto& strandSegment = strandGroup.m_strandSegments.back();
				strandSegment.m_handle = strandSegmentHandle;
				if (inStrandSegment["E"]) strandSegment.m_endSegment = inStrandSegment["E"].as<bool>();
				if (inStrandSegment["R"]) strandSegment.m_recycled = inStrandSegment["R"].as<bool>();
				if (inStrandSegment["P"]) strandSegment.m_prevHandle = inStrandSegment["P"].as<StrandSegmentHandle>();
				if (inStrandSegment["N"]) strandSegment.m_nextHandle = inStrandSegment["N"].as<StrandSegmentHandle>();
				if (inStrandSegment["S"]) strandSegment.m_strandHandle = inStrandSegment["S"].as<StrandSegmentHandle>();
				if (inStrandSegment["I"]) strandSegment.m_index = inStrandSegment["I"].as<int>();

				if (inStrandSegment["IF"])
				{
					const auto& inStrandSegmentInfo = inStrandSegment["IF"];
					if (inStrandSegmentInfo["GP"]) strandSegment.m_info.m_globalPosition = inStrandSegmentInfo["GP"].as<glm::vec3>();
					if (inStrandSegmentInfo["GR"]) strandSegment.m_info.m_globalRotation = inStrandSegmentInfo["GR"].as<glm::quat>();
					if (inStrandSegmentInfo["T"]) strandSegment.m_info.m_thickness = inStrandSegmentInfo["T"].as<float>();
					if (inStrandSegmentInfo["C"]) strandSegment.m_info.m_color = inStrandSegmentInfo["C"].as<glm::vec4>();
					if (inStrandSegmentInfo["B"]) strandSegment.m_info.m_isBoundary = inStrandSegmentInfo["B"].as<bool>();
				}
				
				if (inStrandSegment["D"])
				{
					const auto& inStrandSegmentData = inStrandSegment["D"];
					strandSegmentFunc(inStrandSegmentData, strandSegment.m_data);
				}
				strandSegmentHandle++;
			}
		}
		strandGroup.m_strandPool = {};
		strandGroup.m_strandSegmentPool = {};

		for(const auto& strand : strandGroup.m_strands)
		{
			if(strand.m_recycled)
			{
				strandGroup.m_strandPool.emplace(strand.m_handle);
			}
		}

		for (const auto& strandSegment : strandGroup.m_strandSegments)
		{
			if (strandSegment.m_recycled)
			{
				strandGroup.m_strandSegmentPool.emplace(strandSegment.m_handle);
			}
		}
	}
}
