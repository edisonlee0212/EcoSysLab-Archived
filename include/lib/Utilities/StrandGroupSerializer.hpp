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
		

		const auto strandSize = strandGroup.m_strands.size();
		auto strandRecycledList = std::vector<int>(strandSize);
		auto strandInfoColorList = std::vector<glm::vec4>(strandSize);
		auto strandInfoBaseInfoGlobalPositionList = std::vector<glm::vec3>(strandSize);
		auto strandInfoBaseInfoThicknessList = std::vector<float>(strandSize);
		auto strandInfoBaseInfoColorList = std::vector<glm::vec4>(strandSize);
		auto strandInfoBaseInfoIsBoundaryList = std::vector<int>(strandSize);

		for (int strandIndex = 0; strandIndex < strandSize; strandIndex++)
		{
			const auto& strand = strandGroup.m_strands[strandIndex];
			strandRecycledList[strandIndex] = strand.m_recycled ? 1 : 0;
			strandInfoColorList[strandIndex] = strand.m_info.m_color;
			strandInfoBaseInfoGlobalPositionList[strandIndex] = strand.m_info.m_baseInfo.m_globalPosition;
			strandInfoBaseInfoThicknessList[strandIndex] = strand.m_info.m_baseInfo.m_thickness;
			strandInfoBaseInfoColorList[strandIndex] = strand.m_info.m_baseInfo.m_color;
			strandInfoBaseInfoIsBoundaryList[strandIndex] = strand.m_info.m_baseInfo.m_isBoundary;
		}
		if (strandSize != 0) {
			out << YAML::Key << "m_strands.m_recycled" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandRecycledList.data()), strandRecycledList.size() * sizeof(int));
			out << YAML::Key << "m_strands.m_info.m_color" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandInfoColorList.data()), strandInfoColorList.size() * sizeof(glm::vec4));
			out << YAML::Key << "m_strands.m_info.m_baseInfo.m_globalPosition" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandInfoBaseInfoGlobalPositionList.data()), strandInfoBaseInfoGlobalPositionList.size() * sizeof(glm::vec3));
			out << YAML::Key << "m_strands.m_info.m_baseInfo.m_thickness" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandInfoBaseInfoThicknessList.data()), strandInfoBaseInfoThicknessList.size() * sizeof(float));
			out << YAML::Key << "m_strands.m_info.m_baseInfo.m_color" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandInfoBaseInfoColorList.data()), strandInfoBaseInfoColorList.size() * sizeof(glm::vec4));
			out << YAML::Key << "m_strands.m_info.m_baseInfo.m_isBoundary" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandInfoBaseInfoIsBoundaryList.data()), strandInfoBaseInfoIsBoundaryList.size() * sizeof(int));
		}
		out << YAML::Key << "m_strands" << YAML::Value << YAML::BeginSeq;
		for (const auto& strand : strandGroup.m_strands)
		{
			out << YAML::BeginMap;
			{
				if (!strand.m_strandSegmentHandles.empty()) {
					out << YAML::Key << "m_strandSegmentHandles" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(strand.m_strandSegmentHandles.data()), strand.m_strandSegmentHandles.size() * sizeof(StrandSegmentHandle));
				}
				out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
				{
					strandFunc(out, strand.m_data);
				}
				out << YAML::EndMap;
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;

		const auto strandSegmentSize = strandGroup.m_strandSegments.size();
		auto strandSegmentEndSegmentList = std::vector<int>(strandSegmentSize);
		auto strandSegmentRecycledList = std::vector<int>(strandSegmentSize);

		auto strandSegmentPrevList = std::vector<StrandSegmentHandle>(strandSegmentSize);
		auto strandSegmentNextList = std::vector<StrandSegmentHandle>(strandSegmentSize);
		auto strandSegmentStrandHandleList = std::vector<StrandHandle>(strandSegmentSize);
		auto strandSegmentIndexList = std::vector<int>(strandSegmentSize);

		auto strandSegmentGlobalPositionList = std::vector<glm::vec3>(strandSegmentSize);
		auto strandSegmentThicknessList = std::vector<float>(strandSegmentSize);
		auto strandSegmentColorList = std::vector<glm::vec4>(strandSegmentSize);
		auto strandSegmentBoundaryList = std::vector<int>(strandSegmentSize);

		for (int strandSegmentIndex = 0; strandSegmentIndex < strandSegmentSize; strandSegmentIndex++)
		{
			const auto& strandSegment = strandGroup.m_strandSegments[strandSegmentIndex];
			strandSegmentRecycledList[strandSegmentIndex] = strandSegment.m_recycled ? 1 : 0;
			strandSegmentEndSegmentList[strandSegmentIndex] = strandSegment.m_endSegment ? 1 : 0;

			strandSegmentPrevList[strandSegmentIndex] = strandSegment.m_prevHandle;
			strandSegmentNextList[strandSegmentIndex] = strandSegment.m_nextHandle;
			strandSegmentStrandHandleList[strandSegmentIndex] = strandSegment.m_strandHandle;
			strandSegmentIndexList[strandSegmentIndex] = strandSegment.m_index;

			strandSegmentGlobalPositionList[strandSegmentIndex] = strandSegment.m_info.m_globalPosition;
			strandSegmentThicknessList[strandSegmentIndex] = strandSegment.m_info.m_thickness;
			strandSegmentColorList[strandSegmentIndex] = strandSegment.m_info.m_color;
			strandSegmentBoundaryList[strandSegmentIndex] = strandSegment.m_info.m_isBoundary;
		}
		if (strandSegmentSize != 0) {
			out << YAML::Key << "m_strandSegments.m_endSegment" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentEndSegmentList.data()), strandSegmentEndSegmentList.size() * sizeof(int));
			out << YAML::Key << "m_strandSegments.m_recycled" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentRecycledList.data()), strandSegmentRecycledList.size() * sizeof(int));

			out << YAML::Key << "m_strandSegments.m_prevHandle" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentPrevList.data()), strandSegmentPrevList.size() * sizeof(StrandSegmentHandle));
			out << YAML::Key << "m_strandSegments.m_nextHandle" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentNextList.data()), strandSegmentNextList.size() * sizeof(StrandSegmentHandle));
			out << YAML::Key << "m_strandSegments.m_strandHandle" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentStrandHandleList.data()), strandSegmentStrandHandleList.size() * sizeof(StrandHandle));
			out << YAML::Key << "m_strandSegments.m_index" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentIndexList.data()), strandSegmentIndexList.size() * sizeof(int));

			out << YAML::Key << "m_strandSegments.m_info.m_globalPosition" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentGlobalPositionList.data()), strandSegmentGlobalPositionList.size() * sizeof(glm::vec3));
			out << YAML::Key << "m_strandSegments.m_info.m_thickness" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentThicknessList.data()), strandSegmentThicknessList.size() * sizeof(float));
			out << YAML::Key << "m_strandSegments.m_info.m_color" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentColorList.data()), strandSegmentColorList.size() * sizeof(glm::vec4));
			out << YAML::Key << "m_strandSegments.m_info.m_isBoundary" << YAML::Value << YAML::Binary(
				reinterpret_cast<const unsigned char*>(strandSegmentBoundaryList.data()), strandSegmentBoundaryList.size() * sizeof(int));
		}
		out << YAML::Key << "m_strandSegments.m_data" << YAML::Value << YAML::BeginSeq;
		for (const auto& strandSegment : strandGroup.m_strandSegments)
		{
			out << YAML::BeginMap;
			{
				strandSegmentFunc(out, strandSegment.m_data);
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;

		out << YAML::Key << "m_data" << YAML::Value << YAML::BeginMap;
		groupFunc(out, strandGroup.m_data);
		out << YAML::EndMap;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	void StrandGroupSerializer<StrandGroupData, StrandData, StrandSegmentData>::Deserialize(const YAML::Node& in,
		StrandGroup<StrandGroupData, StrandData, StrandSegmentData>& strandGroup,
		const std::function<void(const YAML::Node& segmentIn, StrandSegmentData& segmentData)>& strandSegmentFunc,
		const std::function<void(const YAML::Node& strandIn, StrandData& strandData)>& strandFunc,
		const std::function<void(const YAML::Node& groupIn, StrandGroupData& groupData)>& groupFunc)
	{
		if (in["m_strands.m_recycled"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_strands.m_recycled"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());

			strandGroup.m_strands.resize(list.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strands[i].m_recycled = list[i] == 1;
			}
		}

		if (in["m_strands.m_info.m_color"])
		{
			auto list = std::vector<glm::vec4>();
			const auto data = in["m_strands.m_info.m_color"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec4));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strands[i].m_info.m_color = list[i];
			}
		}

		if (in["m_strands.m_info.m_baseInfo.m_globalPosition"])
		{
			auto list = std::vector<glm::vec3>();
			const auto data = in["m_strands.m_info.m_baseInfo.m_globalPosition"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec3));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strands[i].m_info.m_baseInfo.m_globalPosition = list[i];
			}
		}

		if (in["m_strands.m_info.m_baseInfo.m_thickness"])
		{
			auto list = std::vector<float>();
			const auto data = in["m_strands.m_info.m_baseInfo.m_thickness"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(float));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strands[i].m_info.m_baseInfo.m_thickness = list[i];
			}
		}

		if (in["m_strands.m_info.m_baseInfo.m_color"])
		{
			auto list = std::vector<glm::vec4>();
			const auto data = in["m_strands.m_info.m_baseInfo.m_color"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec4));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strands[i].m_info.m_baseInfo.m_color = list[i];
			}
		}

		if (in["m_strands.m_info.m_baseInfo.m_isBoundary"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_strands.m_info.m_baseInfo.m_isBoundary"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strands[i].m_info.m_baseInfo.m_isBoundary = list[i] == 1;
			}
		}

		if(in["m_strands"])
		{
			const auto& inStrands = in["m_strands"];
			StrandHandle strandHandle = 0;
			for(const auto& inStrand : inStrands)
			{
				auto& strand = strandGroup.m_strands.at(strandHandle);
				strand.m_handle = strandHandle;
				if (inStrand["m_strandSegmentHandles"])
				{
					const auto segmentHandles = inStrand["m_strandSegmentHandles"].as<YAML::Binary>();
					strand.m_strandSegmentHandles.resize(segmentHandles.size() / sizeof(StrandSegmentHandle));
					std::memcpy(strand.m_strandSegmentHandles.data(), segmentHandles.data(), segmentHandles.size());
				}
				if (inStrand["m_data"])
				{
					const auto& inStrandData = inStrand["D"];
					strandFunc(inStrandData, strand.m_data);
				}
				strandHandle++;
			}
		}

		if (in["m_strandSegments.m_endSegment"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_strandSegments.m_endSegment"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());

			strandGroup.m_strandSegments.resize(list.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_endSegment = list[i] == 1;
			}
		}

		if (in["m_strandSegments.m_recycled"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_strandSegments.m_recycled"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_recycled = list[i] == 1;
			}
		}

		if (in["m_strandSegments.m_prevHandle"])
		{
			auto list = std::vector<StrandSegmentHandle>();
			const auto data = in["m_strandSegments.m_prevHandle"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(StrandSegmentHandle));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_prevHandle = list[i];
			}
		}

		if (in["m_strandSegments.m_nextHandle"])
		{
			auto list = std::vector<StrandSegmentHandle>();
			const auto data = in["m_strandSegments.m_nextHandle"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(StrandSegmentHandle));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_nextHandle = list[i];
			}
		}

		if (in["m_strandSegments.m_strandHandle"])
		{
			auto list = std::vector<StrandHandle>();
			const auto data = in["m_strandSegments.m_strandHandle"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(StrandHandle));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_strandHandle = list[i];
			}
		}

		if (in["m_strandSegments.m_index"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_strandSegments.m_index"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_index = list[i];
			}
		}

		if (in["m_strandSegments.m_info.m_globalPosition"])
		{
			auto list = std::vector<glm::vec3>();
			const auto data = in["m_strandSegments.m_info.m_globalPosition"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec3));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_info.m_globalPosition = list[i];
			}
		}
		
		if (in["m_strandSegments.m_info.m_thickness"])
		{
			auto list = std::vector<float>();
			const auto data = in["m_strandSegments.m_info.m_thickness"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(float));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_info.m_thickness = list[i];
			}
		}

		if (in["m_strandSegments.m_info.m_color"])
		{
			auto list = std::vector<glm::vec4>();
			const auto data = in["m_strandSegments.m_info.m_color"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec4));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_info.m_color = list[i];
			}
		}

		if (in["m_strandSegments.m_info.m_isBoundary"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_strandSegments.m_info.m_isBoundary"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandGroup.m_strandSegments[i].m_info.m_isBoundary = list[i] == 1;
			}
		}

		if (in["m_strandSegments"])
		{
			const auto& inStrandSegments = in["m_strandSegments"];
			StrandSegmentHandle strandSegmentHandle = 0;
			for (const auto& inStrandSegment : inStrandSegments)
			{
				auto& strandSegment = strandGroup.m_strandSegments.at(strandSegmentHandle);
				strandSegment.m_handle = strandSegmentHandle;
				strandSegmentFunc(inStrandSegment, strandSegment.m_data);
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

		if (in["m_data"]) groupFunc(in["m_data"], strandGroup.m_data);
	}
}
