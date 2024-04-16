#pragma once

#include "StrandModelProfile.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	template<typename ParticleData>
	class StrandModelProfileSerializer {
	public:
		static void Serialize(YAML::Emitter& out, const StrandModelProfile<ParticleData>& strandModelProfile,
			const std::function<void(YAML::Emitter& particleOut, const ParticleData& particleData)>& particleFunc);

		static void Deserialize(const YAML::Node& in, StrandModelProfile<ParticleData>& strandModelProfile,
			const std::function<void(const YAML::Node& particleIn, ParticleData& particleData)>& particleFunc);
	};

	template <typename ParticleData>
	void StrandModelProfileSerializer<ParticleData>::Serialize(YAML::Emitter& out,
		const StrandModelProfile<ParticleData>& strandModelProfile,
		const std::function<void(YAML::Emitter& particleOut, const ParticleData& particleData)>& particleFunc)
	{
		const auto particleSize = strandModelProfile.m_particles2D.size();
		auto colorList = std::vector<glm::vec3>(particleSize);
		auto positionList = std::vector<glm::vec2>(particleSize);
		auto lastPositionList = std::vector<glm::vec2>(particleSize);
		auto accelerationList = std::vector<glm::vec2>(particleSize);
		auto deltaPositionList = std::vector<glm::vec2>(particleSize);
		auto boundaryList = std::vector<int>(particleSize);
		auto distanceToBoundaryList = std::vector<float>(particleSize);
		auto initialPositionList = std::vector<glm::vec2>(particleSize);

		auto correspondingChildNodeHandleList = std::vector<SkeletonNodeHandle>(particleSize);
		auto strandList = std::vector<StrandHandle>(particleSize);
		auto strandSegmentHandleList = std::vector<StrandSegmentHandle>(particleSize);
		auto mainChildList = std::vector<int>(particleSize);
		auto baseList = std::vector<int>(particleSize);


		for (int particleIndex = 0; particleIndex < particleSize; particleIndex++)
		{
			const auto& particle = strandModelProfile.m_particles2D[particleIndex];
			colorList[particleIndex] = particle.m_color;
			positionList[particleIndex] = particle.m_position;
			lastPositionList[particleIndex] = particle.m_lastPosition;
			accelerationList[particleIndex] = particle.m_acceleration;
			deltaPositionList[particleIndex] = particle.m_deltaPosition;

			boundaryList[particleIndex] = particle.m_boundary ? 1 : 0;
			distanceToBoundaryList[particleIndex] = particle.m_distanceToBoundary;
			initialPositionList[particleIndex] = particle.m_initialPosition;

			correspondingChildNodeHandleList[particleIndex] = particle.m_correspondingChildNodeHandle;
			strandList[particleIndex] = particle.m_strandHandle;
			strandSegmentHandleList[particleIndex] = particle.m_strandSegmentHandle;
			mainChildList[particleIndex] = particle.m_mainChild ? 1 : 0;
			baseList[particleIndex] = particle.m_base ? 1 : 0;
		}
		out << YAML::Key << "m_particles.m_color" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(colorList.data()), colorList.size() * sizeof(glm::vec3));
		out << YAML::Key << "m_particles.m_position" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(positionList.data()), positionList.size() * sizeof(glm::vec2));
		out << YAML::Key << "m_particles.m_lastPosition" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(lastPositionList.data()), lastPositionList.size() * sizeof(glm::vec2));
		out << YAML::Key << "m_particles.m_acceleration" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(accelerationList.data()), accelerationList.size() * sizeof(glm::vec2));
		out << YAML::Key << "m_particles.m_deltaPosition" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(deltaPositionList.data()), deltaPositionList.size() * sizeof(glm::vec2));
		out << YAML::Key << "m_particles.m_boundary" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(boundaryList.data()), boundaryList.size() * sizeof(int));
		out << YAML::Key << "m_particles.m_distanceToBoundary" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(distanceToBoundaryList.data()), distanceToBoundaryList.size() * sizeof(float));
		out << YAML::Key << "m_particles.m_initialPosition" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(initialPositionList.data()), initialPositionList.size() * sizeof(glm::vec2));

		out << YAML::Key << "m_particles.m_correspondingChildNodeHandle" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(correspondingChildNodeHandleList.data()), correspondingChildNodeHandleList.size() * sizeof(SkeletonNodeHandle));
		out << YAML::Key << "m_particles.m_strandHandle" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(strandList.data()), strandList.size() * sizeof(StrandHandle));
		out << YAML::Key << "m_particles.m_strandSegmentHandle" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(strandSegmentHandleList.data()), strandSegmentHandleList.size() * sizeof(StrandSegmentHandle));
		out << YAML::Key << "m_particles.m_mainChild" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(mainChildList.data()), mainChildList.size() * sizeof(int));
		out << YAML::Key << "m_particles.m_base" << YAML::Value << YAML::Binary(
			reinterpret_cast<const unsigned char*>(baseList.data()), baseList.size() * sizeof(int));


		out << YAML::Key << "m_particles.m_data" << YAML::Value << YAML::BeginSeq;
		for (const auto& particles2D : strandModelProfile.m_particles2D)
		{
			out << YAML::BeginMap;
			{
				particleFunc(out, particles2D.m_data);
			}
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
	}

	template <typename ParticleData>
	void StrandModelProfileSerializer<ParticleData>::Deserialize(const YAML::Node& in,
		StrandModelProfile<ParticleData>& strandModelProfile,
		const std::function<void(const YAML::Node& particleIn, ParticleData& particleData)>& particleFunc)
	{
		if (in["m_particles.m_color"])
		{
			auto list = std::vector<glm::vec3>();
			const auto data = in["m_particles.m_color"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec3));
			std::memcpy(list.data(), data.data(), data.size());

			strandModelProfile.m_particles2D.resize(list.size());
			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_color = list[i];
			}
		}

		if (in["m_particles.m_position"])
		{
			auto list = std::vector<glm::vec2>();
			const auto data = in["m_particles.m_position"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec2));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_position = list[i];
			}
		}

		if (in["m_particles.m_lastPosition"])
		{
			auto list = std::vector<glm::vec2>();
			const auto data = in["m_particles.m_lastPosition"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec2));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_lastPosition = list[i];
			}
		}

		if (in["m_particles.m_acceleration"])
		{
			auto list = std::vector<glm::vec2>();
			const auto data = in["m_particles.m_acceleration"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec2));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_acceleration = list[i];
			}
		}

		if (in["m_particles.m_deltaPosition"])
		{
			auto list = std::vector<glm::vec2>();
			const auto data = in["m_particles.m_deltaPosition"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec2));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_deltaPosition = list[i];
			}
		}

		if (in["m_particles.m_boundary"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_particles.m_boundary"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_boundary = list[i] == 1;
			}
		}

		if (in["m_particles.m_distanceToBoundary"])
		{
			auto list = std::vector<float>();
			const auto data = in["m_particles.m_distanceToBoundary"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(float));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_distanceToBoundary = list[i];
			}
		}

		if (in["m_particles.m_initialPosition"])
		{
			auto list = std::vector<glm::vec2>();
			const auto data = in["m_particles.m_initialPosition"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(glm::vec2));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_initialPosition = list[i];
			}
		}

		if (in["m_particles.m_correspondingChildNodeHandle"])
		{
			auto list = std::vector<SkeletonNodeHandle>();
			const auto data = in["m_particles.m_correspondingChildNodeHandle"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(SkeletonNodeHandle));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_correspondingChildNodeHandle = list[i];
			}
		}

		if (in["m_particles.m_strandHandle"])
		{
			auto list = std::vector<StrandHandle>();
			const auto data = in["m_particles.m_strandHandle"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(StrandHandle));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_strandHandle = list[i];
			}
		}

		if (in["m_particles.m_strandSegmentHandle"])
		{
			auto list = std::vector<StrandSegmentHandle>();
			const auto data = in["m_particles.m_strandSegmentHandle"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(StrandSegmentHandle));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_strandSegmentHandle = list[i];
			}
		}

		if (in["m_particles.m_mainChild"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_particles.m_mainChild"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_mainChild = list[i] == 1;
			}
		}

		if (in["m_particles.m_base"])
		{
			auto list = std::vector<int>();
			const auto data = in["m_particles.m_base"].as<YAML::Binary>();
			list.resize(data.size() / sizeof(int));
			std::memcpy(list.data(), data.data(), data.size());

			for (size_t i = 0; i < list.size(); i++)
			{
				strandModelProfile.m_particles2D[i].m_base = list[i] == 1;
			}
		}

		if (in["m_particles.m_data"])
		{
			const auto& inParticleDataList = in["m_particles.m_data"];
			ParticleHandle particleHandle = 0;
			for (const auto& inParticle2D : inParticleDataList)
			{
				auto& particle2D = strandModelProfile.m_particles2D[particleHandle];
				particle2D.m_handle = particleHandle;
				particleFunc(inParticle2D, particle2D.m_data);
				particleHandle++;
			}
		}
	}
}
