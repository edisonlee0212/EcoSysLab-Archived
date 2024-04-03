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
		out << YAML::Key << "m_particles2D" << YAML::Value << YAML::BeginSeq;
		for (const auto& particles2D : strandModelProfile.m_particles2D)
		{
			out << YAML::BeginMap;
			{
				out << YAML::Key << "C" << YAML::Value << particles2D.m_color;
				out << YAML::Key << "P" << YAML::Value << particles2D.m_position;
				out << YAML::Key << "LP" << YAML::Value << particles2D.m_lastPosition;
				out << YAML::Key << "A" << YAML::Value << particles2D.m_acceleration;
				out << YAML::Key << "DP" << YAML::Value << particles2D.m_deltaPosition;
				out << YAML::Key << "B" << YAML::Value << particles2D.m_boundary;
				out << YAML::Key << "DB" << YAML::Value << particles2D.m_distanceToBoundary;

				out << YAML::Key << "D" << YAML::Value << YAML::BeginMap;
				{
					particleFunc(out, particles2D.m_data);
				}
				out << YAML::EndMap;
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
		if (in["m_particles2D"])
		{
			strandModelProfile.m_particles2D.clear();
			const auto& inParticle2Ds = in["m_particles2D"];
			ParticleHandle particleHandle = 0;
			for (const auto& inParticle2D : inParticle2Ds)
			{
				strandModelProfile.m_particles2D.emplace_back();
				auto& particle2D = strandModelProfile.m_particles2D.back();
				particle2D.m_handle = particleHandle;
				if (inParticle2D["C"]) particle2D.m_color = inParticle2D["C"].as<glm::vec3>();
				if (inParticle2D["P"]) particle2D.m_position = inParticle2D["P"].as<glm::vec2>();
				if (inParticle2D["LP"]) particle2D.m_lastPosition = inParticle2D["LP"].as<glm::vec2>();
				if (inParticle2D["A"]) particle2D.m_acceleration = inParticle2D["A"].as<glm::vec2>();
				if (inParticle2D["DP"]) particle2D.m_deltaPosition = inParticle2D["DP"].as<glm::vec2>();
				if (inParticle2D["B"]) particle2D.m_boundary = inParticle2D["B"].as<bool>();
				if (inParticle2D["DB"]) particle2D.m_distanceToBoundary = inParticle2D["DB"].as<float>();
				if (inParticle2D["D"])
				{
					const auto& inParticle2DData = inParticle2D["D"];
					particleFunc(inParticle2DData, particle2D.m_data);
				}
				particleHandle++;
			}
		}
	}
}
