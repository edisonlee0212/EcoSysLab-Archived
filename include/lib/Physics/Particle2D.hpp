#pragma once
#include "ParticleGrid2D.hpp"
using namespace EvoEngine;
namespace EcoSysLab {

	struct UpdateSettings
	{
		float m_dt;
		float m_damping = 0.0f;
		float m_maxVelocity = 1.0f;
		bool m_checkpoint = false;
	};

	template<typename T>
	class Particle2D
	{
		template<typename PD>
		friend class ParticlePhysics2D;
		glm::vec4 m_color = glm::vec4(1.0f);
		glm::vec2 m_position = glm::vec2(0.0f);
		glm::vec2 m_lastPosition = glm::vec2(0.0f);
		glm::vec2 m_acceleration = glm::vec2(0.0f);
		float m_damping = 0.0f;

		glm::vec2 m_deltaPosition = glm::vec2(0.0f);
		ParticleHandle m_handle = -1;

		glm::vec2 m_checkpointPosition = glm::vec2(0.0f);
	public:
		bool m_enable = true;

		T m_data;
		void Update(const UpdateSettings& updateSettings);
		void Stop();
		[[nodiscard]] ParticleHandle GetHandle() const;
		[[nodiscard]] glm::vec4 GetColor() const;
		void SetColor(const glm::vec4& color);

		[[nodiscard]] glm::vec2 GetPosition() const;
		void SetPosition(const glm::vec2& position);

		void Move(const glm::vec2& position);

		[[nodiscard]] glm::vec2 GetVelocity(float dt) const;
		void SetVelocity(const glm::vec2& velocity, float dt);

		[[nodiscard]] glm::vec2 GetAcceleration() const;
		void SetAcceleration(const glm::vec2& acceleration);

		[[nodiscard]] glm::vec2 GetCheckpointPosition() const;
	};


	template <typename T>
	void Particle2D<T>::Update(const UpdateSettings& updateSettings)
	{
		const auto lastV = m_position - m_lastPosition - m_damping * (m_position - m_lastPosition);
		m_lastPosition = m_position;
		auto targetV = lastV + m_acceleration * updateSettings.m_dt * updateSettings.m_dt;
		const auto speed = glm::length(targetV);
		if (speed > glm::epsilon<float>()) {
			targetV = glm::min(updateSettings.m_maxVelocity * updateSettings.m_dt, speed) * glm::normalize(targetV);
			m_position = m_position + targetV;
		}
		m_acceleration = {};
	}

	template <typename T>
	void Particle2D<T>::Stop()
	{
		m_lastPosition = m_position;
	}

	template <typename T>
	ParticleHandle Particle2D<T>::GetHandle() const
	{
		return m_handle;
	}

	template <typename T>
	glm::vec4 Particle2D<T>::GetColor() const
	{
		return m_color;
	}

	template <typename T>
	void Particle2D<T>::SetColor(const glm::vec4& color)
	{
		m_color = color;
	}

	template <typename T>
	glm::vec2 Particle2D<T>::GetPosition() const
	{
		return m_position;
	}

	template <typename T>
	void Particle2D<T>::SetPosition(const glm::vec2& position)
	{
		const auto velocity = m_position - m_lastPosition;
		m_position = m_checkpointPosition = position;
		m_lastPosition = m_position - velocity;
	}

	template <typename T>
	void Particle2D<T>::Move(const glm::vec2& position)
	{
		m_position = position;
	}

	template <typename T>
	glm::vec2 Particle2D<T>::GetVelocity(const float dt) const
	{
		return (m_position - m_lastPosition) / dt;
	}

	template <typename T>
	void Particle2D<T>::SetVelocity(const glm::vec2& velocity, const float dt)
	{
		m_lastPosition = m_position - velocity * dt;
	}

	template <typename T>
	glm::vec2 Particle2D<T>::GetAcceleration() const
	{
		return m_acceleration;
	}

	template <typename T>
	void Particle2D<T>::SetAcceleration(const glm::vec2& acceleration)
	{
		m_acceleration = acceleration;
	}

	template <typename T>
	glm::vec2 Particle2D<T>::GetCheckpointPosition() const
	{
		return m_checkpointPosition;
	}
}
