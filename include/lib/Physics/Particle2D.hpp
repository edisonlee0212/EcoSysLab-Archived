#pragma once

using namespace EvoEngine;
namespace EcoSysLab {
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
	public:
		T m_data;
		void Update(float dt);
		void Stop();

		[[nodiscard]] glm::vec4 GetColor() const;
		void SetColor(const glm::vec4& color);

		[[nodiscard]] glm::vec2 GetPosition() const;
		void SetPosition(const glm::vec2& position);

		void Move(const glm::vec2& position);

		[[nodiscard]] glm::vec2 GetVelocity(float dt) const;
		void SetVelocity(const glm::vec2& velocity, float dt);

		[[nodiscard]] glm::vec2 GetAcceleration() const;
		void SetAcceleration(const glm::vec2& acceleration);

		[[nodiscard]] float GetDamping() const;
		void SetDamping(float damping);
	};

	template <typename T>
	void Particle2D<T>::Update(const float dt)
	{
		const auto velocity = m_position - m_lastPosition - m_damping * (m_position - m_lastPosition);
		m_lastPosition = m_position;
		m_position = m_position + velocity + m_acceleration * dt * dt;
		m_acceleration = {};
	}

	template <typename T>
	void Particle2D<T>::Stop()
	{
		m_lastPosition = m_position;
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
		m_position = position;
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
	float Particle2D<T>::GetDamping() const
	{
		return m_damping;
	}

	template <typename T>
	void Particle2D<T>::SetDamping(const float damping)
	{
		m_damping = glm::clamp(damping, 0.0f, 1.0f);
	}
}