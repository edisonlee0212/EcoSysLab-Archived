#pragma once

using namespace EvoEngine;
namespace EcoSysLab {
	template<typename T>
	class RigidBody2D
	{
		template<typename PD>
		friend class Physics2D;
		glm::vec4 m_color = glm::vec4(1.0f);
		glm::vec2 m_position = glm::vec2(0.0f);
		glm::vec2 m_lastPosition = glm::vec2(0.0f);
		glm::vec2 m_acceleration = glm::vec2(0.0f);
		float m_thickness = 1.0f;
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

		[[nodiscard]] glm::vec2 GetVelocity() const;
		void SetVelocity(const glm::vec2& velocity);

		[[nodiscard]] glm::vec2 GetAcceleration() const;
		void SetAcceleration(const glm::vec2& acceleration);

		[[nodiscard]] float GetDamping() const;
		void SetDamping(float damping);

		[[nodiscard]] float GetRadius() const;
		void SetRadius(float radius);
	};

	template <typename T>
	void RigidBody2D<T>::Update(const float dt)
	{
		const auto velocity = m_position - m_lastPosition - m_damping * (m_position - m_lastPosition);
		m_lastPosition = m_position;
		m_position = m_position + velocity + m_acceleration * dt * dt;
		m_acceleration = {};
	}

	template <typename T>
	void RigidBody2D<T>::Stop()
	{
		m_lastPosition = m_position;
	}

	template <typename T>
	glm::vec4 RigidBody2D<T>::GetColor() const
	{
		return m_color;
	}

	template <typename T>
	void RigidBody2D<T>::SetColor(const glm::vec4& color)
	{
		m_color = color;
	}

	template <typename T>
	glm::vec2 RigidBody2D<T>::GetPosition() const
	{
		return m_position;
	}

	template <typename T>
	void RigidBody2D<T>::SetPosition(const glm::vec2& position)
	{
		const auto velocity = m_position - m_lastPosition;
		m_position = position;
		m_lastPosition = m_position - velocity;
	}

	template <typename T>
	void RigidBody2D<T>::Move(const glm::vec2& position)
	{
		m_position = position;
	}

	template <typename T>
	glm::vec2 RigidBody2D<T>::GetVelocity() const
	{
		return m_position - m_lastPosition;
	}

	template <typename T>
	void RigidBody2D<T>::SetVelocity(const glm::vec2& velocity)
	{
		m_lastPosition = m_position - velocity;
	}

	template <typename T>
	glm::vec2 RigidBody2D<T>::GetAcceleration() const
	{
		return m_acceleration;
	}

	template <typename T>
	void RigidBody2D<T>::SetAcceleration(const glm::vec2& acceleration)
	{
		m_acceleration = acceleration;
	}

	template <typename T>
	float RigidBody2D<T>::GetDamping() const
	{
		return m_damping;
	}

	template <typename T>
	void RigidBody2D<T>::SetDamping(const float damping)
	{
		m_damping = glm::clamp(damping, 0.0f, 1.0f);
	}

	template <typename T>
	float RigidBody2D<T>::GetRadius() const
	{
		return m_thickness;
	}

	template <typename T>
	void RigidBody2D<T>::SetRadius(const float radius)
	{
		m_thickness = radius;
	}
}