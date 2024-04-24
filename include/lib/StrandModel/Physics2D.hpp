#pragma once
#include "RigidBody2D.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	typedef int RigidBodyHandle;
	template<typename T>
	class Physics2D
	{
		std::vector<RigidBody2D<T>> m_rigidBodies2D{};
		void SolveContact(RigidBodyHandle p1Handle, RigidBodyHandle p2Handle);
		float m_deltaTime = 0.002f;
		void Update(const std::function<void(RigidBody2D<T>& rigidBody)>& modifyRigidBodyFunc);
	public:
		[[nodiscard]] RigidBodyHandle AllocateRigidBody();
		[[nodiscard]] RigidBody2D<T>& RefRigidBody(RigidBodyHandle handle);
		void RemoveRigidBody(RigidBodyHandle handle);
		void Shift(const glm::vec2& offset);
		[[nodiscard]] const std::vector<RigidBody2D<T>>& PeekRigidBodies() const;
		[[nodiscard]] std::vector<RigidBody2D<T>>& RefRigidBodies();
		void Simulate(float time, const std::function<void(RigidBody2D<T>& rigidBody)>& modifyRigidBodyFunc);

		void OnInspect(const std::function<void(glm::vec2 position)>& func, const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc);
	};

	template <typename T>
	void Physics2D<T>::SolveContact(RigidBodyHandle p1Handle, RigidBodyHandle p2Handle)
	{
		if (p1Handle == p2Handle) return;
		auto& p1 = m_rigidBodies2D.at(p1Handle);
		auto& p2 = m_rigidBodies2D.at(p2Handle);
		const auto difference = p1.m_position - p2.m_position;
		const auto distance = glm::length(difference);
		const auto minDistance = p1.m_thickness + p2.m_thickness;
		if (distance < minDistance)
		{
			const auto axis = distance < glm::epsilon<float>() ? glm::vec2(1, 0) : difference / distance;
			const auto delta = minDistance - distance;
			p1.m_position += 0.5f * delta * axis;
			p2.m_position -= 0.5f * delta * axis;
		}
	}

	template <typename T>
	void Physics2D<T>::Update(
		const std::function<void(RigidBody2D<T>& collisionRigidBody)>& modifyRigidBodyFunc)
	{
		Jobs::RunParallelFor(m_rigidBodies2D.size(), [&](unsigned i)
			{
				modifyRigidBodyFunc(m_rigidBodies2D[i]);
			}
		);
		for (size_t i = 0; i < m_rigidBodies2D.size(); i++)
		{
			for (size_t j = 0; j < m_rigidBodies2D.size(); j++)
			{
				SolveContact(i, j);
			}
		}
		Jobs::RunParallelFor(m_rigidBodies2D.size(), [&](unsigned i)
			{
				m_rigidBodies2D[i].Update(m_deltaTime);
			}
		);
	}

	template <typename T>
	RigidBodyHandle Physics2D<T>::AllocateRigidBody()
	{
		m_rigidBodies2D.emplace_back();
		return m_rigidBodies2D.size() - 1;
	}

	template <typename T>
	RigidBody2D<T>& Physics2D<T>::RefRigidBody(RigidBodyHandle handle)
	{
		return m_rigidBodies2D[handle];
	}

	template <typename T>
	void Physics2D<T>::RemoveRigidBody(RigidBodyHandle handle)
	{
		m_rigidBodies2D[handle] = m_rigidBodies2D.back();
		m_rigidBodies2D.pop_back();
	}

	template <typename T>
	void Physics2D<T>::Shift(const glm::vec2& offset)
	{
		Jobs::ParallelFor(m_rigidBodies2D.size(), [&](unsigned i)
			{
				auto& particle = m_rigidBodies2D[i];
				particle.SetPosition(particle.m_position + offset);
			}
		);
	}

	template <typename T>
	const std::vector<RigidBody2D<T>>& Physics2D<T>::PeekRigidBodies() const
	{
		return m_rigidBodies2D;
	}

	template <typename T>
	std::vector<RigidBody2D<T>>& Physics2D<T>::RefRigidBodies()
	{
		return m_rigidBodies2D;
	}

	template <typename T>
	void Physics2D<T>::Simulate(float time,
		const std::function<void(RigidBody2D<T>& collisionRigidBody)>& modifyRigidBodyFunc)
	{
		const auto count = static_cast<size_t>(glm::round(time / m_deltaTime));
		for (size_t i{ count }; i--;)
		{
			Update(modifyRigidBodyFunc);
		}
	}

	template <typename T>
	void Physics2D<T>::OnInspect(const std::function<void(glm::vec2 position)>& func, const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc)
	{
		static auto scrolling = glm::vec2(0.0f);
		static float zoomFactor = 1.f;
		if (ImGui::Button("Recenter")) {
			scrolling = glm::vec2(0.0f);
		}
		ImGui::DragFloat("Zoom", &zoomFactor, zoomFactor / 100.0f, 0.01f, 50.0f);
		zoomFactor = glm::clamp(zoomFactor, 0.01f, 50.0f);
		const ImGuiIO& io = ImGui::GetIO();
		ImDrawList* drawList = ImGui::GetWindowDrawList();

		const ImVec2 canvasP0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
		ImVec2 canvasSz = ImGui::GetContentRegionAvail();   // Resize canvas to what's available
		if (canvasSz.x < 50.0f) canvasSz.x = 50.0f;
		if (canvasSz.y < 50.0f) canvasSz.y = 50.0f;
		const ImVec2 canvasP1 = ImVec2(canvasP0.x + canvasSz.x, canvasP0.y + canvasSz.y);
		const ImVec2 origin(canvasP0.x + canvasSz.x / 2.0f + scrolling.x,
			canvasP0.y + canvasSz.y / 2.0f + scrolling.y); // Lock scrolled origin
		const ImVec2 mousePosInCanvas((io.MousePos.x - origin.x) / zoomFactor,
			(io.MousePos.y - origin.y) / zoomFactor);

		// Draw border and background color
		drawList->AddRectFilled(canvasP0, canvasP1, IM_COL32(50, 50, 50, 255));
		drawList->AddRect(canvasP0, canvasP1, IM_COL32(255, 255, 255, 255));

		// This will catch our interactions
		ImGui::InvisibleButton("canvas", canvasSz,
			ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
		const bool isMouseHovered = ImGui::IsItemHovered(); // Hovered
		const bool isMouseActive = ImGui::IsItemActive();   // Held

		// Pan (we use a zero mouse threshold when there's no context menu)
		// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
		const float mouseThresholdForPan = -1.0f;
		if (isMouseActive && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouseThresholdForPan)) {
			scrolling.x += io.MouseDelta.x;
			scrolling.y += io.MouseDelta.y;
		}
		// Context menu (under default mouse threshold)
		const ImVec2 dragDelta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
		if (dragDelta.x == 0.0f && dragDelta.y == 0.0f)
			ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);
		if (ImGui::BeginPopup("context")) {

			ImGui::EndPopup();
		}

		// Draw profile + all lines in the canvas
		drawList->PushClipRect(canvasP0, canvasP1, true);
		if (isMouseHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
			func(glm::vec2(mousePosInCanvas.x, mousePosInCanvas.y));
		}
		for (const auto& particle : m_rigidBodies2D) {
			const auto& pointPosition = particle.m_position;
			const auto& pointRadius = particle.m_thickness;
			const auto& pointColor = particle.m_color;
			const auto canvasPosition = ImVec2(origin.x + pointPosition.x * zoomFactor,
				origin.y + pointPosition.y * zoomFactor);

			drawList->AddCircleFilled(canvasPosition,
				glm::clamp(zoomFactor * pointRadius, 1.0f, 100.0f),
				IM_COL32(255.0f * pointColor.x, 255.0f * pointColor.y, 255.0f * pointColor.z, 255.0f * pointColor.w));
		}
		
		drawList->AddCircle(origin,
			glm::clamp(0.5f * zoomFactor, 1.0f, 100.0f),
			IM_COL32(255,
				0,
				0, 255));

		drawFunc(origin, zoomFactor, drawList);
		drawList->PopClipRect();

	}
}
