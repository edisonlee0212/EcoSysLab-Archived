#pragma once

#include "ecosyslab_export.h"
#include "PipeGroup.hpp"
#include "Skeleton.hpp"

namespace EcoSysLab
{
	typedef int CellHandle;
	typedef int ProfileHandle;
#pragma region Structural Info
	struct CellInfo
	{
		glm::vec2 m_offset = glm::vec2(0.0f);
		float m_radius = 0.0f;
		bool m_boundary = false;
	};

#pragma endregion
	template<typename CellData>
	class PipeCell
	{
		CellHandle m_handle = -1;
		bool m_recycled = false;

		template<typename PD, typename CD>
		friend class PipeProfile;

		template<typename PGD, typename PD, typename CD>
		friend class PipeProfileGroup;
	public:
		CellData m_data = {};
		CellInfo m_info = {};
		[[nodiscard]] CellHandle GetHandle() const;
		[[nodiscard]] bool IsRecycled() const;


		explicit PipeCell(CellHandle handle);
	};

	struct ProfileInfo
	{
		std::vector<glm::vec2> m_boundary = {};
		bool m_boundaryValid = false;

		
		void CheckBoundary();
		[[nodiscard]] static bool IsBoundaryValid(const std::vector<glm::vec2>& points);
		[[nodiscard]] bool InBoundary(const glm::vec2& point) const;
	};

	template<typename ProfileData, typename CellData>
	class PipeProfile
	{
		std::vector<PipeCell<CellData>> m_cells;

		std::queue<CellHandle> m_cellPool;
		int m_version = -1;

		ProfileHandle m_handle = -1;
		bool m_recycled = false;


		bool m_addingLine = false;
		template<typename PGD, typename PD, typename CD>
		friend class PipeProfileGroup;
	public:
		[[nodiscard]] bool OnInspect(bool editable);

		ProfileData m_data = {};
		ProfileInfo m_info = {};
		[[nodiscard]] bool IsRecycled() const;

		[[nodiscard]] ProfileHandle GetHandle() const;

		void RecycleCell(CellHandle handle);

		[[nodiscard]] CellHandle AllocateCell();

		[[nodiscard]] PipeCell<CellData>& RefCell(CellHandle handle);

		[[nodiscard]] const PipeCell<CellData>& PeekCell(CellHandle handle) const;

		[[nodiscard]] const std::vector<PipeCell<CellData>>& PeekCells() const;

		[[nodiscard]] std::vector<PipeCell<CellData>>& RefCells();

		[[nodiscard]] const std::queue<CellHandle>& PeekCellPool() const;

		[[nodiscard]] std::queue<CellHandle>& RefCellPool();

		[[nodiscard]] size_t GetCellSize() const;

		PipeProfile();
		explicit PipeProfile(ProfileHandle handle);

		void FillCells();
	};

	template<typename GroupData, typename ProfileData, typename CellData>
	class PipeProfileGroup
	{
		std::vector<PipeProfile<ProfileData, CellData>> m_profiles;
		std::queue<ProfileHandle> m_profilePool;

		int m_version = -1;
	public:
		[[nodiscard]] ProfileHandle Allocate();

		void RecycleProfile(ProfileHandle handle);

		[[nodiscard]] const PipeProfile<ProfileData, CellData>& PeekProfile(ProfileHandle handle) const;

		[[nodiscard]] PipeProfile<ProfileData, CellData>& RefProfile(ProfileHandle handle);

		[[nodiscard]] std::vector<PipeProfile<ProfileData, CellData>>& RefProfiles();

		[[nodiscard]] std::queue<ProfileHandle>& RefProfilePool();
	};

	template <typename CellData>
	CellHandle PipeCell<CellData>::GetHandle() const
	{
		return m_handle;
	}

	template <typename CellData>
	bool PipeCell<CellData>::IsRecycled() const
	{
		return m_recycled;
	}

	template <typename CellData>
	PipeCell<CellData>::PipeCell(CellHandle handle)
	{
		m_handle = handle;
		m_recycled = false;
		m_data = {};
		m_info = {};
	}

	template <typename ProfileData, typename CellData>
	bool PipeProfile<ProfileData, CellData>::OnInspect(const bool editable)
	{
		bool changed = false;

		static auto scrolling = glm::vec2(0.0f);
		static float zoomFactor = 10.0f;
		if (ImGui::Button("Recenter")) {
			scrolling = glm::vec2(0.0f);
		}
		ImGui::DragFloat("Zoom", &zoomFactor, zoomFactor / 100.0f, 1.0f, 50.0f);
		zoomFactor = glm::clamp(zoomFactor, 1.0f, 50.0f);
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
		if (editable && isMouseHovered && !m_addingLine && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
			m_info.m_boundary.clear();
			m_info.m_boundary.emplace_back(mousePosInCanvas.x, mousePosInCanvas.y);
			m_addingLine = true;
		}
		for (const auto& cell : m_cells) {
			if(cell.IsRecycled()) continue;
			const auto pointPosition = cell.m_info.m_offset;
			const auto pointRadius = cell.m_info.m_radius;
			const auto canvasPosition = ImVec2(origin.x + pointPosition.x * zoomFactor,
				origin.y + pointPosition.y * zoomFactor);

			drawList->AddCircleFilled(canvasPosition,
				glm::clamp(0.4f * zoomFactor * pointRadius, 1.0f, 100.0f),
				IM_COL32(255, 255, 255, 255));
			/*
			if (knot->m_selected) {
				draw_list->AddCircle(canvasPosition,
					glm::clamp(0.5f * zoomFactor, 1.0f, 100.0f),
					IM_COL32(255,
						255,
						0, 128));
			}

			if (zoomFactor > 20) {
				auto textCanvasPosition = ImVec2(origin.x + pointPosition.x * zoomFactor - 0.3f * zoomFactor,
					origin.y + pointPosition.y * zoomFactor - 0.3f * zoomFactor);
				auto text = std::to_string(knot->m_distanceToBoundary);
				draw_list->AddText(nullptr, 0.5f * zoomFactor, textCanvasPosition, IM_COL32(255, 0, 0, 255),
					text.c_str());
			}
			*/
		}

		drawList->AddCircle(origin,
			glm::clamp(0.5f * zoomFactor, 1.0f, 100.0f),
			IM_COL32(255,
				0,
				0, 255));
		if (m_info.m_boundary.size() >= 2) {
			for (int i = 0; i < m_info.m_boundary.size() - 1; i++) {
				drawList->AddLine(ImVec2(origin.x + m_info.m_boundary[i].x * zoomFactor,
					origin.y + m_info.m_boundary[i].y * zoomFactor),
					ImVec2(origin.x + m_info.m_boundary[i + 1].x * zoomFactor,
						origin.y + m_info.m_boundary[i + 1].y * zoomFactor),
					IM_COL32(255, 0, 0, 255), 2.0f);
			}
		}
		if (m_addingLine) {
			if (glm::distance(m_info.m_boundary.back(), { mousePosInCanvas.x, mousePosInCanvas.y }) >= 10.0f / zoomFactor)
				m_info.m_boundary.emplace_back(mousePosInCanvas.x, mousePosInCanvas.y);
			if (editable && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
				m_addingLine = false;
				changed = true;
				m_info.CheckBoundary();
				if (m_info.m_boundaryValid) {
					FillCells();
				}
			}
		}else if(m_info.m_boundaryValid)
		{
			for (int i = 0; i < m_info.m_boundary.size(); i++)
			{
				drawList->AddLine(ImVec2(origin.x + m_info.m_boundary[m_info.m_boundary.size() - 1].x * zoomFactor,
					origin.y + m_info.m_boundary[m_info.m_boundary.size() - 1].y * zoomFactor),
					ImVec2(origin.x + m_info.m_boundary[0].x * zoomFactor,
						origin.y + m_info.m_boundary[0].y * zoomFactor),
					IM_COL32(255, 0, 0, 255), 2.0f);
			}
		}
		drawList->PopClipRect();

		return changed;
	}

	template <typename ProfileData, typename CellData>
	bool PipeProfile<ProfileData, CellData>::IsRecycled() const
	{
		return m_recycled;
	}

	template <typename ProfileData, typename CellData>
	ProfileHandle PipeProfile<ProfileData, CellData>::GetHandle() const
	{
		return m_handle;
	}

	template <typename ProfileData, typename CellData>
	void PipeProfile<ProfileData, CellData>::RecycleCell(CellHandle handle)
	{
		auto& cell = m_cells[handle];
		assert(!cell.m_recycled);
		cell.m_recycled = true;

		cell.m_data = {};
		cell.m_info = {};

		m_cellPool.push(handle);
	}

	template <typename ProfileData, typename CellData>
	CellHandle PipeProfile<ProfileData, CellData>::AllocateCell()
	{
		CellHandle newCellHandle;
		if (m_cellPool.empty()) {
			auto newCell = m_cells.emplace_back(m_cells.size());
			newCellHandle = newCell.m_handle;
		}
		else {
			newCellHandle = m_cellPool.front();
			m_cellPool.pop();
		}
		auto& newCell = m_cells[newCellHandle];
		newCell.m_recycled = false;

		m_version++;

		return newCellHandle;
	}

	template <typename ProfileData, typename CellData>
	PipeCell<CellData>& PipeProfile<ProfileData, CellData>::RefCell(CellHandle handle)
	{
		return m_cells[handle];
	}

	template <typename ProfileData, typename CellData>
	const PipeCell<CellData>& PipeProfile<ProfileData, CellData>::PeekCell(CellHandle handle) const
	{
		return m_cells[handle];
	}

	template <typename ProfileData, typename CellData>
	const std::vector<PipeCell<CellData>>& PipeProfile<ProfileData, CellData>::PeekCells() const
	{
		return m_cells;
	}

	template <typename ProfileData, typename CellData>
	std::vector<PipeCell<CellData>>& PipeProfile<ProfileData, CellData>::RefCells()
	{
		return m_cells;
	}

	template <typename ProfileData, typename CellData>
	const std::queue<CellHandle>& PipeProfile<ProfileData, CellData>::PeekCellPool() const 
	{
		return m_cellPool;
	}

	template <typename ProfileData, typename CellData>
	std::queue<CellHandle>& PipeProfile<ProfileData, CellData>::RefCellPool()
	{
		return m_cellPool;
	}

	template <typename ProfileData, typename CellData>
	size_t PipeProfile<ProfileData, CellData>::GetCellSize() const
	{
		return m_cells.size() - m_cellPool.size();
	}

	template <typename ProfileData, typename CellData>
	PipeProfile<ProfileData, CellData>::PipeProfile()
	{
		m_handle = 0;
		m_recycled = false;
		m_version = -1;
	}

	template <typename ProfileData, typename CellData>
	PipeProfile<ProfileData, CellData>::PipeProfile(const ProfileHandle handle)
	{
		m_handle = handle;
		m_recycled = false;
		m_version = -1;
	}

	template <typename ProfileData, typename CellData>
	void PipeProfile<ProfileData, CellData>::FillCells()
	{
		auto min = glm::vec2(FLT_MAX);
		auto max = glm::vec2(FLT_MIN);
		float radius = 0.1f;
		for(const auto& i : m_cells)
		{
			if(!i.IsRecycled()) RecycleCell(i.GetHandle());
		}
		for(const auto& i : m_info.m_boundary)
		{
			min = glm::min(min, i);
			max = glm::max(i, max);
		}

		for(float i = min.x; i <= max.x; i += radius * 2.0f)
		{
			for(float j = min.y; j <= max.y; j += radius * 2.0f)
			{
				auto position = glm::vec2(i, j);
				if(m_info.InBoundary(position))
				{
					auto newCellHandle = AllocateCell();
					auto& newCell = RefCell(newCellHandle);
					newCell.m_info.m_offset = position;
					newCell.m_info.m_radius = radius;
					newCell.m_info.m_boundary = false;
				}
			}
		}
	}

	template <typename GroupData, typename ProfileData, typename CellData>
	ProfileHandle PipeProfileGroup<GroupData, ProfileData, CellData>::Allocate()
	{
		ProfileHandle newProfileHandle;
		if (m_profilePool.empty()) {
			auto newProfile = m_profiles.emplace_back(m_profiles.size());
			newProfileHandle = newProfile.m_handle;
		}
		else {
			newProfileHandle = m_profilePool.front();
			m_profilePool.pop();
		}
		m_version++;
		m_profiles[newProfileHandle].m_recycled = false;
		
		return newProfileHandle;
	}

	template <typename GroupData, typename ProfileData, typename CellData>
	void PipeProfileGroup<GroupData, ProfileData, CellData>::RecycleProfile(ProfileHandle handle)
	{
		auto& profile = m_profiles[handle];
		assert(!profile.m_recycled);
		profile.m_recycled = true;
		profile.m_cells.clear();
		profile.m_cellPool = {};
		profile.m_info = {};
		profile.m_data = {};
		
		m_profilePool.push(handle);
	}

	template <typename GroupData, typename ProfileData, typename CellData>
	const PipeProfile<ProfileData, CellData>& PipeProfileGroup<GroupData, ProfileData, CellData>::PeekProfile(
		ProfileHandle handle) const
	{
		return m_profiles[handle];
	}

	template <typename GroupData, typename ProfileData, typename CellData>
	PipeProfile<ProfileData, CellData>& PipeProfileGroup<GroupData, ProfileData, CellData>::RefProfile(
		ProfileHandle handle)
	{
		return m_profiles[handle];
	}

	template <typename GroupData, typename ProfileData, typename CellData>
	std::vector<PipeProfile<ProfileData, CellData>>& PipeProfileGroup<GroupData, ProfileData, CellData>::RefProfiles()
	{
		return m_profiles;
	}

	template <typename GroupData, typename ProfileData, typename CellData>
	std::queue<ProfileHandle>& PipeProfileGroup<GroupData, ProfileData, CellData>::RefProfilePool()
	{
		return m_profilePool;
	}
}
