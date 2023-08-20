#pragma once

#include "ecosyslab_export.h"

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
		std::vector<glm::vec2> m_boundary;

		[[nodiscard]] bool IsBoundaryValid() const;
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
		static bool addingLine = false;
		// Context menu (under default mouse threshold)
		const ImVec2 dragDelta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
		if (dragDelta.x == 0.0f && dragDelta.y == 0.0f)
			ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);
		static std::vector<glm::vec2> points = {};
		if (ImGui::BeginPopup("context")) {

			ImGui::EndPopup();
		}

		// Draw profile + all lines in the canvas
		drawList->PushClipRect(canvasP0, canvasP1, true);
		if (editable && isMouseHovered && !addingLine && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
			points.clear();
			points.emplace_back(mousePosInCanvas.x, mousePosInCanvas.y);
			addingLine = true;
		}
		for (const auto& cell : m_cells) {
			if(cell.IsRecycled()) continue;
			const auto pointPosition = cell.m_info.m_offset;
			const auto canvasPosition = ImVec2(origin.x + pointPosition.x * zoomFactor,
				origin.y + pointPosition.y * zoomFactor);

			drawList->AddCircleFilled(canvasPosition,
				glm::clamp(0.4f * zoomFactor, 1.0f, 100.0f),
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
		for (int i = 0; i < m_info.m_boundary.size(); i++)
		{
			drawList->AddLine(ImVec2(origin.x + m_info.m_boundary[i].x * zoomFactor,
				origin.y + m_info.m_boundary[i].y * zoomFactor),
				ImVec2(origin.x + m_info.m_boundary[(i + 1) % m_info.m_boundary.size()].x * zoomFactor,
					origin.y + m_info.m_boundary[(i + 1) % m_info.m_boundary.size()].y * zoomFactor),
				IM_COL32(255, 0, 0, 255), 2.0f);
		}

		drawList->AddCircle(origin,
			glm::clamp(0.5f * zoomFactor, 1.0f, 100.0f),
			IM_COL32(255,
				0,
				0, 255));
		if (addingLine) {
			const auto size = points.size();
			for (int i = 0; i < size - 1; i++) {
				drawList->AddLine(ImVec2(origin.x + points[i].x * zoomFactor,
					origin.y + points[i].y * zoomFactor),
					ImVec2(origin.x + points[i + 1].x * zoomFactor,
						origin.y + points[i + 1].y * zoomFactor),
					IM_COL32(255, 0, 0, 255), 2.0f);
			}
			if (glm::distance(points.back(), { mousePosInCanvas.x, mousePosInCanvas.y }) >= 10.0f / zoomFactor)
				points.emplace_back(mousePosInCanvas.x, mousePosInCanvas.y);
			if (editable && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
				addingLine = false;
				if (!ProfileInfo::IsBoundaryValid(points)) {
					m_info.m_boundary = points;
					changed = true;
				}
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
	}

	template <typename ProfileData, typename CellData>
	PipeProfile<ProfileData, CellData>::PipeProfile(const ProfileHandle handle)
	{
		m_handle = handle;
		m_recycled = false;
		m_version = -1;
	}
}
