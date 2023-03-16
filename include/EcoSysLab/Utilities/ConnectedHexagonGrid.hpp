#pragma once

#include "ecosyslab_export.h"

namespace EcoSysLab
{
	enum class HexagonGridDirection
	{
		UpLeft,
		UpRight,
		Right,
		DownRight,
		DownLeft,
		Left
	};

	typedef int HexagonCellHandle;
	template<typename CellData>
	class HexagonCell
	{
		HexagonCellHandle m_handle = -1;
		bool m_recycled = false;

		HexagonCellHandle m_upLeft = -1;
		HexagonCellHandle m_upRight = -1;
		HexagonCellHandle m_right = -1;
		HexagonCellHandle m_downRight = -1;
		HexagonCellHandle m_downLeft = -1;
		HexagonCellHandle m_left = -1;

		glm::ivec2 m_coordinate = glm::ivec2(0);

		template<typename CD>
		friend class ConnectedHexagonGrid;

	public:
		CellData m_data;
		explicit HexagonCell(HexagonCellHandle handle);
	};

	template <typename CellData>
	HexagonCell<CellData>::HexagonCell(HexagonCellHandle handle)
	{
		m_handle = handle;
		m_recycled = false;

		m_upLeft = m_upRight = m_right = m_downRight = m_downLeft = m_left = -1;

		m_data = {};
		m_coordinate = glm::ivec2(0);
	}

	template<typename CellData>
	class ConnectedHexagonGrid
	{
		std::vector<HexagonCell<CellData>> m_cells;
		std::queue<HexagonCellHandle> m_cellPool;
		int m_version = -1;
	public:
		
		[[nodiscard]] HexagonCellHandle AllocateAdjacent(HexagonCellHandle targetHandle, HexagonGridDirection direction);
	};

	template <typename CellData>
	HexagonCellHandle ConnectedHexagonGrid<CellData>::AllocateAdjacent(HexagonCellHandle targetHandle,
	                                                          const HexagonGridDirection direction)
	{
		HexagonCellHandle newCellHandle;
		if (m_cellPool.empty()) {
			auto newPipe = m_cells.emplace_back(m_cells.size());
			newCellHandle = newPipe.m_handle;
		}
		else {
			newCellHandle = m_cellPool.front();
			m_cellPool.pop();
		}
		auto& newCell = m_cells[newCellHandle];
		newCell.m_recycled = false;
		auto& originalCell = m_cells[targetHandle];
		m_version++;
		//Set adjacent handles.
		int remainingCheck = 5;
		switch (direction)
		{
		case HexagonGridDirection::UpLeft:
			//First set self.
			newCell.m_downRight = targetHandle;
			originalCell.m_upLeft = newCellHandle;
			if(originalCell.m_upRight != -1)
			{
				remainingCheck--;
				auto& cell1 = m_cells[originalCell.m_upRight];
				newCell.m_right = originalCell.m_upRight;
				cell1.m_left = newCellHandle;
				if(cell1.m_upLeft != -1)
				{
					remainingCheck--;
					auto& cell2 = m_cells[cell1.m_upLeft];
					newCell.m_upRight = cell1.m_upLeft;
					cell2.m_downLeft = newCellHandle;
					if(cell2.m_left != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell2.m_left];
						newCell.m_upLeft = cell2.m_left;
						cell3.m_downRight = newCellHandle;
						if(cell3.m_downLeft != -1)
						{
							remainingCheck--;
							auto& cell4 = m_cells[cell3.m_downLeft];
							newCell.m_left = cell3.m_downLeft;
							cell4.m_right = newCellHandle;
							if(cell4.m_downRight != -1)
							{
								remainingCheck--;
								auto& cell5 = m_cells[cell4.m_downRight];
								newCell.m_downLeft = cell4.m_downRight;
								cell5.m_upRight = newCellHandle;
							}
						}
					}
				}
			}
			if (remainingCheck > 0 && originalCell.m_left != -1)
			{
				remainingCheck--;
				auto& cell5 = m_cells[originalCell.m_left];
				newCell.m_downLeft = originalCell.m_left;
				cell5.m_upRight = newCellHandle;
				if (remainingCheck > 0 && cell5.m_upLeft != -1)
				{
					remainingCheck--;
					auto& cell4 = m_cells[cell5.m_upLeft];
					newCell.m_left = cell5.m_upLeft;
					cell4.m_right = newCellHandle;
					if (remainingCheck > 0 && cell4.m_upRight != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell4.m_upRight];
						newCell.m_upLeft = cell4.m_upRight;
						cell3.m_downRight = newCellHandle;
						if (remainingCheck > 0 && cell3.m_right != -1)
						{
							remainingCheck--;
							auto& cell2 = m_cells[cell3.m_right];
							newCell.m_upRight = cell3.m_right;
							cell2.m_downLeft = newCellHandle;
							if (remainingCheck > 0 && cell2.m_downRight != -1)
							{
								auto& cell1 = m_cells[cell2.m_downRight];
								newCell.m_right = cell2.m_downRight;
								cell1.m_left = newCellHandle;
							}
						}
					}
				}
			}
			break;
		case HexagonGridDirection::UpRight:
			newCell.m_downLeft = targetHandle;
			originalCell.m_upRight = newCellHandle;
			if (originalCell.m_right != -1)
			{
				remainingCheck--;
				auto& cell1 = m_cells[originalCell.m_right];
				newCell.m_downRight = originalCell.m_right;
				cell1.m_upLeft = newCellHandle;
				if (cell1.m_upRight != -1)
				{
					remainingCheck--;
					auto& cell2 = m_cells[cell1.m_upRight];
					newCell.m_right = cell1.m_upRight;
					cell2.m_left = newCellHandle;
					if (cell2.m_upLeft != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell2.m_upLeft];
						newCell.m_upRight = cell2.m_upLeft;
						cell3.m_downLeft = newCellHandle;
						if (cell3.m_left != -1)
						{
							remainingCheck--;
							auto& cell4 = m_cells[cell3.m_left];
							newCell.m_upLeft = cell3.m_left;
							cell4.m_downRight = newCellHandle;
							if (cell4.m_downLeft != -1)
							{
								remainingCheck--;
								auto& cell5 = m_cells[cell4.m_downLeft];
								newCell.m_left = cell4.m_downLeft;
								cell5.m_right = newCellHandle;
							}
						}
					}
				}
			}
			if (remainingCheck > 0 && originalCell.m_upLeft != -1)
			{
				remainingCheck--;
				auto& cell5 = m_cells[originalCell.m_upLeft];
				newCell.m_left = originalCell.m_upLeft;
				cell5.m_right = newCellHandle;
				if (remainingCheck > 0 && cell5.m_upRight != -1)
				{
					remainingCheck--;
					auto& cell4 = m_cells[cell5.m_upRight];
					newCell.m_upLeft = cell5.m_upRight;
					cell4.m_downRight = newCellHandle;
					if (remainingCheck > 0 && cell4.m_right != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell4.m_right];
						newCell.m_upRight = cell4.m_right;
						cell3.m_downLeft = newCellHandle;
						if (remainingCheck > 0 && cell3.m_downRight != -1)
						{
							remainingCheck--;
							auto& cell2 = m_cells[cell3.m_downRight];
							newCell.m_right = cell3.m_downRight;
							cell2.m_left = newCellHandle;
							if (remainingCheck > 0 && cell2.m_downLeft != -1)
							{
								auto& cell1 = m_cells[cell2.m_downLeft];
								newCell.m_downRight = cell2.m_downLeft;
								cell1.m_upLeft = newCellHandle;
							}
						}
					}
				}
			}

			break;
		case HexagonGridDirection::Right:
			newCell.m_left = targetHandle;
			originalCell.m_right = newCellHandle;
			if (originalCell.m_downRight != -1)
			{
				remainingCheck--;
				auto& cell1 = m_cells[originalCell.m_downRight];
				newCell.m_downLeft = originalCell.m_downRight;
				cell1.m_upRight = newCellHandle;
				if (cell1.m_right != -1)
				{
					remainingCheck--;
					auto& cell2 = m_cells[cell1.m_right];
					newCell.m_downRight = cell1.m_right;
					cell2.m_upLeft = newCellHandle;
					if (cell2.m_upRight != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell2.m_upRight];
						newCell.m_right = cell2.m_upRight;
						cell3.m_left = newCellHandle;
						if (cell3.m_upLeft != -1)
						{
							remainingCheck--;
							auto& cell4 = m_cells[cell3.m_upLeft];
							newCell.m_upRight = cell3.m_upLeft;
							cell4.m_downLeft = newCellHandle;
							if (cell4.m_left != -1)
							{
								remainingCheck--;
								auto& cell5 = m_cells[cell4.m_left];
								newCell.m_upLeft = cell4.m_left;
								cell5.m_downRight = newCellHandle;
							}
						}
					}
				}
			}
			if (remainingCheck > 0 && originalCell.m_upRight != -1)
			{
				remainingCheck--;
				auto& cell5 = m_cells[originalCell.m_upRight];
				newCell.m_upLeft = originalCell.m_upRight;
				cell5.m_downRight = newCellHandle;
				if (remainingCheck > 0 && cell5.m_right != -1)
				{
					remainingCheck--;
					auto& cell4 = m_cells[cell5.m_right];
					newCell.m_upRight = cell5.m_right;
					cell4.m_downLeft = newCellHandle;
					if (remainingCheck > 0 && cell4.m_downRight != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell4.m_downRight];
						newCell.m_right = cell4.m_downRight;
						cell3.m_left = newCellHandle;
						if (remainingCheck > 0 && cell3.m_downLeft != -1)
						{
							remainingCheck--;
							auto& cell2 = m_cells[cell3.m_downLeft];
							newCell.m_downRight = cell3.m_downLeft;
							cell2.m_upLeft = newCellHandle;
							if (remainingCheck > 0 && cell2.m_left != -1)
							{
								auto& cell1 = m_cells[cell2.m_left];
								newCell.m_downLeft = cell2.m_left;
								cell1.m_upRight = newCellHandle;
							}
						}
					}
				}
			}
			break;
		case HexagonGridDirection::DownRight:
			newCell.m_upLeft = targetHandle;
			originalCell.m_downRight = newCellHandle;
			if (originalCell.m_downLeft != -1)
			{
				remainingCheck--;
				auto& cell1 = m_cells[originalCell.m_downLeft];
				newCell.m_left = originalCell.m_downLeft;
				cell1.m_right = newCellHandle;
				if (cell1.m_downRight != -1)
				{
					remainingCheck--;
					auto& cell2 = m_cells[cell1.m_downRight];
					newCell.m_downLeft = cell1.m_downRight;
					cell2.m_upRight = newCellHandle;
					if (cell2.m_right != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell2.m_right];
						newCell.m_downRight = cell2.m_right;
						cell3.m_upLeft = newCellHandle;
						if (cell3.m_upRight != -1)
						{
							remainingCheck--;
							auto& cell4 = m_cells[cell3.m_upRight];
							newCell.m_right = cell3.m_upRight;
							cell4.m_left = newCellHandle;
							if (cell4.m_upLeft != -1)
							{
								remainingCheck--;
								auto& cell5 = m_cells[cell4.m_upLeft];
								newCell.m_upRight = cell4.m_upLeft;
								cell5.m_downLeft = newCellHandle;
							}
						}
					}
				}
			}
			if (remainingCheck > 0 && originalCell.m_right != -1)
			{
				remainingCheck--;
				auto& cell5 = m_cells[originalCell.m_right];
				newCell.m_upRight = originalCell.m_right;
				cell5.m_downLeft = newCellHandle;
				if (remainingCheck > 0 && cell5.m_downRight != -1)
				{
					remainingCheck--;
					auto& cell4 = m_cells[cell5.m_downRight];
					newCell.m_right = cell5.m_downRight;
					cell4.m_left = newCellHandle;
					if (remainingCheck > 0 && cell4.m_downLeft != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell4.m_downLeft];
						newCell.m_downRight = cell4.m_downLeft;
						cell3.m_upLeft = newCellHandle;
						if (remainingCheck > 0 && cell3.m_left != -1)
						{
							remainingCheck--;
							auto& cell2 = m_cells[cell3.m_left];
							newCell.m_downLeft = cell3.m_left;
							cell2.m_upRight = newCellHandle;
							if (remainingCheck > 0 && cell2.m_upLeft != -1)
							{
								auto& cell1 = m_cells[cell2.m_upLeft];
								newCell.m_left = cell2.m_upLeft;
								cell1.m_right = newCellHandle;
							}
						}
					}
				}
			}
			break;
		case HexagonGridDirection::DownLeft:
			newCell.m_upRight = targetHandle;
			originalCell.m_downLeft = newCellHandle;
			if (originalCell.m_left != -1)
			{
				remainingCheck--;
				auto& cell1 = m_cells[originalCell.m_left];
				newCell.m_upLeft = originalCell.m_left;
				cell1.m_downRight = newCellHandle;
				if (cell1.m_downLeft != -1)
				{
					remainingCheck--;
					auto& cell2 = m_cells[cell1.m_downLeft];
					newCell.m_left = cell1.m_downLeft;
					cell2.m_right = newCellHandle;
					if (cell2.m_downRight != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell2.m_downRight];
						newCell.m_downLeft = cell2.m_downRight;
						cell3.m_upRight = newCellHandle;
						if (cell3.m_right != -1)
						{
							remainingCheck--;
							auto& cell4 = m_cells[cell3.m_right];
							newCell.m_downRight = cell3.m_right;
							cell4.m_upLeft = newCellHandle;
							if (cell4.m_upRight != -1)
							{
								remainingCheck--;
								auto& cell5 = m_cells[cell4.m_upRight];
								newCell.m_right = cell4.m_upRight;
								cell5.m_left = newCellHandle;
							}
						}
					}
				}
			}
			if (remainingCheck > 0 && originalCell.m_downRight != -1)
			{
				remainingCheck--;
				auto& cell5 = m_cells[originalCell.m_downRight];
				newCell.m_left = originalCell.m_downRight;
				cell5.m_right = newCellHandle;
				if (remainingCheck > 0 && cell5.m_downLeft != -1)
				{
					remainingCheck--;
					auto& cell4 = m_cells[cell5.m_downLeft];
					newCell.m_downRight = cell5.m_downLeft;
					cell4.m_upLeft = newCellHandle;
					if (remainingCheck > 0 && cell4.m_left != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell4.m_left];
						newCell.m_downLeft = cell4.m_left;
						cell3.m_upRight = newCellHandle;
						if (remainingCheck > 0 && cell3.m_upLeft != -1)
						{
							remainingCheck--;
							auto& cell2 = m_cells[cell3.m_upLeft];
							newCell.m_left = cell3.m_upLeft;
							cell2.m_right = newCellHandle;
							if (remainingCheck > 0 && cell2.m_upRight != -1)
							{
								auto& cell1 = m_cells[cell2.m_upRight];
								newCell.m_upLeft = cell2.m_upRight;
								cell1.m_downRight = newCellHandle;
							}
						}
					}
				}
			}
			break;
		case HexagonGridDirection::Left:
			newCell.m_right = targetHandle;
			originalCell.m_left = newCellHandle;
			if (originalCell.m_upLeft != -1)
			{
				remainingCheck--;
				auto& cell1 = m_cells[originalCell.m_upLeft];
				newCell.m_upRight = originalCell.m_upLeft;
				cell1.m_downLeft = newCellHandle;
				if (cell1.m_right != -1)
				{
					remainingCheck--;
					auto& cell2 = m_cells[cell1.m_right];
					newCell.m_upLeft = cell1.m_right;
					cell2.m_downRight = newCellHandle;
					if (cell2.m_downLeft != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell2.m_downLeft];
						newCell.m_left = cell2.m_downLeft;
						cell3.m_right = newCellHandle;
						if (cell3.m_downRight != -1)
						{
							remainingCheck--;
							auto& cell4 = m_cells[cell3.m_downRight];
							newCell.m_downLeft = cell3.m_downRight;
							cell4.m_upRight = newCellHandle;
							if (cell4.m_right != -1)
							{
								remainingCheck--;
								auto& cell5 = m_cells[cell4.m_right];
								newCell.m_downRight = cell4.m_right;
								cell5.m_upLeft = newCellHandle;
							}
						}
					}
				}
			}
			if (remainingCheck > 0 && originalCell.m_downLeft != -1)
			{
				remainingCheck--;
				auto& cell5 = m_cells[originalCell.m_downLeft];
				newCell.m_downRight = originalCell.m_downLeft;
				cell5.m_upLeft = newCellHandle;
				if (remainingCheck > 0 && cell5.m_left != -1)
				{
					remainingCheck--;
					auto& cell4 = m_cells[cell5.m_left];
					newCell.m_downLeft = cell5.m_left;
					cell4.m_upRight = newCellHandle;
					if (remainingCheck > 0 && cell4.m_upLeft != -1)
					{
						remainingCheck--;
						auto& cell3 = m_cells[cell4.m_upLeft];
						newCell.m_left = cell4.m_upLeft;
						cell3.m_right = newCellHandle;
						if (remainingCheck > 0 && cell3.m_upRight != -1)
						{
							remainingCheck--;
							auto& cell2 = m_cells[cell3.m_upRight];
							newCell.m_upLeft = cell3.m_upRight;
							cell2.m_downRight = newCellHandle;
							if (remainingCheck > 0 && cell2.m_right != -1)
							{
								auto& cell1 = m_cells[cell2.m_right];
								newCell.m_upRight = cell2.m_right;
								cell1.m_downLeft = newCellHandle;
							}
						}
					}
				}
			}
			break;
		}
		return newCellHandle;
	}
}
