#pragma once

#include "Vertex.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{

	typedef int StrandHandle;
	typedef int StrandSegmentHandle;

	struct StrandSegmentInfo
	{
		/**
		 * \brief The position of the end of current strand segment.
		 */
		glm::vec3 m_globalPosition = glm::vec3(0.0f);
		/**
		 * \brief The rotation of the end of current strand segment.
		 */
		glm::quat m_globalRotation = glm::vec3(0.0f);
		/**
		 * \brief The thickness of the end of current strand segment.
		 */
		float m_thickness = 0.0f;
		glm::vec4 m_color = glm::vec4(1.0f);
		bool m_isBoundary = false;
	};

	struct StrandInfo
	{
		glm::vec4 m_color = glm::vec4(1.0f);

		/**
		 * \brief The info of the start of the first strand segment in this strand.
		 */
		StrandSegmentInfo m_baseInfo {};
	};

	/**
	 * \brief The data structure that holds a strand segment.
	 * \tparam StrandSegmentData The customizable data for each strand segment.
	 */
	template<typename StrandSegmentData>
	class StrandSegment
	{
		template<typename PGD, typename PD, typename PSD>
		friend class StrandGroup;
		template<typename SGD, typename SD, typename SSD>
		friend class StrandGroupSerializer;
		bool m_endSegment = true;
		bool m_recycled = false;
		StrandSegmentHandle m_prevHandle = -1;
		StrandSegmentHandle m_handle = -1;
		StrandSegmentHandle m_nextHandle = -1;

		StrandHandle m_strandHandle = -1;

		int m_index = -1;
	public:
		StrandSegmentData m_data {};
		StrandSegmentInfo m_info {};

		/**
		 * Whether this segment is the end segment.
		 * @return True if this is end segment, false else wise.
		 */
		[[nodiscard]] bool IsEnd() const;

		/**
		 * Whether this segment is recycled (removed).
		 * @return True if this segment is recycled (removed), false else wise.
		 */
		[[nodiscard]] bool IsRecycled() const;

		/**
		 * Get the handle of self.
		 * @return strandSegmentHandle of current segment.
		 */
		[[nodiscard]] StrandSegmentHandle GetHandle() const;

		/**
		 * Get the handle of belonged strand.
		 * @return strandHandle of current segment.
		 */
		[[nodiscard]] StrandHandle GetStrandHandle() const;
		/**
		 * Get the handle of prev segment.
		 * @return strandSegmentHandle of current segment.
		 */
		[[nodiscard]] StrandSegmentHandle GetPrevHandle() const;

		/**
		 * Get the handle of prev segment.
		 * @return strandSegmentHandle of current segment.
		 */
		[[nodiscard]] StrandSegmentHandle GetNextHandle() const;

		[[nodiscard]] int GetIndex() const;
		StrandSegment() = default;
		explicit StrandSegment(StrandHandle strandHandle, StrandSegmentHandle handle, StrandSegmentHandle prevHandle);
	};

	/**
	 * \brief The data structure that holds a strand.
	 * \tparam StrandData The customizable data for each strand.
	 */
	template<typename StrandData>
	class Strand
	{
		template<typename PGD, typename PD, typename PSD>
		friend class StrandGroup;
		template<typename SGD, typename SD, typename SSD>
		friend class StrandGroupSerializer;
		bool m_recycled = false;
		StrandHandle m_handle = -1;

		std::vector<StrandSegmentHandle> m_strandSegmentHandles;

	public:
		StrandData m_data;
		StrandInfo m_info;

		/**
		 * Whether this segment is recycled (removed).
		 * @return True if this segment is recycled (removed), false else wise.
		 */
		[[nodiscard]] bool IsRecycled() const;

		/**
		 * Get the handle of self.
		 * @return strandSegmentHandle of current segment.
		 */
		[[nodiscard]] StrandHandle GetHandle() const;

		/**
		 * Access the segments that belongs to this flow.
		 * @return The list of handles.
		 */
		[[nodiscard]] const std::vector<StrandSegmentHandle>& PeekStrandSegmentHandles() const;
		Strand() = default;
		explicit Strand(StrandHandle handle);
	};

	/**
	 * \brief The data structure that holds a collection of strands.
	 * \tparam StrandGroupData The customizable data for entire strand group.
	 * \tparam StrandData The customizable data for each strand.
	 * \tparam StrandSegmentData The customizable data for each strand segment.
	 */
	template<typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	class StrandGroup {
		template<typename SGD, typename SD, typename SSD>
		friend class StrandGroupSerializer;
		std::vector<Strand<StrandData>> m_strands;
		std::vector<StrandSegment<StrandSegmentData>> m_strandSegments;

		std::queue<StrandHandle> m_strandPool;
		std::queue<StrandSegmentHandle> m_strandSegmentPool;

		int m_version = -1;
		void BuildStrand(float controlPointRatio, const Strand<StrandData>& strand, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, bool triplePoints, int nodeMaxCount) const;

		[[nodiscard]] StrandSegmentHandle AllocateStrandSegment(StrandHandle strandHandle, StrandSegmentHandle prevHandle, int index);
	public:

		void BuildStrands(float controlPointRatio, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, bool triplePoints, int nodeMaxCount) const;

		StrandGroupData m_data;

		[[nodiscard]] StrandHandle AllocateStrand();

		/**
		 * Extend strand during growth process. The flow structure will also be updated.
		 * @param targetHandle The handle of the segment to branch/prolong
		 * @return The handle of new segment.
		 */
		[[nodiscard]] StrandSegmentHandle Extend(StrandHandle targetHandle);

		/**
		 * Insert strand segment during growth process. The flow structure will also be updated.
		 * @param targetHandle The handle of the strand to be inserted.
		 * @param targetSegmentHandle The handle of the strand segment to be inserted. If there's no subsequent segment this will be a simple extend.
		 * @return The handle of new segment.
		 */
		[[nodiscard]] StrandSegmentHandle Insert(StrandHandle targetHandle, StrandSegmentHandle targetSegmentHandle);

		/**
		 * Recycle (Remove) a segment, the descendents of this segment will also be recycled. The relevant flow will also be removed/restructured.
		 * @param handle The handle of the segment to be removed. Must be valid (non-zero and the segment should not be recycled prior to this operation).
		 */
		void RecycleStrandSegment(StrandSegmentHandle handle);

		/**
		 * Recycle (Remove) a strand. The relevant segment will also be removed/restructured.
		 * @param handle The handle of the strand to be removed. Must be valid (non-zero and the flow should not be recycled prior to this operation).
		 */
		void RecycleStrand(StrandHandle handle);

		/**
		 * \brief Get a unmodifiable reference to all strands.
		 * \return A constant reference to all strands.
		 */
		[[nodiscard]] const std::vector<Strand<StrandData>>& PeekStrands() const;
		/**
		 * \brief Get a unmodifiable reference to all strand segments.
		 * \return A constant reference to all strand segments.
		 */
		[[nodiscard]] const std::vector<StrandSegment<StrandSegmentData>>& PeekStrandSegments() const;
		/**
		 * \brief Get a reference to all strands.
		 * \return A reference to all strands.
		 */
		[[nodiscard]] std::vector<Strand<StrandData>>& RefStrands();
		/**
		 * \brief Get a reference to all strand segments.
		 * \return A reference to all strand segments.
		 */
		[[nodiscard]] std::vector<StrandSegment<StrandSegmentData>>& RefStrandSegments();
		/**
		 * \brief Get a reference to a specific strand.
		 * \param handle The handle of the strand.
		 * \return A reference to the target strand.
		 */
		[[nodiscard]] Strand<StrandData>& RefStrand(StrandHandle handle);
		/**
		 * \brief Get a reference to a specific strand segment.
		 * \param handle The handle of the strand segment.
		 * \return A reference to the target strand segment.
		 */
		[[nodiscard]] StrandSegment<StrandSegmentData>& RefStrandSegment(StrandSegmentHandle handle);
		/**
		 * \brief Get a unmodifiable reference to a specific strand.
		 * \param handle The handle of the strand.
		 * \return A unmodifiable reference to the target strand.
		 */
		[[nodiscard]] const Strand<StrandData>& PeekStrand(StrandHandle handle) const;
		/**
		 * \brief Get a unmodifiable reference to a specific strand.
		 * \param handle The handle of the strand.
		 * \return A unmodifiable reference to the target strand.
		 */
		[[nodiscard]] const StrandSegment<StrandSegmentData>& PeekStrandSegment(StrandSegmentHandle handle) const;

		/**
		 * Get the structural version of the tree. The version will change when the tree structure changes.
		 * @return The version
		 */
		[[nodiscard]] int GetVersion() const;

		[[nodiscard]] glm::vec3 GetStrandSegmentStart(StrandSegmentHandle handle) const;
	};

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::BuildStrand(const float controlPointRatio, const Strand<StrandData>& strand,
		std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, bool triplePoints, int nodeMaxCount) const
	{
		const auto& strandSegmentHandles = strand.PeekStrandSegmentHandles();
		if (strandSegmentHandles.empty())
			return;
		if(triplePoints)
		{
			auto& baseInfo = strand.m_info.m_baseInfo;
			strands.emplace_back(points.size());
			StrandPoint basePoint;
			const auto& secondStrandSegment = PeekStrandSegment(strandSegmentHandles[0]);
			auto basePointDistance = glm::distance(baseInfo.m_globalPosition, secondStrandSegment.m_info.m_globalPosition);
			const auto baseTangent = glm::normalize(baseInfo.m_globalRotation * glm::vec3(0, 0, -1));
			basePoint.m_color = baseInfo.m_color;
			basePoint.m_position = baseInfo.m_globalPosition - baseTangent * basePointDistance * controlPointRatio;
			basePoint.m_thickness = baseInfo.m_thickness;
			points.emplace_back(basePoint);
			basePoint.m_position = baseInfo.m_globalPosition;
			points.emplace_back(basePoint);
			basePoint.m_position = baseInfo.m_globalPosition + baseTangent * basePointDistance * controlPointRatio;
			points.emplace_back(basePoint);

			StrandPoint point;
			for (int i = 0; i < strandSegmentHandles.size() && (nodeMaxCount == -1 || i < nodeMaxCount); i++)
			{
				const auto& strandSegment = PeekStrandSegment(strandSegmentHandles[i]);
				glm::vec3 prevPosition;
				glm::vec3 nextPosition;
				glm::vec3 tangent;
				if (i == 0)
				{
					prevPosition = baseInfo.m_globalPosition;
				}
				else {
					prevPosition = PeekStrandSegment(strandSegmentHandles[i - 1]).m_info.m_globalPosition;
				}

				if(i == strandSegmentHandles.size() - 1)
				{
					nextPosition = 2.0f * strandSegment.m_info.m_globalPosition - prevPosition;
				}else
				{
					nextPosition = PeekStrandSegment(strandSegmentHandles[i + 1]).m_info.m_globalPosition;
				}
				tangent = glm::normalize(nextPosition - prevPosition);
				auto prevDistance = glm::distance(strandSegment.m_info.m_globalPosition, prevPosition);
				auto nextDistance = glm::distance(strandSegment.m_info.m_globalPosition, nextPosition);
				auto distance = glm::min(prevDistance, nextDistance);
				point.m_color = strandSegment.m_info.m_color;
				point.m_position = strandSegment.m_info.m_globalPosition - tangent * distance * controlPointRatio;
				point.m_thickness = strandSegment.m_info.m_thickness;
				points.emplace_back(point);

				point.m_position = strandSegment.m_info.m_globalPosition;
				points.emplace_back(point);

				point.m_position = strandSegment.m_info.m_globalPosition + tangent * distance * controlPointRatio;
				points.emplace_back(point);
			}

		}
		else {
			auto& baseInfo = strand.m_info.m_baseInfo;
			const auto startIndex = points.size();
			strands.emplace_back(startIndex);
			StrandPoint basePoint;
			basePoint.m_color = baseInfo.m_color;
			basePoint.m_thickness = baseInfo.m_thickness;
			basePoint.m_position = baseInfo.m_globalPosition;

			points.emplace_back(basePoint);
			points.emplace_back(basePoint);

			StrandPoint point;
			for (int i = 0; i < strandSegmentHandles.size() && (nodeMaxCount == -1 || i < nodeMaxCount); i++)
			{
				const auto& strandSegment = PeekStrandSegment(strandSegmentHandles[i]);
				//auto distance = glm::min(prevDistance, nextDistance);
				point.m_color = strandSegment.m_info.m_color;
				point.m_thickness = strandSegment.m_info.m_thickness;
				point.m_position = strandSegment.m_info.m_globalPosition;
				points.emplace_back(point);
			}
			auto& backPoint = points.at(points.size() - 2);
			auto& lastPoint = points.at(points.size() - 1);

			point.m_color = 2.0f * lastPoint.m_color - backPoint.m_color;
			point.m_thickness = 2.0f * lastPoint.m_thickness - backPoint.m_thickness;
			point.m_position = 2.0f * lastPoint.m_position - backPoint.m_position;
			points.emplace_back(point);

			auto& firstPoint = points.at(startIndex);
			auto& secondPoint = points.at(startIndex + 1);
			auto& thirdPoint = points.at(startIndex + 2);
			firstPoint.m_color = 2.0f * secondPoint.m_color - thirdPoint.m_color;
			firstPoint.m_thickness = 2.0f * secondPoint.m_thickness - thirdPoint.m_thickness;
			firstPoint.m_position = 2.0f * secondPoint.m_position - thirdPoint.m_position;
		}
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::AllocateStrandSegment(StrandHandle strandHandle, StrandSegmentHandle prevHandle, const int index)
	{
		StrandSegmentHandle newSegmentHandle;
		if (m_strandSegmentPool.empty()) {
			m_strandSegments.emplace_back(strandHandle, m_strandSegments.size(), prevHandle);
			newSegmentHandle = m_strandSegments.back().m_handle;
		}
		else {
			newSegmentHandle = m_strandSegmentPool.front();
			m_strandSegmentPool.pop();
		}
		auto& segment = m_strandSegments[newSegmentHandle];
		if (prevHandle != -1) {
			m_strandSegments[prevHandle].m_nextHandle = newSegmentHandle;
			m_strandSegments[prevHandle].m_endSegment = false;
			segment.m_prevHandle = prevHandle;
		}
		segment.m_nextHandle = -1;
		segment.m_strandHandle = strandHandle;
		segment.m_index = index;
		segment.m_recycled = false;
		return newSegmentHandle;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::BuildStrands(const float controlPointRatio, std::vector<glm::uint>& strands,
		std::vector<StrandPoint>& points, bool triplePoints, int nodeMaxCount) const
	{
		for (const auto& strand : m_strands)
		{
			if (strand.IsRecycled()) continue;
			BuildStrand(controlPointRatio, strand, strands, points, triplePoints, nodeMaxCount);
		}
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	StrandHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::AllocateStrand()
	{
		if (m_strandPool.empty()) {
			m_strands.emplace_back(m_strands.size());
			m_version++;
			return m_strands.back().m_handle;
		}
		auto handle = m_strandPool.front();
		m_strandPool.pop();
		auto& strand = m_strands[handle];
		strand.m_recycled = false;
		m_version++;
		return handle;
	}
	

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::Extend(StrandHandle targetHandle)
	{
		auto& strand = m_strands[targetHandle];
		assert(!strand.m_recycled);
		auto prevHandle = -1;
		if (!strand.m_strandSegmentHandles.empty()) prevHandle = strand.m_strandSegmentHandles.back();
		const auto newSegmentHandle = AllocateStrandSegment(targetHandle, prevHandle, strand.m_strandSegmentHandles.size());
		strand.m_strandSegmentHandles.emplace_back(newSegmentHandle);
		auto& segment = m_strandSegments[newSegmentHandle];
		segment.m_endSegment = true;
		m_version++;
		return newSegmentHandle;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	StrandSegmentHandle StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::Insert(StrandHandle targetHandle, StrandSegmentHandle targetSegmentHandle)
	{
		auto& strand = m_strands[targetHandle];
		assert(!strand.m_recycled);
		auto& prevSegment = m_strandSegments[targetSegmentHandle];
		const auto prevSegmentIndex = prevSegment.m_index;
		const auto nextSegmentHandle = strand.m_strandSegmentHandles[prevSegmentIndex + 1];
		if (strand.m_strandSegmentHandles.size() - 1 == prevSegmentIndex) return Extend(targetHandle);
		const auto newSegmentHandle = AllocateStrandSegment(targetHandle, targetSegmentHandle, prevSegmentIndex + 1);
		auto& newSegment = m_strandSegments[newSegmentHandle];
		newSegment.m_endSegment = false;
		newSegment.m_nextHandle = nextSegmentHandle;
		auto& nextSegment = m_strandSegments[nextSegmentHandle];
		nextSegment.m_prevHandle = newSegmentHandle;
		strand.m_strandSegmentHandles.insert(strand.m_strandSegmentHandles.begin() + prevSegmentIndex + 1, newSegmentHandle);
		for(int i = prevSegmentIndex + 2; i < strand.m_strandSegmentHandles.size(); ++i)
		{
			m_strandSegments[strand.m_strandSegmentHandles[i]].m_index = i;
		}
		m_version++;
		return newSegmentHandle;
	}


	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RecycleStrandSegment(StrandSegmentHandle handle)
	{
		//Recycle subsequent segments from strand.
		auto& segment = m_strandSegments[handle];
		assert(!segment.m_recycled);
		if (segment.m_nextHandle != -1)
		{
			RecycleStrandSegment(segment.m_nextHandle);
		}
		if (segment.m_prevHandle != -1)
		{
			m_strandSegments[segment.m_prevHandle].m_nextHandle = -1;
			m_strandSegments[segment.m_prevHandle].m_endSegment = true;
		}

		auto& strand = m_strands[segment.m_strandHandle];
		assert(strand.m_strandSegmentHandles.back() == handle);
		strand.m_strandSegmentHandles.pop_back();

		segment.m_recycled = true;
		segment.m_endSegment = true;
		segment.m_prevHandle = segment.m_nextHandle = -1;
		segment.m_data = {};
		segment.m_info = {};

		segment.m_index = -1;
		segment.m_strandHandle = -1;
		m_strandSegmentPool.emplace(handle);
		m_version++;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	void StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RecycleStrand(StrandHandle handle)
	{
		//Recycle all segments;
		auto& strand = m_strands[handle];
		assert(!strand.m_recycled);
		for (const auto& segmentHandle : strand.m_strandSegmentHandles)
		{
			auto& segment = m_strandSegments[segmentHandle];
			segment.m_recycled = true;
			segment.m_endSegment = true;
			segment.m_prevHandle = segment.m_nextHandle = -1;
			segment.m_data = {};
			segment.m_info = {};

			segment.m_index = -1;
			segment.m_strandHandle = -1;
			m_strandSegmentPool.emplace(segmentHandle);
		}
		strand.m_strandSegmentHandles.clear();

		//Recycle strand.
		strand.m_recycled = true;
		strand.m_data = {};
		strand.m_info = {};
		m_strandPool.emplace(handle);
		m_version++;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	const std::vector<Strand<StrandData>>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrands() const
	{
		return m_strands;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	const std::vector<StrandSegment<StrandSegmentData>>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegments() const
	{
		return m_strandSegments;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	std::vector<Strand<StrandData>>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrands()
	{
		return m_strands;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	std::vector<StrandSegment<StrandSegmentData>>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegments()
	{
		return m_strandSegments;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	Strand<StrandData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrand(StrandHandle handle)
	{
		return m_strands[handle];
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	StrandSegment<StrandSegmentData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::RefStrandSegment(StrandSegmentHandle handle)
	{
		return m_strandSegments[handle];
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	const Strand<StrandData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrand(StrandHandle handle) const
	{
		return m_strands[handle];
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	const StrandSegment<StrandSegmentData>& StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::PeekStrandSegment(
		StrandSegmentHandle handle) const
	{
		return m_strandSegments[handle];
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	int StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetVersion() const
	{
		return m_version;
	}

	template <typename StrandGroupData, typename StrandData, typename StrandSegmentData>
	glm::vec3 StrandGroup<StrandGroupData, StrandData, StrandSegmentData>::GetStrandSegmentStart(
		StrandSegmentHandle handle) const
	{
		const auto& segment = m_strandSegments[handle];
		glm::vec3 segmentStart;
		if (segment.GetPrevHandle() != -1)
		{
			segmentStart = m_strandSegments[segment.GetPrevHandle()].m_info.m_globalPosition;
		}
		else
		{
			segmentStart = m_strands[segment.GetStrandHandle()].m_info.m_baseInfo.m_globalPosition;
		}
		return segmentStart;
	}

	template <typename StrandSegmentData>
	bool StrandSegment<StrandSegmentData>::IsEnd() const
	{
		return m_endSegment;
	}

	template <typename StrandSegmentData>
	bool StrandSegment<StrandSegmentData>::IsRecycled() const
	{
		return m_recycled;
	}

	template <typename StrandSegmentData>
	StrandSegmentHandle StrandSegment<StrandSegmentData>::GetHandle() const
	{
		return m_handle;
	}

	template <typename StrandSegmentData>
	StrandHandle StrandSegment<StrandSegmentData>::GetStrandHandle() const
	{
		return m_strandHandle;
	}

	template <typename StrandSegmentData>
	StrandSegmentHandle StrandSegment<StrandSegmentData>::GetPrevHandle() const
	{
		return m_prevHandle;
	}

	template <typename StrandSegmentData>
	StrandSegmentHandle StrandSegment<StrandSegmentData>::GetNextHandle() const
	{
		return m_nextHandle;
	}

	template <typename StrandSegmentData>
	int StrandSegment<StrandSegmentData>::GetIndex() const
	{
		return m_index;
	}

	template <typename StrandSegmentData>
	StrandSegment<StrandSegmentData>::StrandSegment(const StrandHandle strandHandle, const StrandSegmentHandle handle, const StrandSegmentHandle prevHandle)
	{
		m_strandHandle = strandHandle;
		m_handle = handle;
		m_prevHandle = prevHandle;
		m_nextHandle = -1;
		m_recycled = false;
		m_endSegment = true;

		m_index = -1;
		m_data = {};
		m_info = {};
	}

	template <typename StrandData>
	bool Strand<StrandData>::IsRecycled() const
	{
		return m_recycled;
	}

	template <typename StrandData>
	StrandHandle Strand<StrandData>::GetHandle() const
	{
		return m_handle;
	}

	template <typename StrandData>
	const std::vector<StrandSegmentHandle>& Strand<StrandData>::PeekStrandSegmentHandles() const
	{
		return m_strandSegmentHandles;
	}

	template <typename StrandData>
	Strand<StrandData>::Strand(const StrandHandle handle)
	{
		m_handle = handle;
		m_recycled = false;

		m_strandSegmentHandles.clear();

		m_data = {};
		m_info = {};
	}
}
