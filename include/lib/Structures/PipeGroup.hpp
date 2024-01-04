#pragma once

#include "Vertex.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{

	typedef int PipeHandle;
	typedef int PipeSegmentHandle;

	struct PipeSegmentInfo
	{
		/**
		 * \brief The position of the end of current pipe segment.
		 */
		glm::vec3 m_globalPosition = glm::vec3(0.0f);
		/**
		 * \brief The rotation of the end of current pipe segment.
		 */
		glm::quat m_globalRotation = glm::vec3(0.0f);
		/**
		 * \brief The thickness of the end of current pipe segment.
		 */
		float m_thickness = 0.0f;
	};

	struct PipeInfo
	{
		glm::vec4 m_color = glm::vec4(1.0f);

		/**
		 * \brief The info of the start of the first pipe segment in this pipe.
		 */
		PipeSegmentInfo m_baseInfo;
	};

	/**
	 * \brief The data structure that holds a pipe segment.
	 * \tparam PipeSegmentData The customizable data for each pipe segment.
	 */
	template<typename PipeSegmentData>
	class PipeSegment
	{
		template<typename PGD, typename PD, typename PSD>
		friend class PipeGroup;

		bool m_endSegment = true;
		bool m_recycled = false;
		PipeSegmentHandle m_prevHandle = -1;
		PipeSegmentHandle m_handle = -1;
		PipeSegmentHandle m_nextHandle = -1;

		PipeHandle m_pipeHandle = -1;

		int m_index = -1;
	public:
		PipeSegmentData m_data;
		PipeSegmentInfo m_info;

		/**
		 * Whether this segment is the end segment.
		 * @return True if this is end segment, false else wise.
		 */
		[[nodiscard]] bool IsEndPipeSegment() const;

		/**
		 * Whether this segment is recycled (removed).
		 * @return True if this segment is recycled (removed), false else wise.
		 */
		[[nodiscard]] bool IsRecycled() const;

		/**
		 * Get the handle of self.
		 * @return PipeSegmentHandle of current segment.
		 */
		[[nodiscard]] PipeSegmentHandle GetHandle() const;

		/**
		 * Get the handle of belonged pipe.
		 * @return PipeHandle of current segment.
		 */
		[[nodiscard]] PipeHandle GetPipeHandle() const;
		/**
		 * Get the handle of prev segment.
		 * @return PipeSegmentHandle of current segment.
		 */
		[[nodiscard]] PipeSegmentHandle GetPrevHandle() const;

		/**
		 * Get the handle of prev segment.
		 * @return PipeSegmentHandle of current segment.
		 */
		[[nodiscard]] PipeSegmentHandle GetNextHandle() const;

		[[nodiscard]] int GetIndex() const;

		explicit PipeSegment(PipeHandle pipeHandle, PipeSegmentHandle handle, PipeSegmentHandle prevHandle);
	};

	/**
	 * \brief The data structure that holds a pipe.
	 * \tparam PipeData The customizable data for each pipe.
	 */
	template<typename PipeData>
	class Pipe
	{
		template<typename PGD, typename PD, typename PSD>
		friend class PipeGroup;

		bool m_recycled = false;
		PipeHandle m_handle = -1;

		std::vector<PipeSegmentHandle> m_pipeSegmentHandles;

	public:
		PipeData m_data;
		PipeInfo m_info;

		/**
		 * Whether this segment is recycled (removed).
		 * @return True if this segment is recycled (removed), false else wise.
		 */
		[[nodiscard]] bool IsRecycled() const;

		/**
		 * Get the handle of self.
		 * @return PipeSegmentHandle of current segment.
		 */
		[[nodiscard]] PipeHandle GetHandle() const;

		/**
		 * Access the segments that belongs to this flow.
		 * @return The list of handles.
		 */
		[[nodiscard]] const std::vector<PipeSegmentHandle>& PeekPipeSegmentHandles() const;

		explicit Pipe(PipeHandle handle);
	};

	/**
	 * \brief The data structure that holds a collection of pipes.
	 * \tparam PipeGroupData The customizable data for entire pipe group.
	 * \tparam PipeData The customizable data for each pipe.
	 * \tparam PipeSegmentData The customizable data for each pipe segment.
	 */
	template<typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	class PipeGroup {

		std::vector<Pipe<PipeData>> m_pipes;
		std::vector<PipeSegment<PipeSegmentData>> m_pipeSegments;

		std::queue<PipeHandle> m_pipePool;
		std::queue<PipeSegmentHandle> m_pipeSegmentPool;

		int m_version = -1;
		void BuildStrand(float frontControlPointRatio, float backControlPointRatio, const Pipe<PipeData>& pipe, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, int nodeMaxCount) const;

		[[nodiscard]] PipeSegmentHandle AllocatePipeSegment(PipeHandle pipeHandle, PipeSegmentHandle prevHandle, int index);
	public:

		void BuildStrands(float frontControlPointRatio, float backControlPointRatio, std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, int nodeMaxCount = -1) const;

		PipeGroupData m_data;

		[[nodiscard]] PipeHandle AllocatePipe();

		/**
		 * Extend pipe during growth process. The flow structure will also be updated.
		 * @param targetHandle The handle of the segment to branch/prolong
		 * @return The handle of new segment.
		 */
		[[nodiscard]] PipeSegmentHandle Extend(PipeHandle targetHandle);

		/**
		 * Insert pipe segment during growth process. The flow structure will also be updated.
		 * @param targetHandle The handle of the pipe to be inserted.
		 * @param targetSegmentHandle The handle of the pipe segment to be inserted. If there's no subsequent segment this will be a simple extend.
		 * @return The handle of new segment.
		 */
		[[nodiscard]] PipeSegmentHandle Insert(PipeHandle targetHandle, PipeSegmentHandle targetSegmentHandle);

		/**
		 * Recycle (Remove) a segment, the descendents of this segment will also be recycled. The relevant flow will also be removed/restructured.
		 * @param handle The handle of the segment to be removed. Must be valid (non-zero and the segment should not be recycled prior to this operation).
		 */
		void RecyclePipeSegment(PipeSegmentHandle handle);

		/**
		 * Recycle (Remove) a pipe. The relevant segment will also be removed/restructured.
		 * @param handle The handle of the pipe to be removed. Must be valid (non-zero and the flow should not be recycled prior to this operation).
		 */
		void RecyclePipe(PipeHandle handle);

		/**
		 * \brief Get a unmodifiable reference to all pipes.
		 * \return A constant reference to all pipes.
		 */
		[[nodiscard]] const std::vector<Pipe<PipeData>>& PeekPipes() const;
		/**
		 * \brief Get a unmodifiable reference to all pipe segments.
		 * \return A constant reference to all pipe segments.
		 */
		[[nodiscard]] const std::vector<PipeSegment<PipeSegmentData>>& PeekPipeSegments() const;
		/**
		 * \brief Get a reference to all pipes.
		 * \return A reference to all pipes.
		 */
		[[nodiscard]] std::vector<Pipe<PipeData>>& RefPipes();
		/**
		 * \brief Get a reference to all pipe segments.
		 * \return A reference to all pipe segments.
		 */
		[[nodiscard]] std::vector<PipeSegment<PipeSegmentData>>& RefPipeSegments();
		/**
		 * \brief Get a reference to a specific pipe.
		 * \param handle The handle of the pipe.
		 * \return A reference to the target pipe.
		 */
		[[nodiscard]] Pipe<PipeData>& RefPipe(PipeHandle handle);
		/**
		 * \brief Get a reference to a specific pipe segment.
		 * \param handle The handle of the pipe segment.
		 * \return A reference to the target pipe segment.
		 */
		[[nodiscard]] PipeSegment<PipeSegmentData>& RefPipeSegment(PipeSegmentHandle handle);
		/**
		 * \brief Get a unmodifiable reference to a specific pipe.
		 * \param handle The handle of the pipe.
		 * \return A unmodifiable reference to the target pipe.
		 */
		[[nodiscard]] const Pipe<PipeData>& PeekPipe(PipeHandle handle) const;
		/**
		 * \brief Get a unmodifiable reference to a specific pipe.
		 * \param handle The handle of the pipe.
		 * \return A unmodifiable reference to the target pipe.
		 */
		[[nodiscard]] const PipeSegment<PipeSegmentData>& PeekPipeSegment(PipeSegmentHandle handle) const;

		/**
		 * Get the structural version of the tree. The version will change when the tree structure changes.
		 * @return The version
		 */
		[[nodiscard]] int GetVersion() const;
	};

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	void PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::BuildStrand(const float frontControlPointRatio, const float backControlPointRatio, const Pipe<PipeData>& pipe,
		std::vector<glm::uint>& strands, std::vector<StrandPoint>& points, int nodeMaxCount) const
	{
		const auto& pipeSegmentHandles = pipe.PeekPipeSegmentHandles();
		if (pipeSegmentHandles.empty())
			return;
		auto& baseInfo = pipe.m_info.m_baseInfo;
		strands.emplace_back(points.size());
		StrandPoint basePoint;
		basePoint.m_color = glm::vec4(0.6f, 0.3f, 0.0f, 1.0f);
		const auto& secondPipeSegment = PeekPipeSegment(pipeSegmentHandles[0]);
		auto basePointDistance = glm::distance(baseInfo.m_globalPosition, secondPipeSegment.m_info.m_globalPosition);
		basePoint.m_normal = glm::normalize(baseInfo.m_globalRotation * glm::vec3(0, 0, -1));
		basePoint.m_position = baseInfo.m_globalPosition - basePoint.m_normal * basePointDistance * frontControlPointRatio;
		basePoint.m_thickness = baseInfo.m_thickness;
		points.emplace_back(basePoint);
		basePoint.m_position = baseInfo.m_globalPosition;
		points.emplace_back(basePoint);
		basePoint.m_position = baseInfo.m_globalPosition + basePoint.m_normal * basePointDistance * backControlPointRatio;
		points.emplace_back(basePoint);

		StrandPoint point;
		point.m_color = glm::vec4(0.6f, 0.3f, 0.0f, 1.0f);
		{
			const auto& pipeSegment = PeekPipeSegment(pipeSegmentHandles[0]);
			
			glm::vec3 nextPosition;
			if (pipeSegmentHandles.size() > 1)
			{
				nextPosition = PeekPipeSegment(pipeSegmentHandles[1]).m_info.m_globalPosition;
			}else
			{
				nextPosition = pipeSegment.m_info.m_globalPosition;
			}
			auto prevDistance = glm::distance(pipeSegment.m_info.m_globalPosition, baseInfo.m_globalPosition);
			auto nextDistance = glm::distance(pipeSegment.m_info.m_globalPosition, nextPosition);
			auto distance = glm::min(prevDistance, nextDistance);

			point.m_normal = glm::normalize(pipeSegment.m_info.m_globalPosition - baseInfo.m_globalPosition); //glm::normalize(pipeSegment.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeSegment.m_info.m_globalPosition - point.m_normal * distance * frontControlPointRatio;
			point.m_thickness = pipeSegment.m_info.m_thickness;
			points.emplace_back(point);

			point.m_normal = glm::normalize(pipeSegment.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeSegment.m_info.m_globalPosition;
			points.emplace_back(point);

			point.m_normal = glm::normalize(nextPosition - pipeSegment.m_info.m_globalPosition); //glm::normalize(pipeSegment.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeSegment.m_info.m_globalPosition + point.m_normal * distance * backControlPointRatio;
			points.emplace_back(point);
		}
		for (int i = 1; i < pipeSegmentHandles.size() && (nodeMaxCount == -1 || i < nodeMaxCount); i++)
		{
			const auto& pipeSegment = PeekPipeSegment(pipeSegmentHandles[i]);
			const auto& prevPipeSegment = PeekPipeSegment(pipeSegmentHandles[i - 1]);
			glm::vec3 nextPosition;
			if (pipeSegmentHandles.size() > i + 1)
			{
				nextPosition = PeekPipeSegment(pipeSegmentHandles[i + 1]).m_info.m_globalPosition;
			}
			else
			{
				nextPosition = pipeSegment.m_info.m_globalPosition;
			}
			auto prevDistance = glm::distance(pipeSegment.m_info.m_globalPosition, prevPipeSegment.m_info.m_globalPosition);
			auto nextDistance = glm::distance(pipeSegment.m_info.m_globalPosition, nextPosition);
			auto distance = glm::min(prevDistance, nextDistance);

			point.m_normal = glm::normalize(pipeSegment.m_info.m_globalPosition - prevPipeSegment.m_info.m_globalPosition); //glm::normalize(pipeSegment.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeSegment.m_info.m_globalPosition - point.m_normal * distance * frontControlPointRatio;
			point.m_thickness = pipeSegment.m_info.m_thickness;
			points.emplace_back(point);

			point.m_normal = glm::normalize(pipeSegment.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeSegment.m_info.m_globalPosition;
			points.emplace_back(point);

			point.m_normal = glm::normalize(nextPosition - pipeSegment.m_info.m_globalPosition); //glm::normalize(pipeSegment.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			point.m_position = pipeSegment.m_info.m_globalPosition + point.m_normal * distance * backControlPointRatio;
			points.emplace_back(point);
		}
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	PipeSegmentHandle PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::AllocatePipeSegment(PipeHandle pipeHandle, PipeSegmentHandle prevHandle, const int index)
	{
		PipeSegmentHandle newSegmentHandle;
		if (m_pipeSegmentPool.empty()) {
			auto newSegment = m_pipeSegments.emplace_back(pipeHandle, m_pipeSegments.size(), prevHandle);
			newSegmentHandle = newSegment.m_handle;
		}
		else {
			newSegmentHandle = m_pipeSegmentPool.front();
			m_pipeSegmentPool.pop();
		}
		auto& segment = m_pipeSegments[newSegmentHandle];
		if (prevHandle != -1) {
			m_pipeSegments[prevHandle].m_nextHandle = newSegmentHandle;
			m_pipeSegments[prevHandle].m_endSegment = false;
			segment.m_prevHandle = prevHandle;
		}
		segment.m_nextHandle = -1;
		segment.m_pipeHandle = pipeHandle;
		segment.m_index = index;
		segment.m_recycled = false;
		return newSegmentHandle;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	void PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::BuildStrands(const float frontControlPointRatio, const float backControlPointRatio, std::vector<glm::uint>& strands,
		std::vector<StrandPoint>& points, int nodeMaxCount) const
	{
		for (const auto& pipe : m_pipes)
		{
			if (pipe.IsRecycled()) continue;
			BuildStrand(frontControlPointRatio, backControlPointRatio, pipe, strands, points, nodeMaxCount);
		}
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	PipeHandle PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::AllocatePipe()
	{
		if (m_pipePool.empty()) {
			auto newPipe = m_pipes.emplace_back(m_pipes.size());
			m_version++;
			return newPipe.m_handle;
		}
		auto handle = m_pipePool.front();
		m_pipePool.pop();
		auto& pipe = m_pipes[handle];
		pipe.m_recycled = false;
		m_version++;
		return handle;
	}
	

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	PipeSegmentHandle PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::Extend(PipeHandle targetHandle)
	{
		auto& pipe = m_pipes[targetHandle];
		assert(!pipe.m_recycled);
		auto prevHandle = -1;
		if (!pipe.m_pipeSegmentHandles.empty()) prevHandle = pipe.m_pipeSegmentHandles.back();
		const auto newSegmentHandle = AllocatePipeSegment(targetHandle, prevHandle, pipe.m_pipeSegmentHandles.size());
		pipe.m_pipeSegmentHandles.emplace_back(newSegmentHandle);
		auto& segment = m_pipeSegments[newSegmentHandle];
		segment.m_endSegment = true;
		m_version++;
		return newSegmentHandle;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	PipeSegmentHandle PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::Insert(PipeHandle targetHandle, PipeSegmentHandle targetSegmentHandle)
	{
		auto& pipe = m_pipes[targetHandle];
		assert(!pipe.m_recycled);
		auto& prevSegment = m_pipeSegments[targetSegmentHandle];
		const auto prevSegmentIndex = prevSegment.m_index;
		const auto nextSegmentHandle = pipe.m_pipeSegmentHandles[prevSegmentIndex + 1];
		if (pipe.m_pipeSegmentHandles.size() - 1 == prevSegmentIndex) return Extend(targetHandle);
		const auto newSegmentHandle = AllocatePipeSegment(targetHandle, targetSegmentHandle, prevSegmentIndex + 1);
		auto& newSegment = m_pipeSegments[newSegmentHandle];
		newSegment.m_endSegment = false;
		newSegment.m_nextHandle = nextSegmentHandle;
		auto& nextSegment = m_pipeSegments[nextSegmentHandle];
		nextSegment.m_prevHandle = newSegmentHandle;
		pipe.m_pipeSegmentHandles.insert(pipe.m_pipeSegmentHandles.begin() + prevSegmentIndex + 1, newSegmentHandle);
		for(int i = prevSegmentIndex + 2; i < pipe.m_pipeSegmentHandles.size(); ++i)
		{
			m_pipeSegments[pipe.m_pipeSegmentHandles[i]].m_index = i;
		}
		m_version++;
		return newSegmentHandle;
	}


	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	void PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::RecyclePipeSegment(PipeSegmentHandle handle)
	{
		//Recycle subsequent segments from pipe.
		auto& segment = m_pipeSegments[handle];
		assert(!segment.m_recycled);
		if (segment.m_nextHandle != -1)
		{
			RecyclePipeSegment(segment.m_nextHandle);
		}
		if (segment.m_prevHandle != -1)
		{
			m_pipeSegments[segment.m_prevHandle].m_nextHandle = -1;
			m_pipeSegments[segment.m_prevHandle].m_endSegment = true;
		}

		auto& pipe = m_pipes[segment.m_pipeHandle];
		assert(pipe.m_pipeSegmentHandles.back() == handle);
		pipe.m_pipeSegmentHandles.pop_back();

		segment.m_recycled = true;
		segment.m_endSegment = true;
		segment.m_prevHandle = segment.m_nextHandle = -1;
		segment.m_data = {};
		segment.m_info = {};

		segment.m_index = -1;
		segment.m_pipeHandle = -1;
		m_pipeSegmentPool.emplace(handle);
		m_version++;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	void PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::RecyclePipe(PipeHandle handle)
	{
		//Recycle all segments;
		auto& pipe = m_pipes[handle];
		assert(!pipe.m_recycled);
		for (const auto& segmentHandle : pipe.m_pipeSegmentHandles)
		{
			auto& segment = m_pipeSegments[segmentHandle];
			segment.m_recycled = true;
			segment.m_endSegment = true;
			segment.m_prevHandle = segment.m_nextHandle = -1;
			segment.m_data = {};
			segment.m_info = {};

			segment.m_index = -1;
			segment.m_pipeHandle = -1;
			m_pipeSegmentPool.emplace(segmentHandle);
		}
		pipe.m_pipeSegmentHandles.clear();

		//Recycle pipe.
		pipe.m_recycled = true;
		pipe.m_data = {};
		pipe.m_info = {};
		m_pipePool.emplace(handle);
		m_version++;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	const std::vector<Pipe<PipeData>>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::PeekPipes() const
	{
		return m_pipes;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	const std::vector<PipeSegment<PipeSegmentData>>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::PeekPipeSegments() const
	{
		return m_pipeSegments;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	std::vector<Pipe<PipeData>>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::RefPipes()
	{
		return m_pipes;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	std::vector<PipeSegment<PipeSegmentData>>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::RefPipeSegments()
	{
		return m_pipeSegments;
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	Pipe<PipeData>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::RefPipe(PipeHandle handle)
	{
		return m_pipes[handle];
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	PipeSegment<PipeSegmentData>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::RefPipeSegment(PipeSegmentHandle handle)
	{
		return m_pipeSegments[handle];
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	const Pipe<PipeData>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::PeekPipe(PipeHandle handle) const
	{
		return m_pipes[handle];
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	const PipeSegment<PipeSegmentData>& PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::PeekPipeSegment(
		PipeSegmentHandle handle) const
	{
		return m_pipeSegments[handle];
	}

	template <typename PipeGroupData, typename PipeData, typename PipeSegmentData>
	int PipeGroup<PipeGroupData, PipeData, PipeSegmentData>::GetVersion() const
	{
		return m_version;
	}

	template <typename PipeSegmentData>
	bool PipeSegment<PipeSegmentData>::IsEndPipeSegment() const
	{
		return m_endSegment;
	}

	template <typename PipeSegmentData>
	bool PipeSegment<PipeSegmentData>::IsRecycled() const
	{
		return m_recycled;
	}

	template <typename PipeSegmentData>
	PipeSegmentHandle PipeSegment<PipeSegmentData>::GetHandle() const
	{
		return m_handle;
	}

	template <typename PipeSegmentData>
	PipeHandle PipeSegment<PipeSegmentData>::GetPipeHandle() const
	{
		return m_pipeHandle;
	}

	template <typename PipeSegmentData>
	PipeSegmentHandle PipeSegment<PipeSegmentData>::GetPrevHandle() const
	{
		return m_prevHandle;
	}

	template <typename PipeSegmentData>
	PipeSegmentHandle PipeSegment<PipeSegmentData>::GetNextHandle() const
	{
		return m_nextHandle;
	}

	template <typename PipeSegmentData>
	int PipeSegment<PipeSegmentData>::GetIndex() const
	{
		return m_index;
	}

	template <typename PipeSegmentData>
	PipeSegment<PipeSegmentData>::PipeSegment(const PipeHandle pipeHandle, const PipeSegmentHandle handle, const PipeSegmentHandle prevHandle)
	{
		m_pipeHandle = pipeHandle;
		m_handle = handle;
		m_prevHandle = prevHandle;
		m_nextHandle = -1;
		m_recycled = false;
		m_endSegment = true;

		m_index = -1;
		m_data = {};
		m_info = {};
	}

	template <typename PipeData>
	bool Pipe<PipeData>::IsRecycled() const
	{
		return m_recycled;
	}

	template <typename PipeData>
	PipeHandle Pipe<PipeData>::GetHandle() const
	{
		return m_handle;
	}

	template <typename PipeData>
	const std::vector<PipeSegmentHandle>& Pipe<PipeData>::PeekPipeSegmentHandles() const
	{
		return m_pipeSegmentHandles;
	}

	template <typename PipeData>
	Pipe<PipeData>::Pipe(const PipeHandle handle)
	{
		m_handle = handle;
		m_recycled = false;

		m_pipeSegmentHandles.clear();

		m_data = {};
		m_info = {};
	}
}
