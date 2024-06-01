#pragma once

#include "Vertex.hpp"
#include "TreeMeshGenerator.hpp"
using namespace EvoEngine;

namespace EcoSysLab {

	template<typename SkeletonData, typename FlowData, typename NodeData>
	class CylindricalSkinnedMeshGenerator {
	public:
		static void GenerateBones(
			const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			const std::vector<SkeletonFlowHandle>& flowHandles,
			std::vector<glm::mat4>& offsetMatrices,
			std::unordered_map<SkeletonFlowHandle, int>& flowStartBoneIdMap,
			std::unordered_map<SkeletonFlowHandle, int>& flowEndBoneIdMap);
		static void Generate(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			std::vector<SkinnedVertex>& skinnedVertices, std::vector<unsigned int>& indices, std::vector<glm::mat4>& offsetMatrices,
			const TreeMeshGeneratorSettings& settings,
			const std::function<void(glm::vec3& vertexPosition, const glm::vec3& direction, float xFactor, float distanceToRoot)>& vertexPositionModifier,
			const std::function<void(glm::vec2& texCoords, float xFactor, float distanceToRoot)>& texCoordsModifier);
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void CylindricalSkinnedMeshGenerator<SkeletonData, FlowData, NodeData>::GenerateBones(
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		const std::vector<SkeletonFlowHandle>& flowHandles,
		std::vector<glm::mat4>& offsetMatrices,
		std::unordered_map<SkeletonFlowHandle, int>& flowStartBoneIdMap,
		std::unordered_map<SkeletonFlowHandle, int>& flowEndBoneIdMap)
	{
		flowStartBoneIdMap.clear();
		flowEndBoneIdMap.clear();
		int currentBoneIndex = 0;
		for (const auto& flowHandle : flowHandles)
		{
			flowStartBoneIdMap[flowHandle] = currentBoneIndex;
			currentBoneIndex++;
		}

		for (const auto& flowHandle : flowHandles)
		{
			const auto& flow = skeleton.PeekFlow(flowHandle);
			const auto& children = flow.PeekChildHandles();
			flowEndBoneIdMap[flowHandle] = flowStartBoneIdMap[flowHandle];
			/*
			if(children.empty()) flowEndBoneIdMap[flowHandle] = flowStartBoneIdMap[flowHandle];
			else
			{
				flowEndBoneIdMap[flowHandle] = flowStartBoneIdMap[children.front()];
			}*/
		}

		offsetMatrices.resize(currentBoneIndex + 1);
		for(const auto& [flowHandle, matrixIndex] : flowStartBoneIdMap)
		{
			const auto& flow = skeleton.PeekFlow(flowHandle);
			offsetMatrices[matrixIndex] = glm::inverse(glm::translate(flow.m_info.m_globalStartPosition) * glm::mat4_cast(flow.m_info.m_globalStartRotation));
		}
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void CylindricalSkinnedMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<SkinnedVertex>& skinnedVertices,
		std::vector<unsigned int>& indices, std::vector<glm::mat4>& offsetMatrices, 
		const TreeMeshGeneratorSettings& settings,
		const std::function<void(glm::vec3& vertexPosition, const glm::vec3& direction, float xFactor, float
			distanceToRoot)>& vertexPositionModifier,
		const std::function<void(glm::vec2& texCoords, float xFactor, float distanceToRoot)>& texCoordsModifier)
	{
		const auto& sortedFlowList = skeleton.PeekSortedFlowList();
		const auto& sortedInternodeList = skeleton.PeekSortedNodeList();


		std::vector<std::vector<RingSegment>> ringsList;
		std::vector<bool> mainChildStatus;

		std::unordered_map<SkeletonNodeHandle, int> steps{};
		ringsList.resize(sortedInternodeList.size());
		mainChildStatus.resize(sortedInternodeList.size());
		std::vector<std::shared_future<void>> results;
		std::vector<std::vector<std::pair<SkeletonNodeHandle, int>>> tempSteps{};
		tempSteps.resize(Jobs::GetWorkerSize());

		Jobs::RunParallelFor(sortedInternodeList.size(), [&](unsigned internodeIndex, unsigned threadIndex) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;

			auto& rings = ringsList[internodeIndex];
			rings.clear();

			glm::vec3 directionStart = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			glm::vec3 directionEnd = directionStart;
			float rootDistanceStart = internodeInfo.m_rootDistance;
			float rootDistanceEnd = rootDistanceStart;

			glm::vec3 positionStart = internodeInfo.m_globalPosition;
			glm::vec3 positionEnd =
				positionStart + internodeInfo.m_length * (settings.m_smoothness ? 1.0f - settings.m_baseControlPointRatio : 1.0f) * internodeInfo.GetGlobalDirection();
			float thicknessStart = internodeInfo.m_thickness;
			float thicknessEnd = internodeInfo.m_thickness;

			if (internode.GetParentHandle() != -1) {
				const auto& parentInternode = skeleton.PeekNode(internode.GetParentHandle());
				thicknessStart = parentInternode.m_info.m_thickness;
				directionStart =
					parentInternode.m_info.m_regulatedGlobalRotation *
					glm::vec3(0, 0, -1);
				positionStart =
					parentInternode.m_info.m_globalPosition + (parentInternode.m_info.m_length * (settings.m_smoothness ? 1.0f - settings.m_baseControlPointRatio : 1.0f)) * parentInternode.m_info.GetGlobalDirection();

				rootDistanceStart = parentInternode.m_info.m_rootDistance;
			}

			if (settings.m_overrideRadius) {
				thicknessStart = settings.m_radius;
				thicknessEnd = settings.m_radius;
			}

			if (settings.m_presentationOverride && settings.m_presentationOverrideSettings.m_maxThickness != 0.0f)
			{
				thicknessStart = glm::min(thicknessStart, settings.m_presentationOverrideSettings.m_maxThickness);
				thicknessEnd = glm::min(thicknessEnd, settings.m_presentationOverrideSettings.m_maxThickness);
			}

#pragma region Subdivision internode here.
			const auto boundaryLength = glm::max(thicknessStart, thicknessEnd) * glm::pi<float>();
			int step = boundaryLength / settings.m_xSubdivision;
			if (step < 4)
				step = 4;
			if (step % 2 != 0)
				++step;

			tempSteps[threadIndex].emplace_back(internodeHandle, step);
			int amount = glm::max(1, static_cast<int>(glm::distance(positionStart, positionEnd) / (internodeInfo.m_thickness >= settings.m_trunkThickness ? settings.m_trunkYSubdivision : settings.m_branchYSubdivision)));
			if (amount % 2 != 0)
				++amount;
			amount = glm::max(1, amount);
			BezierCurve curve = BezierCurve(
				positionStart,
				positionStart +
				(settings.m_smoothness ? internodeInfo.m_length * settings.m_baseControlPointRatio : 0.0f) * directionStart,
				positionEnd -
				(settings.m_smoothness ? internodeInfo.m_length * settings.m_branchControlPointRatio : 0.0f) * directionEnd,
				positionEnd);

			for (int ringIndex = 1; ringIndex <= amount; ringIndex++) {
				const float a = static_cast<float>(ringIndex - 1) / amount;
				const float b = static_cast<float>(ringIndex) / amount;
				if (settings.m_smoothness) {
					rings.emplace_back(a, b,
						curve.GetPoint(a), curve.GetPoint(b),
						glm::mix(directionStart, directionEnd, a),
						glm::mix(directionStart, directionEnd, b),
						glm::mix(thicknessStart, thicknessEnd, a) * .5f,
						glm::mix(thicknessStart, thicknessEnd, b) * .5f,
						glm::mix(rootDistanceStart, rootDistanceEnd, a), glm::mix(rootDistanceStart, rootDistanceEnd, b));
				}
				else {
					rings.emplace_back(a, b,
						curve.GetPoint(a), curve.GetPoint(b),
						directionEnd,
						directionEnd,
						glm::mix(thicknessStart, thicknessEnd, a) * .5f,
						glm::mix(thicknessStart, thicknessEnd, b) * .5f,
						glm::mix(rootDistanceStart, rootDistanceEnd, a), glm::mix(rootDistanceStart, rootDistanceEnd, b));
				}
			}
#pragma endregion
			});

		for (const auto& list : tempSteps)
		{
			for (const auto& element : list)
			{
				steps[element.first] = element.second;
			}
		}

		std::unordered_map<SkeletonNodeHandle, int> vertexLastRingStartVertexIndex{};

		int nextTreePartIndex = 0;
		int nextLineIndex = 0;
		std::unordered_map<SkeletonNodeHandle, TreePartInfo> treePartInfos{};

		std::unordered_map<SkeletonFlowHandle, int> flowStartBoneIdMap;
		std::unordered_map<SkeletonFlowHandle, int> flowEndBoneIdMap;

		GenerateBones(skeleton, sortedFlowList, offsetMatrices, flowStartBoneIdMap, flowEndBoneIdMap);

		for (int internodeIndex = 0; internodeIndex < sortedInternodeList.size(); internodeIndex++) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			auto parentInternodeHandle = internode.GetParentHandle();
			SkinnedVertex archetype{};
			const auto flowHandle = internode.GetFlowHandle();
			const auto& flow = skeleton.PeekFlow(flowHandle);
			const auto& chainHandles = flow.PeekNodeHandles();
			const auto parentFlowHandle = flow.GetParentHandle();
			float distanceToChainStart = 0;
			float distanceToChainEnd = 0;
			const auto chainSize = chainHandles.size();
			for (int i = 0; i < chainSize; i++)
			{
				if (chainHandles[i] == internodeHandle) break;
				distanceToChainStart += skeleton.PeekNode(chainHandles[i]).m_info.m_length;

			}
			distanceToChainEnd = flow.m_info.m_flowLength - distanceToChainStart;
			if (!internode.IsEndNode()) distanceToChainEnd -= internode.m_info.m_length;
#pragma region TreePart
			if (settings.m_vertexColorMode == static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::Junction)) {

				const bool hasMultipleChildren = flow.PeekChildHandles().size() > 1;
				bool onlyChild = true;
				
				float compareRadius = internode.m_info.m_thickness;
				if (parentFlowHandle != -1)
				{
					const auto& parentFlow = skeleton.PeekFlow(parentFlowHandle);
					onlyChild = parentFlow.PeekChildHandles().size() <= 1;
					compareRadius = parentFlow.m_info.m_endThickness;
				}
				int treePartType = 0;
				if (hasMultipleChildren && distanceToChainEnd <= settings.m_treePartBaseDistance * compareRadius) {
					treePartType = 1;
				}
				else if (!onlyChild && distanceToChainStart <= settings.m_treePartEndDistance * compareRadius)
				{
					treePartType = 2;
				}
				int currentTreePartIndex = -1;
				int currentLineIndex = -1;
				archetype.m_vertexInfo4.y = 0;
				if (treePartType == 0)
				{
					//IShape
					//If root or parent is Y Shape or length exceeds limit, create a new IShape from this node.
					bool restartIShape = parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType != 0;
					if (!restartIShape)
					{
						const auto& parentTreePartInfo = treePartInfos[parentInternodeHandle];
						if (parentTreePartInfo.m_distanceToStart / internodeInfo.m_thickness > settings.m_treePartBreakRatio) restartIShape = true;
					}
					if (restartIShape)
					{
						TreePartInfo treePartInfo;
						treePartInfo.m_treePartType = 0;
						treePartInfo.m_treePartIndex = nextTreePartIndex;
						treePartInfo.m_lineIndex = nextLineIndex;
						treePartInfo.m_distanceToStart = 0.0f;
						treePartInfos[internodeHandle] = treePartInfo;
						currentTreePartIndex = nextTreePartIndex;
						nextTreePartIndex++;

						currentLineIndex = nextLineIndex;
						nextLineIndex++;
					}
					else
					{
						auto& currentTreePartInfo = treePartInfos[internodeHandle];
						currentTreePartInfo = treePartInfos[parentInternodeHandle];
						currentTreePartInfo.m_distanceToStart += internodeInfo.m_length;
						currentTreePartInfo.m_treePartType = 0;
						currentTreePartIndex = currentTreePartInfo.m_treePartIndex;

						currentLineIndex = currentTreePartInfo.m_lineIndex;
					}
					archetype.m_vertexInfo4.y = 1;
					//archetype.m_color = glm::vec4(1, 1, 1, 1);
				}
				else if (treePartType == 1)
				{
					//Base of Y Shape
					if (parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType != 1
						|| treePartInfos[parentInternodeHandle].m_baseFlowHandle != flowHandle)
					{
						TreePartInfo treePartInfo;
						treePartInfo.m_treePartType = 1;
						treePartInfo.m_treePartIndex = nextTreePartIndex;
						treePartInfo.m_lineIndex = nextLineIndex;
						treePartInfo.m_distanceToStart = 0.0f;
						treePartInfo.m_baseFlowHandle = flowHandle;
						treePartInfos[internodeHandle] = treePartInfo;
						currentTreePartIndex = nextTreePartIndex;
						nextTreePartIndex++;

						currentLineIndex = nextLineIndex;
						nextLineIndex++;
					}
					else
					{
						auto& currentTreePartInfo = treePartInfos[internodeHandle];
						currentTreePartInfo = treePartInfos[parentInternodeHandle];
						currentTreePartInfo.m_treePartType = 1;
						currentTreePartIndex = currentTreePartInfo.m_treePartIndex;
						currentLineIndex = currentTreePartInfo.m_lineIndex;
					}
					archetype.m_vertexInfo4.y = 2;
					//archetype.m_color = glm::vec4(1, 0, 0, 1);
				}
				else if (treePartType == 2)
				{
					//Branch of Y Shape
					if (parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType == 0
						|| treePartInfos[parentInternodeHandle].m_baseFlowHandle != parentFlowHandle)
					{
					}
					else
					{
						auto& currentTreePartInfo = treePartInfos[internodeHandle];
						currentTreePartInfo = treePartInfos[parentInternodeHandle];
						if (currentTreePartInfo.m_treePartType != 2)
						{
							currentTreePartInfo.m_lineIndex = nextLineIndex;
							nextLineIndex++;
						}
						currentTreePartInfo.m_treePartType = 2;
						currentTreePartIndex = currentTreePartInfo.m_treePartIndex;

						currentLineIndex = currentTreePartInfo.m_lineIndex;
					}
					archetype.m_vertexInfo4.y = 2;
					//archetype.m_color = glm::vec4(1, 0, 0, 1);
				}
				archetype.m_vertexInfo3 = currentLineIndex + 1;
				archetype.m_vertexInfo4.x = currentTreePartIndex + 1;
			}
#pragma endregion
			const glm::vec3 up = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			glm::vec3 parentUp = up;
			bool needStitching = false;
			if (parentInternodeHandle != -1)
			{
				const auto& parentInternode = skeleton.PeekNode(parentInternodeHandle);
				parentUp = parentInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
				if (internode.IsApical() || parentInternode.PeekChildHandles().size() == 1) needStitching = true;
				if (!needStitching)
				{
					float maxChildThickness = -1;
					SkeletonNodeHandle maxChildHandle = -1;
					for (const auto& childHandle : parentInternode.PeekChildHandles()) {
						const auto& childInternode = skeleton.PeekNode(childHandle);
						if (childInternode.IsApical()) break;
						const float childThickness = childInternode.m_info.m_thickness;
						if (childThickness > maxChildThickness)
						{
							maxChildThickness = childThickness;
							maxChildHandle = childHandle;
						}
					}
					if (maxChildHandle == internodeHandle) needStitching = true;
				}
			}

			if (internode.m_info.m_length == 0.0f) {
				//TODO: Model possible knots and wound here.
				continue;
			}
			auto& rings = ringsList[internodeIndex];
			if (rings.empty()) {
				continue;
			}
			// For stitching
			const int step = steps[internodeHandle];
			int pStep = step;
			if (needStitching)
			{
				pStep = steps[parentInternodeHandle];
			}
			float angleStep = 360.0f / static_cast<float>(step);
			float pAngleStep = 360.0f / static_cast<float>(pStep);
			int vertexIndex = skinnedVertices.size();

			
			archetype.m_vertexInfo1 = internodeHandle + 1;
			archetype.m_vertexInfo2 = flowHandle + 1;

			if (!needStitching) {
				for (int p = 0; p < pStep; p++) {
					float xFactor = static_cast<float>(p) / pStep;
					const auto& ring = rings.at(0);
					float yFactor = ring.m_startDistanceToRoot;
					auto direction = ring.GetDirection(parentUp, pAngleStep * p, true);
					archetype.m_position = ring.m_startPosition + direction * ring.m_startRadius;
					vertexPositionModifier(archetype.m_position, direction * ring.m_startRadius, xFactor, yFactor);
					assert(!glm::any(glm::isnan(archetype.m_position)));
					archetype.m_texCoord = glm::vec2(xFactor, yFactor);
					texCoordsModifier(archetype.m_texCoord, xFactor, yFactor);
					if (settings.m_vertexColorMode == static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::InternodeColor)) archetype.m_color = internodeInfo.m_color;
					if(parentFlowHandle != -1) archetype.m_bondId = glm::ivec4(flowStartBoneIdMap[parentFlowHandle], flowEndBoneIdMap[parentFlowHandle], -1, -1);
					else{
						archetype.m_bondId = glm::ivec4(-1, flowStartBoneIdMap[0], -1, -1);
					}
					archetype.m_weight.x = 0.f;
					archetype.m_weight.y = 1.f;
					skinnedVertices.push_back(archetype);
				}
			}
			archetype.m_bondId = glm::ivec4(flowStartBoneIdMap[flowHandle], flowEndBoneIdMap[flowHandle], -1, -1);
			archetype.m_bondId2 = glm::ivec4(-1);
			archetype.m_weight = archetype.m_weight2 = glm::vec4(0.f);

			std::vector<float> angles;
			angles.resize(step);
			std::vector<float> pAngles;
			pAngles.resize(pStep);

			for (auto p = 0; p < pStep; p++) {
				pAngles[p] = pAngleStep * p;
			}
			for (auto s = 0; s < step; s++) {
				angles[s] = angleStep * s;
			}

			std::vector<unsigned> pTarget;
			std::vector<unsigned> target;
			pTarget.resize(pStep);
			target.resize(step);
			for (int p = 0; p < pStep; p++) {
				// First we allocate nearest skinnedVertices for parent.
				auto minAngleDiff = 360.0f;
				for (auto j = 0; j < step; j++) {
					const float diff = glm::abs(pAngles[p] - angles[j]);
					if (diff < minAngleDiff) {
						minAngleDiff = diff;
						pTarget[p] = j;
					}
				}
			}
			for (int s = 0; s < step; s++) {
				// Second we allocate nearest skinnedVertices for child
				float minAngleDiff = 360.0f;
				for (int j = 0; j < pStep; j++) {
					const float diff = glm::abs(angles[s] - pAngles[j]);
					if (diff < minAngleDiff) {
						minAngleDiff = diff;
						target[s] = j;
					}
				}
			}

			int ringSize = rings.size();
			for (auto ringIndex = 0; ringIndex < ringSize; ringIndex++) {
				for (auto s = 0; s < step; s++) {
					float xFactor = static_cast<float>(glm::min(s, step - s)) / step;
					auto& ring = rings.at(ringIndex);
					float yFactor = ring.m_endDistanceToRoot;
					auto direction = ring.GetDirection(
						up, angleStep * s, false);
					archetype.m_position = ring.m_endPosition + direction * ring.m_endRadius;
					vertexPositionModifier(archetype.m_position, direction * ring.m_endRadius, xFactor, yFactor);
					assert(!glm::any(glm::isnan(archetype.m_position)));
					archetype.m_texCoord = glm::vec2(xFactor, yFactor);
					texCoordsModifier(archetype.m_texCoord, xFactor, yFactor);
					if (settings.m_vertexColorMode == static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::InternodeColor)) archetype.m_color = internodeInfo.m_color;

					archetype.m_weight.x = glm::clamp((distanceToChainStart + ring.m_endA * internode.m_info.m_length)/ (distanceToChainStart + distanceToChainEnd), 0.f, 1.f);
					archetype.m_weight.y = glm::clamp((distanceToChainEnd - ring.m_endA * internode.m_info.m_length) / (distanceToChainStart + distanceToChainEnd), 0.f, 1.f);
					skinnedVertices.push_back(archetype);
				}
				if (ringIndex == 0)
				{
					if (needStitching) {
						int parentLastRingStartVertexIndex = vertexLastRingStartVertexIndex[parentInternodeHandle];
						for (int p = 0; p < pStep; p++) {
							if (pTarget[p] == pTarget[p == pStep - 1 ? 0 : p + 1]) {
								auto a = parentLastRingStartVertexIndex + p;
								auto b = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pTarget[p];
								if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
									&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
									&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
									&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
							else {
								auto a = parentLastRingStartVertexIndex + p;
								auto b = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pTarget[p];
								if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
									&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
									&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
									&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
								a = vertexIndex + pTarget[p == pStep - 1 ? 0 : p + 1];
								b = vertexIndex + pTarget[p];
								c = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
								if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
									&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
									&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
									&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
						}
					}
					else
					{
						for (int p = 0; p < pStep; p++) {
							if (pTarget[p] == pTarget[p == pStep - 1 ? 0 : p + 1]) {
								auto a = vertexIndex + p;
								auto b = vertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pStep + pTarget[p];
								if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
									&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
									&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
									&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
							else {
								auto a = vertexIndex + p;
								auto b = vertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pStep + pTarget[p];
								if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
									&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
									&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
									&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
								a = vertexIndex + pStep + pTarget[p == pStep - 1 ? 0 : p + 1];
								b = vertexIndex + pStep + pTarget[p];
								c = vertexIndex + (p == pStep - 1 ? 0 : p + 1);

								if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
									&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
									&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
									&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
									&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
						}
					}
					if (!needStitching) vertexIndex += pStep;
				}
				else {
					for (int s = 0; s < step - 1; s++) {
						// Down triangle
						auto a = vertexIndex + (ringIndex - 1) * step + s;
						auto b = vertexIndex + (ringIndex - 1) * step + s + 1;
						auto c = vertexIndex + ringIndex * step + s;
						if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
							&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
							&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
							&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
							&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
							&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
							indices.push_back(a);
							indices.push_back(b);
							indices.push_back(c);
						}


						// Up triangle
						a = vertexIndex + ringIndex * step + s + 1;
						b = vertexIndex + ringIndex * step + s;
						c = vertexIndex + (ringIndex - 1) * step + s + 1;
						if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
							&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
							&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
							&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
							&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
							&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
							indices.push_back(a);
							indices.push_back(b);
							indices.push_back(c);
						}
					}
					// Down triangle
					auto a = vertexIndex + (ringIndex - 1) * step + step - 1;
					auto b = vertexIndex + (ringIndex - 1) * step;
					auto c = vertexIndex + ringIndex * step + step - 1;
					if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
						&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
						&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
						&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
						&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
						&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}
					// Up triangle
					a = vertexIndex + ringIndex * step;
					b = vertexIndex + ringIndex * step + step - 1;
					c = vertexIndex + (ringIndex - 1) * step;
					if (skinnedVertices[a].m_position != skinnedVertices[b].m_position
						&& skinnedVertices[b].m_position != skinnedVertices[c].m_position
						&& skinnedVertices[a].m_position != skinnedVertices[c].m_position
						&& !glm::any(glm::isnan(skinnedVertices[a].m_position))
						&& !glm::any(glm::isnan(skinnedVertices[b].m_position))
						&& !glm::any(glm::isnan(skinnedVertices[c].m_position))) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}
				}
			}
			vertexLastRingStartVertexIndex[internodeHandle] = skinnedVertices.size() - step;
		}


	}
}
