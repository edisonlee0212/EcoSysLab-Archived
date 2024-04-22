#pragma once

#include "Vertex.hpp"
#include "Jobs.hpp"
#include "TreeModel.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	struct RingSegment {
		glm::vec3 m_startPosition, m_endPosition;
		glm::vec3 m_startAxis, m_endAxis;
		float m_startRadius, m_endRadius;
		float m_startDistanceToRoot;
		float m_endDistanceToRoot;
		RingSegment() = default;

		RingSegment(glm::vec3 startPosition, glm::vec3 endPosition,
			glm::vec3 startAxis, glm::vec3 endAxis,
			float startRadius, float endRadius, float startDistanceToRoot, float endDistanceToRoot);

		void AppendPoints(std::vector<Vertex>& vertices, glm::vec3& normalDir,
			int step);

		[[nodiscard]] glm::vec3 GetPoint(const glm::vec3& normalDir, float angle, bool isStart, float multiplier = 0.0f) const;
		[[nodiscard]] glm::vec3 GetDirection(const glm::vec3& normalDir, float angle, bool isStart) const;
	};



	struct PresentationOverrideSettings
	{
		float m_maxThickness = 0.0f;
	};

	struct TreeMeshGeneratorSettings {
		bool m_vertexColorOnly = false;
		bool m_enableFoliage = true;
		bool m_enableFruit = false;
		bool m_enableBranch = true;
		bool m_enableTwig = false;

		bool m_presentationOverride = false;
		PresentationOverrideSettings m_presentationOverrideSettings = {};
		float m_trunkThickness = 0.1f;
		float m_xSubdivision = 0.01f;
		float m_trunkYSubdivision = 0.01f;
		float m_branchYSubdivision = 0.01f;

		bool m_overrideRadius = false;
		float m_radius = 0.01f;
		bool m_junctionColor = false;
		float m_baseControlPointRatio = 0.3f;
		float m_branchControlPointRatio = 0.3f;
		bool m_smoothness = true;

		bool m_autoLevel = true;
		int m_voxelSubdivisionLevel = 10;
		int m_voxelSmoothIteration = 5;
		bool m_removeDuplicate = true;

		unsigned m_branchMeshType = 0;

		float m_treePartBaseDistance = 0.5f;
		float m_treePartEndDistance = 2.f;
		float m_treePartBreakRatio = 4.0f;

		float m_marchingCubeRadius = 0.01f;


		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);

		void Save(const std::string& name, YAML::Emitter& out);

		void Load(const std::string& name, const YAML::Node& in);
	};

	template<typename SkeletonData, typename FlowData, typename NodeData>
	class CylindricalMeshGenerator {
	public:
		static void Generate(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
			const std::function<void(glm::vec3& vertexPosition, const glm::vec3& direction, float xFactor, float distanceToRoot)>& vertexPositionModifier,
			const std::function<void(glm::vec2& texCoords, float xFactor, float distanceToRoot)>& texCoordsModifier);

		static void GeneratePartially(
			const std::unordered_set<SkeletonNodeHandle>& nodeHandles,
			const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
			const std::function<void(glm::vec3& vertexPosition, const glm::vec3& direction, float xFactor, float distanceToRoot)>& vertexPositionModifier,
			const std::function<void(glm::vec2& texCoords, float xFactor, float distanceToRoot)>& texCoordsModifier);
	};
	template<typename SkeletonData, typename FlowData, typename NodeData>
	class VoxelMeshGenerator {
	public:
		static void Generate(const Skeleton<SkeletonData, FlowData, NodeData>& treeSkeleton, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings);
	};

	struct TreePartInfo
	{
		int m_treePartIndex = -1;
		int m_lineIndex = -1;
		int m_treePartType = 0;
		float m_distanceToStart = 0.0f;
		SkeletonFlowHandle m_baseFlowHandle = -1;
	};

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void CylindricalMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(const
		Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
		std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
		const std::function<void(glm::vec3& vertexPosition, const glm::vec3& direction, float xFactor, float yFactor)>& vertexPositionModifier,
		const std::function<void(glm::vec2& texCoords, float xFactor, float yFactor)>& texCoordsModifier) {
		const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
		std::vector<std::vector<RingSegment>> ringsList;
		std::vector<bool> mainChildStatus;

		std::unordered_map<SkeletonNodeHandle, int> steps{};
		ringsList.resize(sortedInternodeList.size());
		mainChildStatus.resize(sortedInternodeList.size());
		std::vector<std::shared_future<void>> results;
		std::vector<std::vector<std::pair<SkeletonNodeHandle, int>>> tempSteps{};
		tempSteps.resize(Jobs::GetDefaultThreadSize());

		Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned internodeIndex, unsigned threadIndex) {
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
					rings.emplace_back(
						curve.GetPoint(a), curve.GetPoint(b),
						glm::mix(directionStart, directionEnd, a),
						glm::mix(directionStart, directionEnd, b),
						glm::mix(thicknessStart, thicknessEnd, a) * .5f,
						glm::mix(thicknessStart, thicknessEnd, b) * .5f,
						glm::mix(rootDistanceStart, rootDistanceEnd, a), glm::mix(rootDistanceStart, rootDistanceEnd, b));
				}
				else {
					rings.emplace_back(
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

		for (int internodeIndex = 0; internodeIndex < sortedInternodeList.size(); internodeIndex++) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			auto parentInternodeHandle = internode.GetParentHandle();
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
			int vertexIndex = vertices.size();
			Vertex archetype;
			const auto flowHandle = internode.GetFlowHandle();
			archetype.m_vertexInfo1 = internodeHandle + 1;
			archetype.m_vertexInfo2 = flowHandle + 1;
			if (settings.m_junctionColor) {
#pragma region TreePart
				const auto& flow = skeleton.PeekFlow(internode.GetFlowHandle());
				const auto& chainHandles = flow.PeekNodeHandles();
				const bool hasMultipleChildren = flow.PeekChildHandles().size() > 1;
				bool onlyChild = true;
				const auto parentFlowHandle = flow.GetParentHandle();
				float distanceToChainStart = 0;
				float distanceToChainEnd = 0;
				const auto chainSize = chainHandles.size();
				for (int i = 0; i < chainSize; i++)
				{
					if (chainHandles[i] == internodeHandle) break;
					distanceToChainStart += skeleton.PeekNode(chainHandles[i]).m_info.m_length;

				}
				distanceToChainEnd = flow.m_info.m_flowLength - distanceToChainStart - internode.m_info.m_length;
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

#pragma endregion
			}
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
					if (!settings.m_junctionColor) archetype.m_color = internodeInfo.m_color;
					vertices.push_back(archetype);
				}
			}
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
				// First we allocate nearest vertices for parent.
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
				// Second we allocate nearest vertices for child
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
					float xFactor = static_cast<float>(s) / step;
					auto& ring = rings.at(ringIndex);
					float yFactor = ring.m_endDistanceToRoot;
					auto direction = ring.GetDirection(
						up, angleStep * s, false);
					archetype.m_position = ring.m_endPosition + direction * ring.m_endRadius;
					vertexPositionModifier(archetype.m_position, direction * ring.m_endRadius, xFactor, yFactor);
					assert(!glm::any(glm::isnan(archetype.m_position)));
					archetype.m_texCoord = glm::vec2(xFactor, yFactor);
					texCoordsModifier(archetype.m_texCoord, xFactor, yFactor);
					if (!settings.m_junctionColor) archetype.m_color = internodeInfo.m_color;
					vertices.push_back(archetype);
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
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position
									&& !glm::any(glm::isnan(vertices[a].m_position))
									&& !glm::any(glm::isnan(vertices[b].m_position))
									&& !glm::any(glm::isnan(vertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
							else {
								auto a = parentLastRingStartVertexIndex + p;
								auto b = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pTarget[p];
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position
									&& !glm::any(glm::isnan(vertices[a].m_position))
									&& !glm::any(glm::isnan(vertices[b].m_position))
									&& !glm::any(glm::isnan(vertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
								a = vertexIndex + pTarget[p == pStep - 1 ? 0 : p + 1];
								b = vertexIndex + pTarget[p];
								c = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position
									&& !glm::any(glm::isnan(vertices[a].m_position))
									&& !glm::any(glm::isnan(vertices[b].m_position))
									&& !glm::any(glm::isnan(vertices[c].m_position))) {
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
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position
									&& !glm::any(glm::isnan(vertices[a].m_position))
									&& !glm::any(glm::isnan(vertices[b].m_position))
									&& !glm::any(glm::isnan(vertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
							else {
								auto a = vertexIndex + p;
								auto b = vertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pStep + pTarget[p];
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position
									&& !glm::any(glm::isnan(vertices[a].m_position))
									&& !glm::any(glm::isnan(vertices[b].m_position))
									&& !glm::any(glm::isnan(vertices[c].m_position))) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
								a = vertexIndex + pStep + pTarget[p == pStep - 1 ? 0 : p + 1];
								b = vertexIndex + pStep + pTarget[p];
								c = vertexIndex + (p == pStep - 1 ? 0 : p + 1);

								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position
									&& !glm::any(glm::isnan(vertices[a].m_position))
									&& !glm::any(glm::isnan(vertices[b].m_position))
									&& !glm::any(glm::isnan(vertices[c].m_position))) {
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
						if (vertices[a].m_position != vertices[b].m_position
							&& vertices[b].m_position != vertices[c].m_position
							&& vertices[a].m_position != vertices[c].m_position
							&& !glm::any(glm::isnan(vertices[a].m_position))
							&& !glm::any(glm::isnan(vertices[b].m_position))
							&& !glm::any(glm::isnan(vertices[c].m_position))) {
							indices.push_back(a);
							indices.push_back(b);
							indices.push_back(c);
						}


						// Up triangle
						a = vertexIndex + ringIndex * step + s + 1;
						b = vertexIndex + ringIndex * step + s;
						c = vertexIndex + (ringIndex - 1) * step + s + 1;
						if (vertices[a].m_position != vertices[b].m_position
							&& vertices[b].m_position != vertices[c].m_position
							&& vertices[a].m_position != vertices[c].m_position
							&& !glm::any(glm::isnan(vertices[a].m_position))
							&& !glm::any(glm::isnan(vertices[b].m_position))
							&& !glm::any(glm::isnan(vertices[c].m_position))) {
							indices.push_back(a);
							indices.push_back(b);
							indices.push_back(c);
						}
					}
					// Down triangle
					auto a = vertexIndex + (ringIndex - 1) * step + step - 1;
					auto b = vertexIndex + (ringIndex - 1) * step;
					auto c = vertexIndex + ringIndex * step + step - 1;
					if (vertices[a].m_position != vertices[b].m_position
						&& vertices[b].m_position != vertices[c].m_position
						&& vertices[a].m_position != vertices[c].m_position
						&& !glm::any(glm::isnan(vertices[a].m_position))
						&& !glm::any(glm::isnan(vertices[b].m_position))
						&& !glm::any(glm::isnan(vertices[c].m_position))) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}
					// Up triangle
					a = vertexIndex + ringIndex * step;
					b = vertexIndex + ringIndex * step + step - 1;
					c = vertexIndex + (ringIndex - 1) * step;
					if (vertices[a].m_position != vertices[b].m_position
						&& vertices[b].m_position != vertices[c].m_position
						&& vertices[a].m_position != vertices[c].m_position
						&& !glm::any(glm::isnan(vertices[a].m_position))
						&& !glm::any(glm::isnan(vertices[b].m_position))
						&& !glm::any(glm::isnan(vertices[c].m_position))) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}
				}
			}
			vertexLastRingStartVertexIndex[internodeHandle] = vertices.size() - step;
		}
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void CylindricalMeshGenerator<SkeletonData, FlowData, NodeData>::GeneratePartially(
		const std::unordered_set<SkeletonNodeHandle>& nodeHandles,
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
		std::vector<unsigned>& indices, const TreeMeshGeneratorSettings& settings,
		const std::function<void(glm::vec3& vertexPosition, const glm::vec3& direction, float xFactor, float distanceToRoot)>& vertexPositionModifier,
		const std::function<void(glm::vec2& texCoords, float xFactor, float distanceToRoot)>& texCoordsModifier)
	{
		const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
		std::vector<std::vector<RingSegment>> ringsList;
		std::unordered_map<SkeletonNodeHandle, int> steps{};
		ringsList.resize(sortedInternodeList.size());
		std::vector<std::shared_future<void>> results;
		std::vector<std::vector<std::pair<SkeletonNodeHandle, int>>> tempSteps{};
		tempSteps.resize(Jobs::GetDefaultThreadSize());

		Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned internodeIndex, unsigned threadIndex) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;

			auto& rings = ringsList[internodeIndex];
			rings.clear();

			bool hasParent = nodeHandles.find(internode.GetParentHandle()) != nodeHandles.end();
			glm::vec3 directionStart = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			glm::vec3 directionEnd = directionStart;
			float rootDistanceStart = internodeInfo.m_rootDistance;
			float rootDistanceEnd = rootDistanceStart;

			glm::vec3 positionStart = internodeInfo.m_globalPosition;
			glm::vec3 positionEnd =
				positionStart + internodeInfo.m_length * (settings.m_smoothness ? 1.0f - settings.m_baseControlPointRatio : 1.0f) * internodeInfo.GetGlobalDirection();
			float thicknessStart = internodeInfo.m_thickness;
			float thicknessEnd = internodeInfo.m_thickness;

			if (hasParent) {
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
					rings.emplace_back(
						curve.GetPoint(a), curve.GetPoint(b),
						glm::mix(directionStart, directionEnd, a),
						glm::mix(directionStart, directionEnd, b),
						glm::mix(thicknessStart, thicknessEnd, a) * .5f,
						glm::mix(thicknessStart, thicknessEnd, b) * .5f,
						glm::mix(rootDistanceStart, rootDistanceEnd, a), glm::mix(rootDistanceStart, rootDistanceEnd, b));
				}
				else {
					rings.emplace_back(
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
		std::unordered_map<SkeletonNodeHandle, TreePartInfo> treePartInfos{};

		for (int internodeIndex = 0; internodeIndex < sortedInternodeList.size(); internodeIndex++) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			if (nodeHandles.find(internodeHandle) == nodeHandles.end()) continue;
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			auto parentInternodeHandle = internode.GetParentHandle();

			bool hasParent = nodeHandles.find(internode.GetParentHandle()) != nodeHandles.end();
			bool needStitching = false;

			const glm::vec3 up = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			glm::vec3 parentUp = up;

			if (hasParent)
			{
				const auto& parentInternode = skeleton.PeekNode(parentInternodeHandle);
				parentUp = parentInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
				if (internode.IsApical() || parentInternode.PeekChildHandles().size() == 1) needStitching = true;
				if (!needStitching)
				{
					float maxChildThickness = -1;
					SkeletonNodeHandle maxChildHandle = -1;
					for (const auto& childHandle : parentInternode.PeekChildHandles()) {
						if (nodeHandles.find(childHandle) == nodeHandles.end()) continue;
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
			int vertexIndex = vertices.size();
			Vertex archetype;
			const auto flowHandle = internode.GetFlowHandle();
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
					if (!settings.m_junctionColor) archetype.m_color = internodeInfo.m_color;
					vertices.push_back(archetype);
				}
			}
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
				// First we allocate nearest vertices for parent.
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
				// Second we allocate nearest vertices for child
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
					float xFactor = static_cast<float>(s) / step;
					auto& ring = rings.at(ringIndex);
					float yFactor = ring.m_endDistanceToRoot;
					auto direction = ring.GetDirection(up, angleStep * s, false);
					archetype.m_position = ring.m_endPosition + direction * ring.m_endRadius;
					vertexPositionModifier(archetype.m_position, direction * ring.m_endRadius, xFactor, yFactor);
					assert(!glm::any(glm::isnan(archetype.m_position)));
					archetype.m_texCoord = glm::vec2(xFactor, yFactor);
					texCoordsModifier(archetype.m_texCoord, xFactor, yFactor);
					if (!settings.m_junctionColor) archetype.m_color = internodeInfo.m_color;
					vertices.push_back(archetype);
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
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
							else {
								auto a = parentLastRingStartVertexIndex + p;
								auto b = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pTarget[p];
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
								a = vertexIndex + pTarget[p == pStep - 1 ? 0 : p + 1];
								b = vertexIndex + pTarget[p];
								c = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position) {
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
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
							}
							else {
								auto a = vertexIndex + p;
								auto b = vertexIndex + (p == pStep - 1 ? 0 : p + 1);
								auto c = vertexIndex + pStep + pTarget[p];
								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position) {
									indices.push_back(a);
									indices.push_back(b);
									indices.push_back(c);
								}
								a = vertexIndex + pStep + pTarget[p == pStep - 1 ? 0 : p + 1];
								b = vertexIndex + pStep + pTarget[p];
								c = vertexIndex + (p == pStep - 1 ? 0 : p + 1);

								if (vertices[a].m_position != vertices[b].m_position
									&& vertices[b].m_position != vertices[c].m_position
									&& vertices[a].m_position != vertices[c].m_position) {
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
						if (vertices[a].m_position != vertices[b].m_position
							&& vertices[b].m_position != vertices[c].m_position
							&& vertices[a].m_position != vertices[c].m_position) {
							indices.push_back(a);
							indices.push_back(b);
							indices.push_back(c);
						}


						// Up triangle
						a = vertexIndex + ringIndex * step + s + 1;
						b = vertexIndex + ringIndex * step + s;
						c = vertexIndex + (ringIndex - 1) * step + s + 1;
						if (vertices[a].m_position != vertices[b].m_position
							&& vertices[b].m_position != vertices[c].m_position
							&& vertices[a].m_position != vertices[c].m_position) {
							indices.push_back(a);
							indices.push_back(b);
							indices.push_back(c);
						}
					}
					// Down triangle
					auto a = vertexIndex + (ringIndex - 1) * step + step - 1;
					auto b = vertexIndex + (ringIndex - 1) * step;
					auto c = vertexIndex + ringIndex * step + step - 1;
					if (vertices[a].m_position != vertices[b].m_position
						&& vertices[b].m_position != vertices[c].m_position
						&& vertices[a].m_position != vertices[c].m_position) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}
					// Up triangle
					a = vertexIndex + ringIndex * step;
					b = vertexIndex + ringIndex * step + step - 1;
					c = vertexIndex + (ringIndex - 1) * step;
					if (vertices[a].m_position != vertices[b].m_position
						&& vertices[b].m_position != vertices[c].m_position
						&& vertices[a].m_position != vertices[c].m_position) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}
				}
			}
			vertexLastRingStartVertexIndex[internodeHandle] = vertices.size() - step;
		}
	}

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void VoxelMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(const
		Skeleton<SkeletonData, FlowData, NodeData>& treeSkeleton, std::vector<Vertex>& vertices,
		std::vector<unsigned>& indices, const TreeMeshGeneratorSettings& settings)
	{
		const auto boxSize = treeSkeleton.m_max - treeSkeleton.m_min;
		Octree<bool> octree;
		if (settings.m_autoLevel)
		{
			const float maxRadius = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z) * 0.5f + 2.0f * settings.m_marchingCubeRadius;
			int subdivisionLevel = -1;
			float testRadius = settings.m_marchingCubeRadius;
			while (testRadius <= maxRadius)
			{
				subdivisionLevel++;
				testRadius *= 2.f;
			}
			EVOENGINE_LOG("Mesh formation: Auto set level to " + std::to_string(subdivisionLevel))

				octree.Reset(maxRadius, subdivisionLevel, (treeSkeleton.m_min + treeSkeleton.m_max) * 0.5f);
		}
		else {
			octree.Reset(glm::max((boxSize.x, boxSize.y), glm::max(boxSize.y, boxSize.z)) * 0.5f,
				glm::clamp(settings.m_voxelSubdivisionLevel, 4, 16), (treeSkeleton.m_min + treeSkeleton.m_max) / 2.0f);
		}
		auto& nodeList = treeSkeleton.PeekSortedNodeList();
		for (const auto& nodeIndex : nodeList)
		{
			const auto& node = treeSkeleton.PeekNode(nodeIndex);
			const auto& info = node.m_info;
			auto thickness = info.m_thickness;
			if (node.GetParentHandle() > 0)
			{
				thickness = (thickness + treeSkeleton.PeekNode(node.GetParentHandle()).m_info.m_thickness) / 2.0f;
			}
			octree.Occupy(info.m_globalPosition, info.m_globalRotation, info.m_length, thickness, [](OctreeNode&) {});
		}
		octree.TriangulateField(vertices, indices, settings.m_removeDuplicate);
	}
}
