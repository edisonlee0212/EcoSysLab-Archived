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
			const std::function<float(float xFactor, float distanceToRoot)>& func);

		static void GeneratePartially(
			const std::unordered_set<NodeHandle>& nodeHandles,
			const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
			const std::function<float(float xFactor, float distanceToRoot)>& func);
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
		FlowHandle m_baseFlowHandle = -1;
	};

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void CylindricalMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(const
		Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
		std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
		const std::function<float(float xFactor, float distanceToRoot)>& func) {
		const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
		std::vector<std::vector<RingSegment>> ringsList;
		std::vector<bool> mainChildStatus;

		std::unordered_map<NodeHandle, int> steps{};
		ringsList.resize(sortedInternodeList.size());
		mainChildStatus.resize(sortedInternodeList.size());
		std::vector<std::shared_future<void>> results;
		std::vector<std::vector<std::pair<NodeHandle, int>>> tempSteps{};
		tempSteps.resize(Jobs::Workers().Size());

		Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned internodeIndex, unsigned threadIndex) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;

			auto& rings = ringsList[internodeIndex];
			rings.clear();

			glm::vec3 p[4];
			glm::vec3 f[4];
			float d[4];

			p[1] = internode.m_info.m_globalPosition;
			f[1] = internode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			d[1] = internode.m_info.m_rootDistance;

			p[2] = internode.m_info.GetGlobalEndPosition();
			f[2] = internode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			d[2] = internode.m_info.m_rootDistance;

			float thicknessStart, thicknessEnd, thicknessStartT, thicknessEndT;
			thicknessStart = thicknessEnd = internode.m_info.m_thickness;
			if (internode.GetParentHandle() == -1)
			{
				p[0] = p[1] * 2.0f - p[2];
				f[0] = f[1] * 2.0f - f[2];
				d[0] = d[1] * 2.0f - d[2];
			}
			else
			{
				p[0] = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_globalPosition;
				f[0] = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				thicknessStart = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_thickness;
				d[0] = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_rootDistance;
			}
			if (internode.IsEndNode())
			{
				p[3] = p[2] * 2.0f - p[1];
				f[3] = f[2] * 2.0f - f[1];
				d[3] = d[2] * 2.0f - d[1];
			}
			else
			{
				float maxChildThickness = -1;
				NodeHandle maxChildHandle = -1;
				const auto& childHandles = internode.RefChildHandles();
				if (childHandles.size() == 1)
				{
					maxChildHandle = childHandles.at(0);
				}
				else {
					for (const auto& childHandle : childHandles) {
						const auto& childInternode = skeleton.PeekNode(childHandle);
						if (childInternode.IsApical())
						{
							maxChildHandle = childHandle;
							break;
						}
						const float childThickness = childInternode.m_info.m_thickness;
						if (childThickness > maxChildThickness)
						{
							maxChildThickness = childThickness;
							maxChildHandle = childHandle;
						}
					}
				}
				const auto& childInternode = skeleton.PeekNode(maxChildHandle);
				p[3] = childInternode.m_info.GetGlobalEndPosition();
				f[3] = childInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				d[3] = childInternode.m_info.m_rootDistance;
			}
#pragma region Subdivision internode here.
			const auto diameter = glm::max(thicknessStart, thicknessEnd) * glm::pi<float>();
			int step = diameter / settings.m_xSubdivision;
			if (step < 4)
				step = 4;
			if (step % 2 != 0)
				++step;

			tempSteps[threadIndex].emplace_back(internodeHandle, step);
			int amount = internodeInfo.m_length / (internodeInfo.m_thickness >= settings.m_trunkThickness ? settings.m_trunkYSubdivision : settings.m_branchYSubdivision);
			amount = glm::max(1, amount);
			for (int ringIndex = 1; ringIndex <= amount; ringIndex++) {
				const float a = static_cast<float>(ringIndex - 1) / amount;
				const float b = static_cast<float>(ringIndex) / amount;
				glm::vec3 startPosition, endPosition, startAxis, endAxis, tempStart, tempEnd;
				float rootDistanceStart, rootDistanceEnd;
				Strands::CubicInterpolation(p[0], p[1], p[2], p[3], startPosition, tempStart, a);
				Strands::CubicInterpolation(p[0], p[1], p[2], p[3], endPosition, tempEnd, b);
				Strands::CubicInterpolation(f[0], f[1], f[2], f[3], startAxis, tempStart, a);
				Strands::CubicInterpolation(f[0], f[1], f[2], f[3], endAxis, tempEnd, b);
				float ringThicknessStart = glm::mix(thicknessStart, thicknessEnd, a);
				float ringThicknessEnd = glm::mix(thicknessStart, thicknessEnd, b);
				Strands::CubicInterpolation(d[0], d[1], d[2], d[3], rootDistanceStart, thicknessStartT, a);
				Strands::CubicInterpolation(d[0], d[1], d[2], d[3], rootDistanceEnd, thicknessEndT, b);
				rings.emplace_back(
					startPosition, endPosition,
					startAxis,
					endAxis,
					ringThicknessStart * .5f, ringThicknessEnd * .5f, rootDistanceStart, rootDistanceEnd);
			}
#pragma endregion
			}, results);
		for (auto& i : results) i.wait();

		for (const auto& list : tempSteps)
		{
			for (const auto& element : list)
			{
				steps[element.first] = element.second;
			}
		}

		std::unordered_map<NodeHandle, int> vertexLastRingStartVertexIndex{};

		int nextTreePartIndex = 0;
		int nextLineIndex = 0;
		std::unordered_map<NodeHandle, TreePartInfo> treePartInfos{};

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
				if (internode.IsApical() || parentInternode.RefChildHandles().size() == 1) needStitching = true;
				if (!needStitching)
				{
					float maxChildThickness = -1;
					NodeHandle maxChildHandle = -1;
					for (const auto& childHandle : parentInternode.RefChildHandles()) {
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
				const auto& chainHandles = flow.RefNodeHandles();
				const bool hasMultipleChildren = flow.RefChildHandles().size() > 1;
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
					onlyChild = parentFlow.RefChildHandles().size() <= 1;
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
			float textureXStep = 1.0f / pStep * 4.0f;
			if (!needStitching) {
				for (int p = 0; p < pStep; p++) {
					float xFactor = static_cast<float>(p) / pStep;
					float yFactor = internodeInfo.m_rootDistance - internodeInfo.m_length;
					auto& ring = rings.at(0);
					auto direction = ring.GetDirection(
						parentUp, pAngleStep * p, true);
					archetype.m_position = ring.m_startPosition + direction * ring.m_startRadius * func(xFactor, yFactor);
					const float x =
						p < pStep / 2 ? p * textureXStep : (pStep - p) * textureXStep;
					archetype.m_texCoord = glm::vec2(x, 0.0f);
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

			textureXStep = 1.0f / step * 4.0f;
			int ringSize = rings.size();
			for (auto ringIndex = 0; ringIndex < ringSize; ringIndex++) {
				for (auto s = 0; s < step; s++) {
					float xFactor = static_cast<float>(s) / step;
					float yFactor = internodeInfo.m_rootDistance - internodeInfo.m_length + (ringIndex + 1) * internodeInfo.m_length / ringSize;
					auto& ring = rings.at(ringIndex);
					auto direction = ring.GetDirection(
						up, angleStep * s, false);
					archetype.m_position = ring.m_endPosition + direction * ring.m_endRadius * func(xFactor, yFactor);
					const auto x =
						s < (step / 2) ? s * textureXStep : (step - s) * textureXStep;
					const auto y = ringIndex % 2 == 0 ? 1.0f : 0.0f;
					archetype.m_texCoord = glm::vec2(x, y);
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
		const std::unordered_set<NodeHandle>& nodeHandles,
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, std::vector<Vertex>& vertices,
		std::vector<unsigned>& indices, const TreeMeshGeneratorSettings& settings,
		const std::function<float(float xFactor, float distanceToRoot)>& func)
	{
		const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
		std::vector<std::vector<RingSegment>> ringsList;
		std::unordered_map<NodeHandle, int> steps{};
		ringsList.resize(sortedInternodeList.size());
		std::vector<std::shared_future<void>> results;
		std::vector<std::vector<std::pair<NodeHandle, int>>> tempSteps{};
		tempSteps.resize(Jobs::Workers().Size());

		Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned internodeIndex, unsigned threadIndex) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;


			auto& rings = ringsList[internodeIndex];
			rings.clear();

			glm::vec3 p[4];
			glm::vec3 f[4];
			float d[4];

			p[1] = internode.m_info.m_globalPosition;
			f[1] = internode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			d[1] = internode.m_info.m_rootDistance;

			p[2] = internode.m_info.GetGlobalEndPosition();
			f[2] = internode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			d[2] = internode.m_info.m_rootDistance;

			float thicknessStart, thicknessEnd, thicknessStartT, thicknessEndT;
			thicknessStart = thicknessEnd = internode.m_info.m_thickness;
			bool hasParent = nodeHandles.find(internode.GetParentHandle()) != nodeHandles.end();
			bool hasChildren = false;
			const auto& childHandles = internode.RefChildHandles();
			for (const auto childHandle : childHandles)
			{
				if (nodeHandles.find(childHandle) == nodeHandles.end()) hasChildren = true;
			}

			if (!hasParent)
			{
				p[0] = p[1] * 2.0f - p[2];
				f[0] = f[1] * 2.0f - f[2];
				d[0] = d[1] * 2.0f - d[2];
			}
			else
			{
				p[0] = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_globalPosition;
				f[0] = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				thicknessStart = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_thickness;
				d[0] = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_rootDistance;
			}
			if (!hasChildren)
			{
				p[3] = p[2] * 2.0f - p[1];
				f[3] = f[2] * 2.0f - f[1];
				d[3] = d[2] * 2.0f - d[1];
			}
			else
			{
				float maxChildThickness = -1;
				NodeHandle maxChildHandle = -1;
				for (const auto& childHandle : childHandles) {
					if (nodeHandles.find(childHandle) == nodeHandles.end()) continue;
					const auto& childInternode = skeleton.PeekNode(childHandle);
					if (childInternode.IsApical())
					{
						maxChildHandle = childHandle;
						break;
					}
					const float childThickness = childInternode.m_info.m_thickness;
					if (childThickness > maxChildThickness)
					{
						maxChildThickness = childThickness;
						maxChildHandle = childHandle;
					}
				}

				const auto& childInternode = skeleton.PeekNode(maxChildHandle);

				p[3] = childInternode.m_info.GetGlobalEndPosition();
				f[3] = childInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				d[3] = childInternode.m_info.m_rootDistance;

			}
#pragma region Subdivision internode here.
			const auto diameter = glm::max(thicknessStart, thicknessEnd) * glm::pi<float>();
			int step = diameter / settings.m_xSubdivision;
			if (step < 4)
				step = 4;
			if (step % 2 != 0)
				++step;

			tempSteps[threadIndex].emplace_back(internodeHandle, step);
			int amount = internodeInfo.m_length / (internodeInfo.m_thickness >= settings.m_trunkThickness ? settings.m_trunkYSubdivision : settings.m_branchYSubdivision);

			amount = glm::max(1, amount);
			for (int ringIndex = 1; ringIndex <= amount; ringIndex++) {
				const float a = static_cast<float>(ringIndex - 1) / amount;
				const float b = static_cast<float>(ringIndex) / amount;
				glm::vec3 startPosition, endPosition, startAxis, endAxis, tempStart, tempEnd;
				float rootDistanceStart, rootDistanceEnd;
				Strands::CubicInterpolation(p[0], p[1], p[2], p[3], startPosition, tempStart, a);
				Strands::CubicInterpolation(p[0], p[1], p[2], p[3], endPosition, tempEnd, b);
				Strands::CubicInterpolation(f[0], f[1], f[2], f[3], startAxis, tempStart, a);
				Strands::CubicInterpolation(f[0], f[1], f[2], f[3], endAxis, tempEnd, b);
				float ringThicknessStart = glm::mix(thicknessStart, thicknessEnd, a);
				float ringThicknessEnd = glm::mix(thicknessStart, thicknessEnd, b);

				Strands::CubicInterpolation(d[0], d[1], d[2], d[3], rootDistanceStart, thicknessStartT, a);
				Strands::CubicInterpolation(d[0], d[1], d[2], d[3], rootDistanceEnd, thicknessEndT, b);
				rings.emplace_back(
					startPosition, endPosition,
					startAxis,
					endAxis,
					ringThicknessStart * .5f, ringThicknessEnd * .5f, rootDistanceStart, rootDistanceEnd);
			}
#pragma endregion
			}, results);
		for (auto& i : results) i.wait();

		for (const auto& list : tempSteps)
		{
			for (const auto& element : list)
			{
				steps[element.first] = element.second;
			}
		}

		std::unordered_map<NodeHandle, int> vertexLastRingStartVertexIndex{};
		std::unordered_map<NodeHandle, TreePartInfo> treePartInfos{};

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
				if (internode.IsApical() || parentInternode.RefChildHandles().size() == 1) needStitching = true;
				if (!needStitching)
				{
					float maxChildThickness = -1;
					NodeHandle maxChildHandle = -1;
					for (const auto& childHandle : parentInternode.RefChildHandles()) {
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
			float textureXStep = 1.0f / pStep * 4.0f;
			if (!needStitching) {
				for (int p = 0; p < pStep; p++) {
					float xFactor = static_cast<float>(p) / pStep;
					float yFactor = internodeInfo.m_rootDistance - internodeInfo.m_length;
					auto& ring = rings.at(0);
					auto direction = ring.GetDirection(
						parentUp, pAngleStep * p, true);
					archetype.m_position = ring.m_startPosition + direction * ring.m_startRadius * func(xFactor, yFactor);
					assert(!glm::any(glm::isnan(archetype.m_position)));
					const float x =
						p < pStep / 2 ? p * textureXStep : (pStep - p) * textureXStep;
					archetype.m_texCoord = glm::vec2(x, 0.0f);
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

			textureXStep = 1.0f / step * 4.0f;
			int ringSize = rings.size();
			for (auto ringIndex = 0; ringIndex < ringSize; ringIndex++) {
				for (auto s = 0; s < step; s++) {
					float xFactor = static_cast<float>(s) / step;
					float yFactor = internodeInfo.m_rootDistance - internodeInfo.m_length + (ringIndex + 1) * internodeInfo.m_length / ringSize;
					auto& ring = rings.at(ringIndex);
					auto direction = ring.GetDirection(
						up, angleStep * s, false);
					archetype.m_position = ring.m_endPosition + direction * ring.m_endRadius * func(xFactor, yFactor);
					assert(!glm::any(glm::isnan(archetype.m_position)));
					const auto x =
						s < (step / 2) ? s * textureXStep : (step - s) * textureXStep;
					const auto y = ringIndex % 2 == 0 ? 1.0f : 0.0f;
					archetype.m_texCoord = glm::vec2(x, y);
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
