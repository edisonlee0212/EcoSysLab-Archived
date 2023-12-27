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

		RingSegment() = default;

		RingSegment(glm::vec3 startPosition, glm::vec3 endPosition,
			glm::vec3 startAxis, glm::vec3 endAxis,
			float startRadius, float endRadius);

		void AppendPoints(std::vector<Vertex>& vertices, glm::vec3& normalDir,
			int step);

		[[nodiscard]] glm::vec3 GetPoint(const glm::vec3& normalDir, float angle, bool isStart, float multiplier = 0.0f) const;
		[[nodiscard]] glm::vec3 GetDirection(const glm::vec3& normalDir, float angle, const bool isStart) const;
	};

	struct FoliageParameters
	{
		glm::vec2 m_leafSize = glm::vec2(0.03f, 0.05f);
		glm::vec3 m_leafColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
		int m_leafCountPerInternode = 4;
		float m_positionVariance = 0.25f;
		float m_rotationVariance = 1.f;
		float m_branchingAngle = 30.f;
		float m_maxNodeThickness = 1.0f;
		float m_minRootDistance = 0.0f;
		float m_maxEndDistance = 10.f;
	};

	struct PresentationOverrideSettings
	{
		glm::vec3 m_rootOverrideColor = glm::vec3(80, 60, 50) / 255.0f;
		glm::vec3 m_branchOverrideColor = glm::vec3(109, 79, 75) / 255.0f;

		float m_maxThickness = 0.0f;
	};

	struct TwigParameters
	{
		float m_segmentLength = 0.01f;
		float m_apicalAngleVariance = 3.0f;
		float m_branchingAngle = 30.f;
		float m_thickness = 0.002f;
		float m_maxNodeThickness = 0.003f;
		float m_minRootDistance = 1.75f;
		float m_maxEndDistance = 999.0f;
		int m_segmentSize = 8;
		float m_unitDistance = 0.03f;
	};

	struct FineRootParameters
	{
		float m_segmentLength = 0.02f;
		float m_apicalAngleVariance = 5.0f;
		float m_branchingAngle = 30.f;
		float m_thickness = 0.002f;
		float m_maxNodeThickness = 0.003f;
		float m_minRootDistance = 0.0f;
		float m_maxEndDistance = 999.0f;
		int m_segmentSize = 8;
		float m_unitDistance = 0.025f;
	};



	struct TreeMeshGeneratorSettings {
		bool m_vertexColorOnly = false;
		bool m_enableFoliage = true;
		bool m_enableFruit = true;
		bool m_enableBranch = true;
		bool m_enableRoot = true;
		bool m_enableFineRoot = true;
		bool m_enableTwig = true;

		bool m_presentationOverride = false;
		bool m_foliageOverride = false;
		FoliageParameters m_foliageOverrideSettings = {};
		PresentationOverrideSettings m_presentationOverrideSettings = {};
		AssetRef m_foliageAlbedoTexture;
		AssetRef m_foliageNormalTexture;
		AssetRef m_foliageRoughnessTexture;
		AssetRef m_foliageMetallicTexture;

		float m_xSubdivision = 0.02f;
		float m_trunkYSubdivision = 0.02f;
		float m_trunkThickness = 0.1f;
		float m_branchYSubdivision = 0.1f;

		bool m_overrideRadius = false;
		float m_radius = 0.01f;
		bool m_overrideVertexColor = false;
		
		float m_baseControlPointRatio = 0.3f;
		float m_branchControlPointRatio = 0.3f;
		bool m_smoothness = true;

		bool m_autoLevel = true;
		int m_voxelSubdivisionLevel = 10;
		int m_voxelSmoothIteration = 5;
		bool m_removeDuplicate = true;

		glm::vec3 m_branchVertexColor = glm::vec3(1.0f);
		glm::vec3 m_foliageVertexColor = glm::vec3(1.0f);
		glm::vec3 m_rootVertexColor = glm::vec3(1.0f);

		unsigned m_branchMeshType = 0;
		unsigned m_rootMeshType = 0;

		float m_treePartBaseDistance = 1;
		float m_treePartEndDistance = 2;
		float m_treePartBreakRatio = 4.0f;


		FineRootParameters m_fineRootParameters{};
		TwigParameters m_twigParameters{};
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);

		void Save(const std::string& name, YAML::Emitter& out);

		void Load(const std::string& name, const YAML::Node& in);
	};

	template<typename SkeletonData, typename FlowData, typename NodeData>
	class CylindricalMeshGenerator {
	public:
		void Generate(const Skeleton<SkeletonData, FlowData, NodeData>& treeSkeleton, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
			const std::function<float(float xFactor, float distanceToRoot)>& func) const;
	};
	template<typename SkeletonData, typename FlowData, typename NodeData>
	class VoxelMeshGenerator {
	public:
		void Generate(const Skeleton<SkeletonData, FlowData, NodeData>& treeSkeleton, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings, float minRadius) const;
	};

	struct TreePartInfo
	{
		int m_treePartIndex = -1;
		int m_treePartType = 0;
		float m_distanceToStart = 0.0f;
	};

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void CylindricalMeshGenerator<SkeletonData, FlowData, NodeData>::Generate(const
		Skeleton<SkeletonData, FlowData, NodeData>& treeSkeleton, std::vector<Vertex>& vertices,
		std::vector<unsigned int>& indices, const TreeMeshGeneratorSettings& settings,
		const std::function<float(float xFactor, float distanceToRoot)>& func) const {
		const auto& sortedInternodeList = treeSkeleton.RefSortedNodeList();
		std::vector<std::vector<RingSegment>> ringsList;
		std::map<NodeHandle, int> steps{};
		ringsList.resize(sortedInternodeList.size());
		std::vector<std::shared_future<void>> results;
		std::vector<std::vector<std::pair<NodeHandle, int>>> tempSteps{};
		tempSteps.resize(Jobs::Workers().Size());

		Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned internodeIndex, unsigned threadIndex) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = treeSkeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			

			auto& rings = ringsList[internodeIndex];
			rings.clear();

			glm::vec3 directionStart = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			glm::vec3 directionEnd = directionStart;

			glm::vec3 positionStart = internodeInfo.m_globalPosition;
			glm::vec3 positionEnd =
				positionStart + internodeInfo.m_length * (settings.m_smoothness ? 1.0f - settings.m_baseControlPointRatio : 1.0f) * internodeInfo.m_globalDirection;
			float thicknessStart = internodeInfo.m_thickness;
			float thicknessEnd = internodeInfo.m_thickness;

			if (internode.GetParentHandle() != -1) {
				const auto& parentInternode = treeSkeleton.PeekNode(internode.GetParentHandle());
				thicknessStart = parentInternode.m_info.m_thickness;
				directionStart =
					parentInternode.m_info.m_regulatedGlobalRotation *
					glm::vec3(0, 0, -1);
				positionStart =
					parentInternode.m_info.m_globalPosition + (parentInternode.m_info.m_length * (settings.m_smoothness ? 1.0f - settings.m_baseControlPointRatio : 1.0f)) * parentInternode.m_info.m_globalDirection;
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
			const auto diameter = glm::max(thicknessStart, thicknessEnd) * 2.0f * glm::pi<float>();
			int step = diameter / settings.m_xSubdivision;
			if (step < 4)
				step = 4;
			if (step % 2 != 0)
				++step;

			tempSteps[threadIndex].emplace_back(internodeHandle, step);
			int amount = glm::max(1, static_cast<int>(glm::distance(positionStart, positionEnd) / (internodeInfo.m_thickness >= settings.m_trunkThickness ? settings.m_trunkYSubdivision : settings.m_branchYSubdivision)));
			if (amount % 2 != 0)
				++amount;
			BezierCurve curve = BezierCurve(
				positionStart,
				positionStart +
				(settings.m_smoothness ? internodeInfo.m_length * settings.m_baseControlPointRatio : 0.0f) * directionStart,
				positionEnd -
				(settings.m_smoothness ? internodeInfo.m_length * settings.m_branchControlPointRatio : 0.0f) * directionEnd,
				positionEnd);
			float posStep = 1.0f / static_cast<float>(amount);
			glm::vec3 dirStep = (directionEnd - directionStart) / static_cast<float>(amount);
			float radiusStep = (thicknessEnd - thicknessStart) /
				static_cast<float>(amount);

			for (int ringIndex = 1; ringIndex < amount; ringIndex++) {
				float startThickness = static_cast<float>(ringIndex - 1) * radiusStep;
				float endThickness = static_cast<float>(ringIndex) * radiusStep;
				if (settings.m_smoothness) {
					rings.emplace_back(
						curve.GetPoint(posStep * (ringIndex - 1)), curve.GetPoint(posStep * ringIndex),
						directionStart + static_cast<float>(ringIndex - 1) * dirStep,
						directionStart + static_cast<float>(ringIndex) * dirStep,
						thicknessStart + startThickness, thicknessStart + endThickness);
				}
				else {
					rings.emplace_back(
						curve.GetPoint(posStep * (ringIndex - 1)), curve.GetPoint(posStep * ringIndex),
						directionEnd,
						directionEnd,
						thicknessStart + startThickness, thicknessStart + endThickness);
				}
			}
			if (amount > 1)
				rings.emplace_back(
					curve.GetPoint(1.0f - posStep), positionEnd, directionEnd - dirStep,
					directionEnd,
					thicknessEnd - radiusStep,
					thicknessEnd);
			else
				rings.emplace_back(positionStart, positionEnd,
					directionStart, directionEnd, thicknessStart,
					thicknessEnd);
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

		std::map<NodeHandle, int> vertexLastRingStartVertexIndex{};

		int nextTreePartIndex = 0;
		std::unordered_map<NodeHandle, TreePartInfo> treePartInfos{};

		for (int internodeIndex = 0; internodeIndex < sortedInternodeList.size(); internodeIndex++) {
			auto internodeHandle = sortedInternodeList[internodeIndex];
			const auto& internode = treeSkeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			auto parentInternodeHandle = internode.GetParentHandle();
			const glm::vec3 up = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			glm::vec3 parentUp = up;
			if (parentInternodeHandle != -1)
			{
				const auto& parentInternode = treeSkeleton.PeekNode(parentInternodeHandle);
				parentUp = parentInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			}
			auto& rings = ringsList[internodeIndex];
			if (rings.empty()) {
				continue;
			}
			// For stitching
			const int step = steps[internodeHandle];
			int pStep = step;
			if (parentInternodeHandle != -1)
			{
				pStep = steps[parentInternodeHandle];
			}
			float angleStep = 360.0f / static_cast<float>(step);
			float pAngleStep = 360.0f / static_cast<float>(pStep);
			int vertexIndex = vertices.size();
			Vertex archetype;
			if (settings.m_overrideVertexColor) archetype.m_color = glm::vec4(settings.m_branchVertexColor, 1.0f);
			const auto flowHandle = internode.GetFlowHandle();
			archetype.m_vertexInfo1 = internodeHandle + 1;
			archetype.m_vertexInfo2 = flowHandle + 1;
#pragma region TreePart
			const auto& flow = treeSkeleton.PeekFlow(internode.GetFlowHandle());
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
				distanceToChainStart += treeSkeleton.PeekNode(chainHandles[i]).m_info.m_length;
				
			}
			distanceToChainEnd = flow.m_info.m_flowLength - distanceToChainStart - internode.m_info.m_length;
			float compareRadius = internode.m_info.m_thickness;
			if (parentFlowHandle != -1)
			{
				const auto& parentFlow = treeSkeleton.PeekFlow(parentFlowHandle);
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
			if(treePartType == 0)
			{
				//IShape
				//If root or parent is Y Shape or length exceeds limit, create a new IShape from this node.
				bool restartIShape = parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType > 0;
				if(!restartIShape)
				{
					const auto& parentTreePartInfo = treePartInfos[parentInternodeHandle];
					if (parentTreePartInfo.m_distanceToStart / internodeInfo.m_thickness > settings.m_treePartBreakRatio) restartIShape = true;
				}
				if (restartIShape)
				{
					TreePartInfo treePartInfo;
					treePartInfo.m_treePartType = 0;
					treePartInfo.m_treePartIndex = nextTreePartIndex;
					treePartInfo.m_distanceToStart = 0.0f;
					treePartInfos[internodeHandle] = treePartInfo;
					currentTreePartIndex = nextTreePartIndex;
					nextTreePartIndex++;
				}
				else
				{
					auto& currentTreePartInfo = treePartInfos[internodeHandle];
					currentTreePartInfo = treePartInfos[parentInternodeHandle];
					currentTreePartInfo.m_distanceToStart += internodeInfo.m_length;
					currentTreePartIndex = currentTreePartInfo.m_treePartIndex;
				}
				archetype.m_color = glm::vec4(1, 1, 1, 1);
			}else if(treePartType == 1)
			{
				//Base of Y Shape
				if (parentInternodeHandle == -1 || !treePartInfos[parentInternodeHandle].m_treePartType == 1)
				{
					TreePartInfo treePartInfo;
					treePartInfo.m_treePartType = 1;
					treePartInfo.m_treePartIndex = nextTreePartIndex;
					treePartInfo.m_distanceToStart = 0.0f;
					treePartInfos[internodeHandle] = treePartInfo;
					currentTreePartIndex = nextTreePartIndex;
					nextTreePartIndex++;
				}
				else
				{
					auto& currentTreePartInfo = treePartInfos[internodeHandle];
					currentTreePartInfo = treePartInfos[parentInternodeHandle];
					currentTreePartIndex = currentTreePartInfo.m_treePartIndex;
				}
				archetype.m_color = glm::vec4(1, 0, 0, 1);
			}else if(treePartType == 2)
			{
				//Branch of Y Shape
				if (parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType == 0)
				{
					TreePartInfo treePartInfo;
					treePartInfo.m_treePartType = 2;
					treePartInfo.m_treePartIndex = nextTreePartIndex;
					treePartInfo.m_distanceToStart = 0.0f;
					treePartInfos[internodeHandle] = treePartInfo;
					currentTreePartIndex = nextTreePartIndex;
					nextTreePartIndex++;
				}
				else
				{
					auto& currentTreePartInfo = treePartInfos[internodeHandle];
					currentTreePartInfo = treePartInfos[parentInternodeHandle];
					currentTreePartInfo.m_treePartType = 2;
					currentTreePartIndex = currentTreePartInfo.m_treePartIndex;
				}
				archetype.m_color = glm::vec4(1, 0, 0, 1);
			}
			archetype.m_vertexInfo3 = currentTreePartIndex;
#pragma endregion

			float textureXStep = 1.0f / pStep * 4.0f;
			if (parentInternodeHandle == -1) {
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
					archetype.m_color = internodeInfo.m_color;
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
					vertices.push_back(archetype);
				}
				if (ringIndex == 0)
				{
					if (parentInternodeHandle != -1) {
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
					if (parentInternodeHandle == -1) vertexIndex += pStep;
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
		std::vector<unsigned>& indices, const TreeMeshGeneratorSettings& settings, float minRadius) const
	{
		const auto boxSize = treeSkeleton.m_max - treeSkeleton.m_min;
		Octree<bool> octree;
		if (settings.m_autoLevel)
		{
			const float maxRadius = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z) * 0.5f + 2.0f * minRadius;
			int subdivisionLevel = -1;
			float testRadius = minRadius;
			while (testRadius <= maxRadius)
			{
				subdivisionLevel++;
				testRadius *= 2.f;
			}
			EVOENGINE_LOG("Root mesh formation: Auto set level to " + std::to_string(subdivisionLevel))

				octree.Reset(maxRadius, subdivisionLevel, (treeSkeleton.m_min + treeSkeleton.m_max) * 0.5f);
		}
		else {
			octree.Reset(glm::max((boxSize.x, boxSize.y), glm::max(boxSize.y, boxSize.z)) * 0.5f,
				glm::clamp(settings.m_voxelSubdivisionLevel, 4, 16), (treeSkeleton.m_min + treeSkeleton.m_max) / 2.0f);
		}
		auto& nodeList = treeSkeleton.RefSortedNodeList();
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
		octree.TriangulateField(vertices, indices, settings.m_removeDuplicate, settings.m_voxelSmoothIteration);
	}
}
