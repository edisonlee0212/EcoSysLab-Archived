#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "HexagonProfileData.hpp"
#include "PipeModel.hpp"
#include "Jobs.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	enum class PruningMode {
		None,
		Stroke
	};

	enum class ShootVisualizerMode {
		Default,
		LightIntensity,
		LightDirection,
		IsMaxChild,
		AllocatedVigor,

	};

	enum class RootVisualizerMode {
		Default,
		AllocatedVigor,
	};

	struct TreeVisualizerColorSettings {
		int m_shootVisualizationMode = static_cast<int>(ShootVisualizerMode::Default);
		int m_rootVisualizationMode = static_cast<int>(RootVisualizerMode::Default);
		float m_shootColorMultiplier = 1.0f;
		float m_rootColorMultiplier = 1.0f;
	};

	class TreeVisualizer {
		std::vector<glm::vec4> m_randomColors;

		std::shared_ptr<ParticleInfoList> m_internodeMatrices;
		std::shared_ptr<ParticleInfoList> m_rootNodeMatrices;

		std::vector<glm::vec2> m_storedMousePositions;
		bool m_visualization = true;

		bool m_hexagonProfileGui = true;
		bool m_treeHierarchyGui = false;
		bool m_rootHierarchyGui = false;

		TreeVisualizerColorSettings m_settings;

		NodeHandle m_selectedInternodeHandle = -1;
		float m_selectedInternodeLengthFactor = 0.0f;
		std::vector<NodeHandle> m_selectedInternodeHierarchyList;

		NodeHandle m_selectedRootNodeHandle = -1;
		float m_selectedRootNodeLengthFactor = 0.0f;
		std::vector<NodeHandle> m_selectedRootNodeHierarchyList;

		PruningMode m_mode = PruningMode::None;

		template<typename SkeletonData, typename FlowData, typename NodeData>
		bool
			RayCastSelection(const std::shared_ptr<Camera>& cameraComponent,
				const glm::vec2& mousePosition, const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
				const GlobalTransform& globalTransform, NodeHandle& selectedNodeHandle,
				std::vector<NodeHandle>& hierarchyList, float& lengthFactor);

		template<typename SkeletonData, typename FlowData, typename NodeData>
		bool ScreenCurvePruning(const std::function<void(NodeHandle)>& handler,
			Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			const GlobalTransform& globalTransform, NodeHandle& selectedNodeHandle,
			std::vector<NodeHandle>& hierarchyList);


		bool DrawInternodeInspectionGui(
			TreeModel& treeModel,
			NodeHandle internodeHandle, bool& deleted,
			const unsigned& hierarchyLevel);

		template<typename SkeletonData, typename FlowData, typename NodeData>
		void PeekNodeInspectionGui(
			const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			NodeHandle nodeHandle, NodeHandle& selectedNodeHandle, std::vector<NodeHandle>& hierarchyList,
			const unsigned& hierarchyLevel);

		bool DrawRootNodeInspectionGui(
			TreeModel& treeModel,
			NodeHandle rootNodeHandle, bool& deleted,
			const unsigned& hierarchyLevel);

		void
			PeekInternode(const ShootSkeleton& shootSkeleton,
				NodeHandle internodeHandle) const;

		bool
			InspectInternode(ShootSkeleton& shootSkeleton,
				NodeHandle internodeHandle);

		void
			PeekRootNode(
				const RootSkeleton& rootSkeleton,
				NodeHandle rootNodeHandle) const;

		bool
			InspectRootNode(RootSkeleton& rootSkeleton,
				NodeHandle rootNodeHandle);

	public:
		void Initialize();

		template<typename SkeletonData, typename FlowData, typename NodeData>
		void SetSelectedNode(
			const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			NodeHandle nodeHandle, NodeHandle& selectedNodeHandle, std::vector<NodeHandle>& hierarchyList);

		template<typename SkeletonData, typename FlowData, typename NodeData>
		void
			SyncMatrices(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, const std::shared_ptr<ParticleInfoList>& particleInfoList, NodeHandle& selectedNodeHandle, float& lengthFactor);

		void SyncColors(const ShootSkeleton& shootSkeleton, NodeHandle& selectedNodeHandle);

		void SyncColors(const RootSkeleton& rootSkeleton, const NodeHandle& selectedNodeHandle);

		int m_iteration = 0;
		bool m_needUpdate = false;
		bool m_needShootColorUpdate = false;
		bool m_needRootColorUpdate = false;

		bool
			OnInspect(TreeModel& treeModel, PipeModel& pipeModel,
				const GlobalTransform& globalTransform);

		bool Visualize(TreeModel& treeModel,
			const GlobalTransform& globalTransform);

		void Reset(TreeModel& treeModel);

		void Clear();
	};

	template<typename SkeletonData, typename FlowData, typename NodeData>
	bool TreeVisualizer::ScreenCurvePruning(const std::function<void(NodeHandle)>& handler,
		Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		const GlobalTransform& globalTransform, NodeHandle& selectedNodeHandle,
		std::vector<NodeHandle>& hierarchyList) {
		auto editorLayer = Application::GetLayer<EditorLayer>();
		const auto cameraRotation = editorLayer->GetSceneCameraRotation();
		const auto cameraPosition = editorLayer->GetSceneCameraPosition();
		const glm::vec3 cameraFront = cameraRotation * glm::vec3(0, 0, -1);
		const glm::vec3 cameraUp = cameraRotation * glm::vec3(0, 1, 0);
		glm::mat4 projectionView = editorLayer->GetSceneCamera()->GetProjection() *
			glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);

		const auto& sortedInternodeList = skeleton.RefSortedNodeList();
		bool changed = false;
		for (const auto& internodeHandle : sortedInternodeList) {
			if (internodeHandle == 0) continue;
			auto& internode = skeleton.RefNode(internodeHandle);
			if (internode.IsRecycled()) continue;
			glm::vec3 position = internode.m_info.m_globalPosition;
			auto rotation = internode.m_info.m_globalRotation;
			const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
			auto position2 =
				position + internode.m_info.m_length * direction;

			position = (globalTransform.m_value *
				glm::translate(position))[3];
			position2 = (globalTransform.m_value *
				glm::translate(position2))[3];
			const glm::vec4 internodeScreenStart4 = projectionView * glm::vec4(position, 1.0f);
			const glm::vec4 internodeScreenEnd4 = projectionView * glm::vec4(position2, 1.0f);
			glm::vec3 internodeScreenStart = internodeScreenStart4 / internodeScreenStart4.w;
			glm::vec3 internodeScreenEnd = internodeScreenEnd4 / internodeScreenEnd4.w;
			internodeScreenStart.x *= -1.0f;
			internodeScreenEnd.x *= -1.0f;
			if (internodeScreenStart.x < -1.0f || internodeScreenStart.x > 1.0f || internodeScreenStart.y < -1.0f ||
				internodeScreenStart.y > 1.0f || internodeScreenStart.z < 0.0f)
				continue;
			if (internodeScreenEnd.x < -1.0f || internodeScreenEnd.x > 1.0f || internodeScreenEnd.y < -1.0f ||
				internodeScreenEnd.y > 1.0f || internodeScreenEnd.z < 0.0f)
				continue;
			bool intersect = false;
			for (int i = 0; i < m_storedMousePositions.size() - 1; i++) {
				auto& lineStart = m_storedMousePositions[i];
				auto& lineEnd = m_storedMousePositions[i + 1];
				float a1 = internodeScreenEnd.y - internodeScreenStart.y;
				float b1 = internodeScreenStart.x - internodeScreenEnd.x;
				float c1 = a1 * (internodeScreenStart.x) + b1 * (internodeScreenStart.y);

				// Line CD represented as a2x + b2y = c2
				float a2 = lineEnd.y - lineStart.y;
				float b2 = lineStart.x - lineEnd.x;
				float c2 = a2 * (lineStart.x) + b2 * (lineStart.y);

				float determinant = a1 * b2 - a2 * b1;
				if (determinant == 0.0f) continue;
				float x = (b2 * c1 - b1 * c2) / determinant;
				float y = (a1 * c2 - a2 * c1) / determinant;
				if (x <= glm::max(internodeScreenStart.x, internodeScreenEnd.x) &&
					x >= glm::min(internodeScreenStart.x, internodeScreenEnd.x) &&
					y <= glm::max(internodeScreenStart.y, internodeScreenEnd.y) &&
					y >= glm::min(internodeScreenStart.y, internodeScreenEnd.y) &&
					x <= glm::max(lineStart.x, lineEnd.x) &&
					x >= glm::min(lineStart.x, lineEnd.x) &&
					y <= glm::max(lineStart.y, lineEnd.y) &&
					y >= glm::min(lineStart.y, lineEnd.y)) {
					intersect = true;
					break;
				}
			}
			if (intersect) {
				handler(internodeHandle);
				changed = true;
			}

		}
		if (changed) {
			selectedNodeHandle = -1;
			hierarchyList.clear();
		}
		return changed;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	bool TreeVisualizer::RayCastSelection(const std::shared_ptr<Camera>& cameraComponent,
		const glm::vec2& mousePosition,
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		const GlobalTransform& globalTransform, NodeHandle& selectedNodeHandle,
		std::vector<NodeHandle>& hierarchyList, float& lengthFactor) {
		auto editorLayer = Application::GetLayer<EditorLayer>();
		bool changed = false;
#pragma region Ray selection
		NodeHandle currentFocusingNodeHandle = -1;
		std::mutex writeMutex;
		float minDistance = FLT_MAX;
		GlobalTransform cameraLtw;
		cameraLtw.m_value =
			glm::translate(
				editorLayer->GetSceneCameraPosition()) *
			glm::mat4_cast(
				editorLayer->GetSceneCameraRotation());
		const Ray cameraRay = cameraComponent->ScreenPointToRay(
			cameraLtw, mousePosition);
		const auto& sortedNodeList = skeleton.RefSortedNodeList();
		std::vector<std::shared_future<void>> results;
		Jobs::ParallelFor(sortedNodeList.size(), [&](unsigned i) {
			const auto& node = skeleton.PeekNode(sortedNodeList[i]);
			auto rotation = globalTransform.GetRotation() * node.m_info.m_globalRotation;
			glm::vec3 position = (globalTransform.m_value *
				glm::translate(node.m_info.m_globalPosition))[3];
			const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
			const glm::vec3 position2 =
				position + node.m_info.m_length * direction;
			const auto center =
				(position + position2) / 2.0f;
			auto radius = node.m_info.m_thickness;
			const auto height = glm::distance(position2,
				position);
			radius *= height / node.m_info.m_length;
			if (!cameraRay.Intersect(center,
				height / 2.0f) && !cameraRay.Intersect(center,
					radius)) {
				return;
			}
			const auto& dir = -cameraRay.m_direction;
#pragma region Line Line intersection
			/*
* http://geomalgorithms.com/a07-_distance.html
*/
			glm::vec3 v = position - position2;
			glm::vec3 w = (cameraRay.m_start + dir) - position2;
			const auto a = glm::dot(dir, dir); // always >= 0
			const auto b = glm::dot(dir, v);
			const auto c = glm::dot(v, v); // always >= 0
			const auto d = glm::dot(dir, w);
			const auto e = glm::dot(v, w);
			const auto dotP = a * c - b * b; // always >= 0
			float sc, tc;
			// compute the line parameters of the two closest points
			if (dotP < 0.00001f) { // the lines are almost parallel
				sc = 0.0f;
				tc = (b > c ? d / b : e / c); // use the largest denominator
			}
			else {
				sc = (b * e - c * d) / dotP;
				tc = (a * e - b * d) / dotP;
			}
			// get the difference of the two closest points
			glm::vec3 dP = w + sc * dir - tc * v; // =  L1(sc) - L2(tc)
			if (glm::length(dP) > radius)
				return;
#pragma endregion

			const auto distance = glm::distance(
				glm::vec3(cameraLtw.m_value[3]),
				glm::vec3(center));
			std::lock_guard<std::mutex> lock(writeMutex);
			if (distance < minDistance) {
				minDistance = distance;
				lengthFactor = glm::clamp(1.0f - tc, 0.0f, 1.0f);
				currentFocusingNodeHandle = sortedNodeList[i];
			}
			}, results);
		for (auto& i : results) i.wait();
		if (currentFocusingNodeHandle != -1) {
			SetSelectedNode(skeleton, currentFocusingNodeHandle, selectedNodeHandle,
				hierarchyList);
			changed = true;
#pragma endregion
		}
		return changed;
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void TreeVisualizer::PeekNodeInspectionGui(
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		NodeHandle nodeHandle, NodeHandle& selectedNodeHandle, std::vector<NodeHandle>& hierarchyList,
		const unsigned& hierarchyLevel) {
		const int index = hierarchyList.size() - hierarchyLevel - 1;
		if (!hierarchyList.empty() && index >= 0 &&
			index < hierarchyList.size() &&
			hierarchyList[index] == nodeHandle) {
			ImGui::SetNextItemOpen(true);
		}
		const bool opened = ImGui::TreeNodeEx(("Handle: " + std::to_string(nodeHandle)).c_str(),
			ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
			ImGuiTreeNodeFlags_NoAutoOpenOnLog |
			(selectedNodeHandle == nodeHandle ? ImGuiTreeNodeFlags_Framed
				: ImGuiTreeNodeFlags_FramePadding));
		if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
			SetSelectedNode(skeleton, nodeHandle, selectedNodeHandle, hierarchyList);
		}
		if (opened) {
			ImGui::TreePush(std::to_string(nodeHandle).c_str());
			const auto& internode = skeleton.PeekNode(nodeHandle);
			const auto& internodeChildren = internode.RefChildHandles();
			for (const auto& child : internodeChildren) {
				PeekNodeInspectionGui(skeleton, child, selectedNodeHandle, hierarchyList, hierarchyLevel + 1);
			}
			ImGui::TreePop();
		}
	}
	

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void TreeVisualizer::SetSelectedNode(
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		NodeHandle nodeHandle, NodeHandle& selectedNodeHandle, std::vector<NodeHandle>& hierarchyList) {
		if (nodeHandle != selectedNodeHandle) {
			hierarchyList.clear();
			if (nodeHandle < 0) {
				selectedNodeHandle = -1;
			}
			else {
				selectedNodeHandle = nodeHandle;
				auto walker = nodeHandle;
				while (walker != -1) {
					hierarchyList.push_back(walker);
					const auto& internode = skeleton.PeekNode(walker);
					walker = internode.GetParentHandle();
				}
			}
		}
	}

	template<typename SkeletonData, typename FlowData, typename NodeData>
	void TreeVisualizer::SyncMatrices(const Skeleton<SkeletonData, FlowData, NodeData>& skeleton, const std::shared_ptr<ParticleInfoList>& particleInfoList, NodeHandle& selectedNodeHandle,
		float& lengthFactor) {

		const auto& sortedNodeList = skeleton.RefSortedNodeList();
		auto& matrices = particleInfoList->m_particleInfos;
		particleInfoList->SetPendingUpdate();
		matrices.resize(sortedNodeList.size() + 1);
		Jobs::ParallelFor(sortedNodeList.size(), [&](unsigned i) {
			auto nodeHandle = sortedNodeList[i];
			const auto& node = skeleton.PeekNode(nodeHandle);
			glm::vec3 position = node.m_info.m_globalPosition;
			const auto direction = glm::normalize(node.m_info.m_globalRotation * glm::vec3(0, 0, -1));
			auto rotation = glm::quatLookAt(
				direction, glm::vec3(direction.y, direction.z, direction.x));
			rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
			const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
			matrices[i + 1].m_instanceMatrix.m_value =
				glm::translate(position + (node.m_info.m_length / 2.0f) * direction) *
				rotationTransform *
				glm::scale(glm::vec3(
					node.m_info.m_thickness * 2.0f,
					node.m_info.m_length,
					node.m_info.m_thickness * 2.0f));
			if (nodeHandle == selectedNodeHandle) {
				const glm::vec3 selectedCenter =
					position + (node.m_info.m_length * lengthFactor) * direction;
				matrices[0].m_instanceMatrix.m_value = glm::translate(selectedCenter) *
					rotationTransform *
					glm::scale(glm::vec3(
						2.0f * node.m_info.m_thickness + 0.001f,
						node.m_info.m_length / 5.0f,
						2.0f * node.m_info.m_thickness + 0.001f));
				matrices[0].m_instanceColor = glm::vec4(1.0f);
			}
			});
	}
}