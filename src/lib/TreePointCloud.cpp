#include "TreePointCloud.hpp"
#include <unordered_set>
#include "Graphics.hpp"
#include "EcoSysLabLayer.hpp"
#include "rapidcsv.h"
using namespace EcoSysLab;

void TreePointCloud::ApplyCurve(const OperatorBranch& branch) {
	auto& skeleton = m_skeletons[branch.m_skeletonIndex];
	const auto chainAmount = branch.m_chainNodeHandles.size();
	for (int i = 0; i < chainAmount; i++) {
		auto& node = skeleton.RefNode(branch.m_chainNodeHandles[i]);
		node.m_data.m_globalEndPosition = branch.m_bezierCurve.GetPoint(static_cast<float>(i + 1) / chainAmount);
		node.m_info.m_thickness = branch.m_thickness;
		node.m_data.m_branchHandle = branch.m_handle;
		node.m_info.m_color = glm::vec4(branch.m_color, 1.0f);
	}
}

void TreePointCloud::BuildVoxelGrid()
{
	m_scatterPointsVoxelGrid.Initialize(2.0f * m_connectivityGraphSettings.m_pointPointConnectionDetectionRadius, m_min, m_max);
	m_allocatedPointsVoxelGrid.Initialize(2.0f * m_connectivityGraphSettings.m_pointPointConnectionDetectionRadius, m_min, m_max);
	
	m_branchEndsVoxelGrid.Initialize(2.0f * m_connectivityGraphSettings.m_pointPointConnectionDetectionRadius, m_min, m_max);
	for (auto& point : m_allocatedPoints) {
		point.m_branchHandle = point.m_nodeHandle = point.m_skeletonIndex = -1;
	}
	for (auto& point : m_scatteredPoints) {
		point.m_neighborScatterPoints.clear();
		point.m_p3.clear();
		point.m_p0.clear();
		PointData voxel;
		voxel.m_handle = point.m_handle;
		voxel.m_position = point.m_position;
		m_scatterPointsVoxelGrid.Ref(point.m_position).emplace_back(voxel);
	}
	for (auto& point : m_allocatedPoints) {
		PointData voxel;
		voxel.m_handle = point.m_handle;
		voxel.m_position = point.m_position;
		m_allocatedPointsVoxelGrid.Ref(point.m_position).emplace_back(voxel);
	}
	for (auto& predictedBranch : m_predictedBranches) {
		predictedBranch.m_pointsToP3.clear();
		predictedBranch.m_p3ToP0.clear();

		predictedBranch.m_pointsToP0.clear();
		predictedBranch.m_p3ToP3.clear();
		predictedBranch.m_p0ToP0.clear();
		predictedBranch.m_p0ToP3.clear();

		BranchEndData voxel;
		voxel.m_branchHandle = predictedBranch.m_handle;
		voxel.m_position = predictedBranch.m_bezierCurve.m_p0;
		voxel.m_isP0 = true;
		m_branchEndsVoxelGrid.Ref(predictedBranch.m_bezierCurve.m_p0).emplace_back(voxel);
		voxel.m_position = predictedBranch.m_bezierCurve.m_p3;
		voxel.m_isP0 = false;
		m_branchEndsVoxelGrid.Ref(predictedBranch.m_bezierCurve.m_p3).emplace_back(voxel);
	}
}

bool TreePointCloud::DirectConnectionCheck(const BezierCurve& parentCurve, const BezierCurve& childCurve, bool reverse)
{
	const auto parentPA = parentCurve.m_p0;
	const auto parentPB = parentCurve.m_p3;
	const auto childPA = reverse ? childCurve.m_p3 : childCurve.m_p0;
	const auto childPB = reverse ? childCurve.m_p0 : childCurve.m_p3;
	const auto dotP = glm::dot(glm::normalize(parentPB - parentPA),
		glm::normalize(childPB - childPA));
	if (dotP < glm::cos(glm::radians(m_connectivityGraphSettings.m_directionConnectionAngleLimit))) return false;
	if (m_connectivityGraphSettings.m_zigzagCheck) {
		auto shortenedParentP0 = parentCurve.GetPoint(m_connectivityGraphSettings.m_zigzagBranchShortening);
		auto shortenedParentP3 = parentCurve.GetPoint(1.0f - m_connectivityGraphSettings.m_zigzagBranchShortening);
		auto shortenedChildP0 = childCurve.GetPoint(m_connectivityGraphSettings.m_zigzagBranchShortening);
		auto shortenedChildP3 = childCurve.GetPoint(1.0f - m_connectivityGraphSettings.m_zigzagBranchShortening);

		const auto dotC0 = glm::dot(glm::normalize(shortenedChildP3 - shortenedChildP0), glm::normalize(shortenedParentP3 - shortenedChildP0));
		//const auto dotC3 = glm::dot(glm::normalize(shortenedChildP0 - shortenedChildP3), glm::normalize(shortenedParentP3 - shortenedChildP3));
		const auto dotP3 = glm::dot(glm::normalize(shortenedChildP0 - shortenedParentP3), glm::normalize(shortenedParentP0 - shortenedParentP3));
		//const auto dotP0 = glm::dot(glm::normalize(shortenedChildP0 - shortenedParentP0), glm::normalize(shortenedParentP3 - shortenedParentP0));
		if (dotC0 > 0 || dotP3 > 0/* && dotP0 < 0*/) return false;
	}
	if (m_connectivityGraphSettings.m_parallelShiftCheck)
	{
		const auto parentDirection = glm::normalize(parentPA - parentPB);
		const auto projectedC0 = glm::closestPointOnLine(childPA, parentPA + 10.0f * parentDirection, parentPB - 10.0f * parentDirection);
		const auto childLength = glm::distance(childPA, childPB);
		const auto parentLength = glm::distance(parentPA, parentPB);
		const auto projectedLength = glm::distance(projectedC0, childPA);
		if (projectedLength > m_connectivityGraphSettings.m_parallelShiftLimitRange * childLength
			|| projectedLength > m_connectivityGraphSettings.m_parallelShiftLimitRange * parentLength) return false;
	}
	if (m_connectivityGraphSettings.m_pointCheckRadius > 0.0f) {
		const auto middlePoint = (childPA + parentPB) * 0.5f;
		if (!HasPoints(middlePoint, m_allocatedPointsVoxelGrid, m_connectivityGraphSettings.m_pointCheckRadius) && !HasPoints(middlePoint, m_scatterPointsVoxelGrid, m_connectivityGraphSettings.m_pointCheckRadius)) return false;
	}
	return true;

}

void TreePointCloud::FindPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid,
	float radius,
	const std::function<void(const PointData& voxel)>& func)
{
	pointVoxelGrid.ForEach(position, radius, [&](const std::vector<PointData>& voxels) {
		for (const auto& voxel : voxels) {
			if (glm::distance(position, voxel.m_position) > radius) continue;
			func(voxel);
		}
		});
}

bool TreePointCloud::HasPoints(const glm::vec3& position, VoxelGrid<std::vector<PointData>>& pointVoxelGrid,
	float radius)
{
	bool retVal = false;
	pointVoxelGrid.ForEach(position, radius, [&](const std::vector<PointData>& voxels) {
		if (retVal) return;
		for (const auto& voxel : voxels) {
			if (glm::distance(position, voxel.m_position) <= radius) retVal = true;
		}
		});
	return retVal;
}

void
TreePointCloud::ForEachBranchEnd(const glm::vec3& position, VoxelGrid<std::vector<BranchEndData>>& branchEndsVoxelGrid,
	float radius,
	const std::function<void(const BranchEndData& voxel)>& func)
{
	branchEndsVoxelGrid.ForEach(position, radius, [&](const std::vector<BranchEndData>& branchEnds) {
		for (const auto& branchEnd : branchEnds) {
			if (glm::distance(position, branchEnd.m_position) > radius) continue;
			func(branchEnd);
		}
		});
}

void TreePointCloud::CalculateNodeTransforms(ReconstructionSkeleton& skeleton)
{
	skeleton.m_min = glm::vec3(FLT_MAX);
	skeleton.m_max = glm::vec3(FLT_MIN);
	for (const auto& nodeHandle : skeleton.RefSortedNodeList()) {
		auto& node = skeleton.RefNode(nodeHandle);
		auto& nodeInfo = node.m_info;
		auto& nodeData = node.m_data;
		if (node.GetParentHandle() != -1) {
			auto& parentInfo = skeleton.RefNode(node.GetParentHandle()).m_info;
			nodeInfo.m_globalDirection = glm::normalize(nodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
			nodeInfo.m_globalPosition =
				parentInfo.m_globalPosition
				+ parentInfo.m_length * parentInfo.m_globalDirection;
			auto parentRegulatedUp = parentInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			auto regulatedUp = glm::normalize(glm::cross(glm::cross(nodeInfo.m_globalDirection, parentRegulatedUp), nodeInfo.m_globalDirection));
			nodeInfo.m_regulatedGlobalRotation = glm::quatLookAt(nodeInfo.m_globalDirection, regulatedUp);
		}
		skeleton.m_min = glm::min(skeleton.m_min, nodeInfo.m_globalPosition);
		skeleton.m_max = glm::max(skeleton.m_max, nodeInfo.m_globalPosition);
		const auto endPosition = nodeInfo.m_globalPosition
			+ nodeInfo.m_length * nodeInfo.m_globalDirection;
		skeleton.m_min = glm::min(skeleton.m_min, endPosition);
		skeleton.m_max = glm::max(skeleton.m_max, endPosition);
	}
}

void TreePointCloud::BuildConnectionBranch(const BranchHandle processingBranchHandle, NodeHandle& prevNodeHandle)
{
	m_operatingBranches.emplace_back();
	auto& processingBranch = m_operatingBranches[processingBranchHandle];
	auto& skeleton = m_skeletons[processingBranch.m_skeletonIndex];
	auto& connectionBranch = m_operatingBranches.back();
	auto& parentBranch = m_operatingBranches[processingBranch.m_parentHandle];
	assert(parentBranch.m_skeletonIndex == processingBranch.m_skeletonIndex);
	connectionBranch.m_color = parentBranch.m_color;
	connectionBranch.m_handle = m_operatingBranches.size() - 1;
	connectionBranch.m_skeletonIndex = processingBranch.m_skeletonIndex;
	connectionBranch.m_thickness = (parentBranch.m_thickness + processingBranch.m_thickness) * 0.5f;
	connectionBranch.m_childHandles.emplace_back(processingBranch.m_handle);
	connectionBranch.m_parentHandle = processingBranch.m_parentHandle;
	processingBranch.m_parentHandle = connectionBranch.m_handle;
	for (int& childHandle : parentBranch.m_childHandles)
	{
		if (childHandle == processingBranch.m_handle) {
			childHandle = connectionBranch.m_handle;
			break;
		}
	}

	NodeHandle bestPrevNodeHandle = parentBranch.m_chainNodeHandles.back();
	float dotMax = -1.0f;
	glm::vec3 connectionBranchStartPosition = parentBranch.m_bezierCurve.m_p3;
	NodeHandle backTrackWalker = bestPrevNodeHandle;
	int branchBackTrackCount = 0;
	BranchHandle prevBranchHandle = processingBranch.m_parentHandle;
	for (int i = 0; i < m_reconstructionSettings.m_nodeBackTrackLimit; i++)
	{
		if (backTrackWalker == -1) break;
		auto& node = skeleton.PeekNode(backTrackWalker);
		if (node.m_data.m_branchHandle != prevBranchHandle) {
			branchBackTrackCount++;
			prevBranchHandle = node.m_data.m_branchHandle;
		}
		if (branchBackTrackCount > m_reconstructionSettings.m_branchBackTrackLimit) break;
		const auto nodeEndPosition = node.m_data.m_globalEndPosition;
		//fad
		const auto dotVal = glm::dot(glm::normalize(processingBranch.m_bezierCurve.m_p3 - processingBranch.m_bezierCurve.m_p0),
			glm::normalize(processingBranch.m_bezierCurve.m_p0 - nodeEndPosition));
		if (dotVal > dotMax)
		{
			dotMax = dotVal;
			bestPrevNodeHandle = backTrackWalker;
			connectionBranchStartPosition = nodeEndPosition;
		}
		backTrackWalker = node.GetParentHandle();
	}

	connectionBranch.m_bezierCurve.m_p0 = connectionBranchStartPosition;

	connectionBranch.m_bezierCurve.m_p3 = processingBranch.m_bezierCurve.m_p0;

	connectionBranch.m_bezierCurve.m_p1 = glm::mix(connectionBranch.m_bezierCurve.m_p0,
		connectionBranch.m_bezierCurve.m_p3, 0.25f);

	connectionBranch.m_bezierCurve.m_p2 = glm::mix(connectionBranch.m_bezierCurve.m_p0,
		connectionBranch.m_bezierCurve.m_p3, 0.75f);

	prevNodeHandle = bestPrevNodeHandle;

	auto connectionFirstNodeHandle = skeleton.Extend(prevNodeHandle, !processingBranch.m_apical);
	connectionBranch.m_chainNodeHandles.emplace_back(connectionFirstNodeHandle);
	prevNodeHandle = connectionFirstNodeHandle;
	const float connectionChainLength = connectionBranch.m_bezierCurve.GetLength();
	const int connectionChainAmount = glm::max(2, static_cast<int>(connectionChainLength /
		m_reconstructionSettings.m_internodeLength));
	for (int i = 1; i < connectionChainAmount; i++) {
		prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
		connectionBranch.m_chainNodeHandles.emplace_back(prevNodeHandle);

	}
	ApplyCurve(connectionBranch);
	prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
	processingBranch.m_chainNodeHandles.emplace_back(prevNodeHandle);
}

void TreePointCloud::Unlink(const BranchHandle childHandle, const BranchHandle parentHandle)
{
	auto& childBranch = m_operatingBranches[childHandle];
	auto& parentBranch = m_operatingBranches[parentHandle];
	//Establish relationship
	childBranch.m_parentHandle = -1;
	childBranch.m_used = false;

	for(int i = 0; i < parentBranch.m_childHandles.size(); i++)
	{
		if(childHandle == parentBranch.m_childHandles[i])
		{
			parentBranch.m_childHandles[i] = parentBranch.m_childHandles.back();
			parentBranch.m_childHandles.pop_back();
			break;
		}
	}
}

void TreePointCloud::Link(const BranchHandle childHandle, const BranchHandle parentHandle)
{
	auto& childBranch = m_operatingBranches[childHandle];
	auto& parentBranch = m_operatingBranches[parentHandle];
	//Establish relationship
	childBranch.m_parentHandle = parentHandle;
	childBranch.m_used = true;
	parentBranch.m_childHandles.emplace_back(childHandle);
}

void TreePointCloud::GetSortedBranchList(BranchHandle branchHandle, std::vector<BranchHandle>& list)
{
	const auto childHandles = m_operatingBranches[branchHandle].m_childHandles;
	list.push_back(branchHandle);
	for (const auto& childHandle : childHandles)
	{
		GetSortedBranchList(childHandle, list);
	}
}

void TreePointCloud::ConnectBranches(const BranchHandle branchHandle)
{
	const auto childHandles = m_operatingBranches[branchHandle].m_childHandles;
	for (const auto& childHandle : childHandles) {
		//Connect branches.
		NodeHandle prevNodeHandle = -1;
		BuildConnectionBranch(childHandle, prevNodeHandle);
		auto& childBranch = m_operatingBranches[childHandle];
		const float chainLength = childBranch.m_bezierCurve.GetLength();
		const int chainAmount = glm::max(2, static_cast<int>(chainLength /
			m_reconstructionSettings.m_internodeLength));
		auto& skeleton = m_skeletons[childBranch.m_skeletonIndex];
		for (int i = 1; i < chainAmount; i++) {
			prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
			childBranch.m_chainNodeHandles.emplace_back(prevNodeHandle);
		}
		ApplyCurve(childBranch);
	}
	for (const auto& childHandle : childHandles)
	{
		ConnectBranches(childHandle);
	}
}

void TreePointCloud::ImportGraph(const std::filesystem::path& path, float scaleFactor) {
	if (!std::filesystem::exists(path)) {
		EVOENGINE_ERROR("Not exist!");
		return;
	}
	try {
		std::ifstream stream(path.string());
		std::stringstream stringStream;
		stringStream << stream.rdbuf();
		YAML::Node in = YAML::Load(stringStream.str());

		const auto& tree = in["Tree"];
		const auto& scatterPoints = tree["Scatter Points"];
		const auto& treeParts = tree["Tree Parts"];

		m_scatteredPoints.resize(scatterPoints.size());

		for (int i = 0; i < scatterPoints.size(); i++) {
			auto& point = m_scatteredPoints[i];
			point.m_position = scatterPoints[i].as<glm::vec3>() * scaleFactor;

			point.m_handle = i;
			point.m_neighborScatterPoints.clear();
		}

		m_predictedBranches.clear();
		m_operatingBranches.clear();
		m_treeParts.clear();
		m_allocatedPoints.clear();
		m_skeletons.clear();
		m_scatterPointToBranchEndConnections.clear();
		m_scatterPointToBranchStartConnections.clear();
		m_scatterPointsConnections.clear();
		m_candidateBranchConnections.clear();
		m_reversedCandidateBranchConnections.clear();
		m_filteredBranchConnections.clear();
		m_branchConnections.clear();
		m_min = glm::vec3(FLT_MAX);
		m_max = glm::vec3(FLT_MIN);
		float minHeight = 999.0f;
		for (int i = 0; i < treeParts.size(); i++) {
			const auto& inTreeParts = treeParts[i];
			auto& treePart = m_treeParts.emplace_back();
			treePart.m_handle = m_treeParts.size() - 1;
			try {
				if (inTreeParts["Color"]) treePart.m_color = inTreeParts["Color"].as<glm::vec3>() / 255.0f;
			}
			catch (const std::exception& e)
			{
				EVOENGINE_ERROR("Color is wrong at node " + std::to_string(i) + ": " + std::string(e.what()));
			}
			for (const auto& inBranch : inTreeParts["Branches"]) {
				auto& branch = m_predictedBranches.emplace_back();
				branch.m_bezierCurve.m_p0 = inBranch["Start Pos"].as<glm::vec3>() * scaleFactor;
				branch.m_bezierCurve.m_p3 = inBranch["End Pos"].as<glm::vec3>() * scaleFactor;
				branch.m_color = treePart.m_color;
				auto cPLength = glm::distance(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3) * 0.3f;
				branch.m_bezierCurve.m_p1 =
					glm::normalize(inBranch["Start Dir"].as<glm::vec3>()) * cPLength + branch.m_bezierCurve.m_p0;
				branch.m_bezierCurve.m_p2 =
					branch.m_bezierCurve.m_p3 - glm::normalize(inBranch["End Dir"].as<glm::vec3>()) * cPLength;
				if (glm::any(glm::isnan(branch.m_bezierCurve.m_p1)))
				{
					branch.m_bezierCurve.m_p1 = glm::mix(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3, 0.25f);
				}
				if (glm::any(glm::isnan(branch.m_bezierCurve.m_p2)))
				{
					branch.m_bezierCurve.m_p2 = glm::mix(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3, 0.75f);
				}
				branch.m_startThickness = inBranch["Start Radius"].as<float>() * scaleFactor;
				branch.m_endThickness = inBranch["End Radius"].as<float>() * scaleFactor;
				branch.m_handle = m_predictedBranches.size() - 1;
				treePart.m_branchHandles.emplace_back(branch.m_handle);
				branch.m_treePartHandle = treePart.m_handle;
				minHeight = glm::min(minHeight, branch.m_bezierCurve.m_p0.y);
				minHeight = glm::min(minHeight, branch.m_bezierCurve.m_p3.y);
			}

			for (const auto& inAllocatedPoint : inTreeParts["Allocated Points"]) {
				auto& allocatedPoint = m_allocatedPoints.emplace_back();
				allocatedPoint.m_color = treePart.m_color;
				allocatedPoint.m_position = inAllocatedPoint.as<glm::vec3>() * scaleFactor;
				allocatedPoint.m_handle = m_allocatedPoints.size() - 1;
				allocatedPoint.m_treePartHandle = treePart.m_handle;
				allocatedPoint.m_branchHandle = -1;
				treePart.m_allocatedPoints.emplace_back(allocatedPoint.m_handle);
			}
		}
		for (auto& scatterPoint : m_scatteredPoints) {
			scatterPoint.m_position.y -= minHeight;

			m_min = glm::min(m_min, scatterPoint.m_position);
			m_max = glm::max(m_max, scatterPoint.m_position);
		}
		for (auto& predictedBranch : m_predictedBranches) {
			predictedBranch.m_bezierCurve.m_p0.y -= minHeight;
			predictedBranch.m_bezierCurve.m_p1.y -= minHeight;
			predictedBranch.m_bezierCurve.m_p2.y -= minHeight;
			predictedBranch.m_bezierCurve.m_p3.y -= minHeight;
		}
		for (auto& allocatedPoint : m_allocatedPoints) {
			allocatedPoint.m_position.y -= minHeight;

			m_min = glm::min(m_min, allocatedPoint.m_position);
			m_max = glm::max(m_max, allocatedPoint.m_position);

			const auto& treePart = m_treeParts[allocatedPoint.m_treePartHandle];
			std::map<float, BranchHandle> distances;
			for (const auto& branchHandle : treePart.m_branchHandles)
			{
				const auto& branch = m_predictedBranches[branchHandle];
				const auto distance0 = glm::distance(allocatedPoint.m_position, branch.m_bezierCurve.m_p0);
				const auto distance3 = glm::distance(allocatedPoint.m_position, branch.m_bezierCurve.m_p3);
				distances[distance0] = branchHandle;
				distances[distance3] = branchHandle;
			}
			allocatedPoint.m_branchHandle = distances.begin()->second;
			m_predictedBranches[allocatedPoint.m_branchHandle].m_allocatedPoints.emplace_back(allocatedPoint.m_handle);
		}
		for (auto& predictedBranch : m_predictedBranches) {
			m_min = glm::min(m_min, predictedBranch.m_bezierCurve.m_p0);
			m_max = glm::max(m_max, predictedBranch.m_bezierCurve.m_p0);
			m_min = glm::min(m_min, predictedBranch.m_bezierCurve.m_p3);
			m_max = glm::max(m_max, predictedBranch.m_bezierCurve.m_p3);

			if (!predictedBranch.m_allocatedPoints.empty()) {
				const auto& origin = predictedBranch.m_bezierCurve.m_p0;
				const auto normal = glm::normalize(predictedBranch.m_bezierCurve.m_p3 - origin);
				const auto xAxis = glm::vec3(normal.y, normal.z, normal.x);
				const auto yAxis = glm::vec3(normal.z, normal.x, normal.y);
				auto positionAvg = glm::vec2(0.0f);
				for (const auto& pointHandle : predictedBranch.m_allocatedPoints)
				{
					auto& point = m_allocatedPoints[pointHandle];
					const auto v = predictedBranch.m_bezierCurve.m_p0 - point.m_position;
					const auto d = glm::dot(v, normal);
					const auto p = v + d * normal;
					const auto x = glm::distance(origin, glm::closestPointOnLine(p, origin, origin + 10.0f * xAxis));
					const auto y = glm::distance(origin, glm::closestPointOnLine(p, origin, origin + 10.0f * yAxis));
					point.m_planePosition = glm::vec2(x, y);
					positionAvg += point.m_planePosition;
				}
				positionAvg /= predictedBranch.m_allocatedPoints.size();
				auto distanceAvg = 0.0f;
				for (const auto& pointHandle : predictedBranch.m_allocatedPoints)
				{
					auto& point = m_allocatedPoints[pointHandle];
					point.m_planePosition -= positionAvg;
					distanceAvg += glm::length(point.m_planePosition);
				}
				distanceAvg /= predictedBranch.m_allocatedPoints.size();
				predictedBranch.m_branchThickness = distanceAvg * 2.0f;
			}
			else
			{
				predictedBranch.m_branchThickness = (predictedBranch.m_startThickness + predictedBranch.m_endThickness) * 0.5f;
			}
		}

		auto center = (m_min + m_max) / 2.0f;
		auto newMin = center + (m_min - center) * 1.25f;
		auto newMax = center + (m_max - center) * 1.25f;
		m_min = newMin;
		m_max = newMax;

		BuildVoxelGrid();
	}
	catch (std::exception e) {
		EVOENGINE_ERROR("Failed to load!");
	}
}

void TreePointCloud::ExportForestOBJ(const std::filesystem::path& path) const
{
	if (path.extension() == ".obj") {
		std::ofstream of;
		of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
		if (of.is_open()) {
			std::string start = "#Forest OBJ exporter, by Bosheng Li";
			start += "\n";
			of.write(start.c_str(), start.size());
			of.flush();
			unsigned startIndex = 1;
			const auto branchMeshes = GenerateForestBranchMeshes();
			if (!branchMeshes.empty()) {
				unsigned treeIndex = 0;
				for (auto& mesh : branchMeshes) {
					auto& vertices = mesh->UnsafeGetVertices();
					auto& triangles = mesh->UnsafeGetTriangles();
					if (!vertices.empty() && !triangles.empty()) {
						std::string header =
							"#Vertices: " + std::to_string(vertices.size()) +
							", tris: " + std::to_string(triangles.size());
						header += "\n";
						of.write(header.c_str(), header.size());
						of.flush();
						std::stringstream data;
						data << "o tree " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
						for (auto i = 0; i < vertices.size(); i++) {
							auto& vertexPosition = vertices.at(i).m_position;
							auto& color = vertices.at(i).m_color;
							data << "v " + std::to_string(vertexPosition.x) + " " +
								std::to_string(vertexPosition.y) + " " +
								std::to_string(vertexPosition.z) + " " +
								std::to_string(color.x) + " " + std::to_string(color.y) + " " +
								std::to_string(color.z) + "\n";
						}
						for (const auto& vertex : vertices) {
							data << "vt " + std::to_string(vertex.m_texCoord.x) + " " +
								std::to_string(vertex.m_texCoord.y) + "\n";
						}
						// data += "s off\n";
						data << "# List of indices for faces vertices, with (x, y, z).\n";
						for (auto i = 0; i < triangles.size(); i++) {
							const auto triangle = triangles[i];
							const auto f1 = triangle.x + startIndex;
							const auto f2 = triangle.y + startIndex;
							const auto f3 = triangle.z + startIndex;
							data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" +
								std::to_string(f1) + " " + std::to_string(f2) + "/" +
								std::to_string(f2) + "/" + std::to_string(f2) + " " +
								std::to_string(f3) + "/" + std::to_string(f3) + "/" +
								std::to_string(f3) + "\n";
						}
#pragma endregion
						const auto result = data.str();
						of.write(result.c_str(), result.size());
						of.flush();
						startIndex += vertices.size();
						treeIndex++;
					}
				}
			}
			if (m_treeMeshGeneratorSettings.m_enableFoliage) {
				const auto foliageMeshes = GenerateFoliageMeshes();
				if (!foliageMeshes.empty()) {

					unsigned treeIndex = 0;
					for (auto& mesh : foliageMeshes) {
						auto& vertices = mesh->UnsafeGetVertices();
						auto& triangles = mesh->UnsafeGetTriangles();
						if (!vertices.empty() && !triangles.empty()) {
							std::string header =
								"#Vertices: " + std::to_string(vertices.size()) +
								", tris: " + std::to_string(triangles.size());
							header += "\n";
							of.write(header.c_str(), header.size());
							of.flush();
							std::stringstream data;
							data << "o tree " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
							for (auto i = 0; i < vertices.size(); i++) {
								auto& vertexPosition = vertices.at(i).m_position;
								auto& color = vertices.at(i).m_color;
								data << "v " + std::to_string(vertexPosition.x) + " " +
									std::to_string(vertexPosition.y) + " " +
									std::to_string(vertexPosition.z) + " " +
									std::to_string(color.x) + " " + std::to_string(color.y) + " " +
									std::to_string(color.z) + "\n";
							}
							for (const auto& vertex : vertices) {
								data << "vt " + std::to_string(vertex.m_texCoord.x) + " " +
									std::to_string(vertex.m_texCoord.y) + "\n";
							}
							// data += "s off\n";
							data << "# List of indices for faces vertices, with (x, y, z).\n";
							for (auto i = 0; i < triangles.size(); i++) {
								const auto triangle = triangles[i];
								const auto f1 = triangle.x + startIndex;
								const auto f2 = triangle.y + startIndex;
								const auto f3 = triangle.z + startIndex;
								data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" +
									std::to_string(f1) + " " + std::to_string(f2) + "/" +
									std::to_string(f2) + "/" + std::to_string(f2) + " " +
									std::to_string(f3) + "/" + std::to_string(f3) + "/" +
									std::to_string(f3) + "\n";
							}
#pragma endregion
							const auto result = data.str();
							of.write(result.c_str(), result.size());
							of.flush();
							startIndex += vertices.size();
							treeIndex++;
						}
					}
				}
			}
			of.close();
		}
	}
}

void TreePointCloud::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	static Handle previousHandle = 0;
	static std::shared_ptr<ParticleInfoList> allocatedPointInfoList;
	static std::shared_ptr<ParticleInfoList> scatterPointInfoList;
	static std::shared_ptr<ParticleInfoList> nodeInfoList;

	static std::shared_ptr<ParticleInfoList> scatteredPointConnectionInfoList;
	static std::vector<glm::vec3> scatteredPointConnectionsStarts;
	static std::vector<glm::vec3> scatteredPointConnectionsEnds;
	static std::vector<glm::vec4> scatteredPointConnectionColors;

	static std::shared_ptr<ParticleInfoList> candidateBranchConnectionInfoList;
	static std::vector<glm::vec3> candidateBranchConnectionStarts;
	static std::vector<glm::vec3> candidateBranchConnectionEnds;
	static std::vector<glm::vec4> candidateBranchConnectionColors;

	static std::shared_ptr<ParticleInfoList> reversedCandidateBranchConnectionInfoList;
	static std::vector<glm::vec3> reversedCandidateBranchConnectionStarts;
	static std::vector<glm::vec3> reversedCandidateBranchConnectionEnds;
	static std::vector<glm::vec4> reversedCandidateBranchConnectionColors;

	static std::shared_ptr<ParticleInfoList> filteredBranchConnectionInfoList;
	static std::vector<glm::vec3> filteredBranchConnectionStarts;
	static std::vector<glm::vec3> filteredBranchConnectionEnds;
	static std::vector<glm::vec4> filteredBranchConnectionColors;

	static std::shared_ptr<ParticleInfoList> branchConnectionInfoList;
	static std::vector<glm::vec3> branchConnectionStarts;
	static std::vector<glm::vec3> branchConnectionEnds;
	static std::vector<glm::vec4> branchConnectionColors;

	static std::shared_ptr<ParticleInfoList> scatterPointToBranchConnectionInfoList;
	static std::vector<glm::vec3> scatterPointToBranchConnectionStarts;
	static std::vector<glm::vec3> scatterPointToBranchConnectionEnds;
	static std::vector<glm::vec4> scatterPointToBranchConnectionColors;

	static std::shared_ptr<ParticleInfoList> predictedBranchConnectionInfoList;
	static std::vector<glm::vec3> predictedBranchConnectionStarts;
	static std::vector<glm::vec3> predictedBranchConnectionEnds;
	static std::vector<glm::vec4> predictedBranchConnectionColors;


	if (!allocatedPointInfoList) allocatedPointInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!scatterPointInfoList) scatterPointInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!nodeInfoList) nodeInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!scatteredPointConnectionInfoList) scatteredPointConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

	if (!candidateBranchConnectionInfoList) candidateBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!reversedCandidateBranchConnectionInfoList) reversedCandidateBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!filteredBranchConnectionInfoList) filteredBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!branchConnectionInfoList) branchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

	if (!scatterPointToBranchConnectionInfoList) scatterPointToBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!predictedBranchConnectionInfoList) predictedBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

	auto& allocatedPointMatrices = allocatedPointInfoList->m_particleInfos;
	auto& scatterPointMatrices = scatterPointInfoList->m_particleInfos;
	auto& nodeMatrices = nodeInfoList->m_particleInfos;

	static bool enableDebugRendering = true;

	static bool drawAllocatedPoints = true;
	static bool drawpredictedBranches = true;
	static bool drawScatteredPoints = true;
	static bool drawScatteredPointConnections = false;

	static bool drawCandidateConnections = false;
	static bool drawReversedCandidateConnections = false;
	static bool drawFilteredConnections = false;
	static bool drawBranchConnections = true;

	static bool drawScatterPointToBranchConnections = false;
	static bool drawNode = true;
	static float predictedBranchWidth = 0.005f;
	static float connectionWidth = 0.001f;
	static float pointSize = 0.002f;

	static float nodeSize = 1.0f;
	bool refreshData = false;

	static int colorMode = 0;


	static glm::vec4 scatterPointToBranchConnectionColor = glm::vec4(1, 0, 1, 1);
	static glm::vec4 scatterPointColor = glm::vec4(0, 1, 0, 1);
	static glm::vec4 scatteredPointConnectionColor = glm::vec4(1, 1, 1, 1);

	static glm::vec4 candidateBranchConnectionColor = glm::vec4(1, 1, 0, 1);
	static glm::vec4 reversedCandidateBranchConnectionColor = glm::vec4(0, 1, 1, 1);
	static glm::vec4 filteredBranchConnectionColor = glm::vec4(0, 0, 1, 1);
	static glm::vec4 branchConnectionColor = glm::vec4(1, 0, 0, 1);

	static float importScale = 0.1f;
	ImGui::DragFloat("Import scale", &importScale, 0.01f, 0.01f, 10.0f);
	FileUtils::OpenFile("Load YAML", "YAML", { ".yml" }, [&](const std::filesystem::path& path) {
		ImportGraph(path, importScale);
		refreshData = true;
		}, false);

	if (!m_scatteredPoints.empty()) {
		if (ImGui::TreeNodeEx("Graph Settings")) {
			m_connectivityGraphSettings.OnInspect();
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Reconstruction Settings")) {
			m_reconstructionSettings.OnInspect();
			ImGui::TreePop();
		}
		if (ImGui::Button("Rebuild Voxel Grid"))
		{
			BuildVoxelGrid();
		}
		if (ImGui::Button("Build Skeleton")) {
			EstablishConnectivityGraph();
			BuildSkeletons();
			refreshData = true;
		}
		m_treeMeshGeneratorSettings.OnInspect(editorLayer);
		if (ImGui::Button("Form tree mesh")) {
			if (m_branchConnections.empty()) {
				m_skeletons.clear();
				EstablishConnectivityGraph();
			}
			if (m_skeletons.empty()) BuildSkeletons();
			FormGeometryEntity();
		}
		if (ImGui::Button("Clear meshes"))
		{
			ClearMeshes();
		}
		FileUtils::SaveFile("Export all trees as OBJ", "OBJ", { ".obj" }, [&](const std::filesystem::path& path) {
			ExportForestOBJ(path);
			}, false);
		if (ImGui::Button("Refresh Data")) {
			refreshData = true;
		}
	}

	ImGui::Checkbox("Debug Rendering", &enableDebugRendering);
	if (enableDebugRendering) {
		GizmoSettings gizmoSettings;
		if (ImGui::Combo("Color mode", { "TreePart", "Branch", "Node" }, colorMode)) refreshData = true;
		if (ImGui::TreeNode("Render settings")) {
			if (ImGui::DragFloat("Branch width", &predictedBranchWidth, 0.0001f, 0.0001f, 1.0f, "%.4f")) refreshData = true;
			if (ImGui::DragFloat("Connection width", &connectionWidth, 0.0001f, 0.0001f, 1.0f, "%.4f")) refreshData = true;
			if (ImGui::DragFloat("Point size", &pointSize, 0.0001f, 0.0001f, 1.0f, "%.4f")) refreshData = true;
			if (ImGui::DragFloat("Node size", &nodeSize, 0.01f, 0.1f, 10.0f)) refreshData = true;

			if (ImGui::Checkbox("Render branches", &drawpredictedBranches)) refreshData = true;
			if (ImGui::Checkbox("Render allocated points", &drawAllocatedPoints)) refreshData = true;
			if (ImGui::Checkbox("Render scattered points", &drawScatteredPoints)) refreshData = true;
			if (drawScatteredPoints) {
				if (ImGui::ColorEdit4("Scatter Point color", &scatterPointColor.x)) refreshData = true;
				if (ImGui::Checkbox("Render Point-Point links", &drawScatteredPointConnections)) refreshData = true;
				if (ImGui::Checkbox("Render Point-Branch links", &drawScatterPointToBranchConnections)) refreshData = true;
				if (drawScatteredPointConnections && ImGui::ColorEdit4("Point-Point links color", &scatteredPointConnectionColor.x)) refreshData = true;
				if (drawScatterPointToBranchConnections && ImGui::ColorEdit4("Point-Branch links color", &scatterPointToBranchConnectionColor.x)) refreshData = true;
			}

			if (ImGui::Checkbox("Render candidate connections", &drawCandidateConnections)) refreshData = true;
			if (drawCandidateConnections && ImGui::ColorEdit4("Candidate connection color", &candidateBranchConnectionColor.x))
				refreshData = true;
			if (ImGui::Checkbox("Render reversed candidate connections", &drawReversedCandidateConnections)) refreshData = true;
			if (drawReversedCandidateConnections && ImGui::ColorEdit4("Reversed candidate connection color", &reversedCandidateBranchConnectionColor.x))
				refreshData = true;
			if (ImGui::Checkbox("Render filtered connections", &drawFilteredConnections)) refreshData = true;
			if (drawFilteredConnections && ImGui::ColorEdit4("Filtered Connection Color", &filteredBranchConnectionColor.x))
				refreshData = true;
			if (ImGui::Checkbox("Render connections", &drawBranchConnections)) refreshData = true;
			if (drawBranchConnections && ImGui::ColorEdit4("Branch Connection Color", &branchConnectionColor.x))
				refreshData = true;

			if (ImGui::Checkbox("Render nodes", &drawNode)) refreshData = true;
			gizmoSettings.m_drawSettings.OnInspect();

			ImGui::TreePop();
		}
		if (GetHandle() != previousHandle) refreshData = true;

		if (refreshData) {
			previousHandle = GetHandle();
			const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();

			allocatedPointMatrices.resize(m_allocatedPoints.size());

			predictedBranchConnectionStarts.resize(m_predictedBranches.size());
			predictedBranchConnectionEnds.resize(m_predictedBranches.size());
			predictedBranchConnectionColors.resize(m_predictedBranches.size());

			allocatedPointInfoList->SetPendingUpdate();
			scatterPointInfoList->SetPendingUpdate();
			nodeInfoList->SetPendingUpdate();

			nodeMatrices.clear();
			switch (colorMode) {
			case 0: {
				//TreePart
				for (int i = 0; i < m_allocatedPoints.size(); i++) {
					allocatedPointMatrices[i].m_instanceMatrix.m_value =
						glm::translate(m_allocatedPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
					allocatedPointMatrices[i].m_instanceColor = glm::vec4(m_allocatedPoints[i].m_color, 1.0f);
				}

				for (int i = 0; i < m_predictedBranches.size(); i++) {
					predictedBranchConnectionStarts[i] = m_predictedBranches[i].m_bezierCurve.m_p0;
					predictedBranchConnectionEnds[i] = m_predictedBranches[i].m_bezierCurve.m_p3;
					predictedBranchConnectionColors[i] = glm::vec4(m_predictedBranches[i].m_color, 1.0f);
				}
				predictedBranchConnectionInfoList->ApplyConnections(predictedBranchConnectionStarts, predictedBranchConnectionEnds, predictedBranchConnectionColors, predictedBranchWidth);
				for (const auto& skeleton : m_skeletons) {
					const auto& nodeList = skeleton.RefSortedNodeList();
					const auto startIndex = nodeMatrices.size();
					nodeMatrices.resize(startIndex + nodeList.size());
					for (int i = 0; i < nodeList.size(); i++) {
						const auto& node = skeleton.PeekNode(nodeList[i]);
						nodeMatrices[startIndex + i].m_instanceMatrix.m_value = glm::translate(node.m_info.m_globalPosition + skeleton.m_data.m_rootPosition) *
							glm::scale(glm::vec3(node.m_info.m_thickness));
						nodeMatrices[startIndex + i].m_instanceColor = glm::vec4(1.0f);
					}
				}
			}
				  break;
			case 1: {
				//Branch
				for (int i = 0; i < m_allocatedPoints.size(); i++) {
					allocatedPointMatrices[i].m_instanceMatrix.m_value =
						glm::translate(m_allocatedPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
					if (m_allocatedPoints[i].m_branchHandle >= 0) {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_branchHandle], 1.0f);
					}
					else {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_treePartHandle], 1.0f);
					}
				}

				for (int i = 0; i < m_predictedBranches.size(); i++) {
					predictedBranchConnectionStarts[i] = m_predictedBranches[i].m_bezierCurve.m_p0;
					predictedBranchConnectionEnds[i] = m_predictedBranches[i].m_bezierCurve.m_p3;
					predictedBranchConnectionColors[i] = glm::vec4(
						ecoSysLabLayer->RandomColors()[m_predictedBranches[i].m_handle], 1.0f);
				}
				predictedBranchConnectionInfoList->ApplyConnections(predictedBranchConnectionStarts, predictedBranchConnectionEnds, predictedBranchConnectionColors, predictedBranchWidth);

				for (const auto& skeleton : m_skeletons) {
					const auto& nodeList = skeleton.RefSortedNodeList();
					const auto startIndex = nodeMatrices.size();
					nodeMatrices.resize(startIndex + nodeList.size());
					for (int i = 0; i < nodeList.size(); i++) {
						const auto& node = skeleton.PeekNode(nodeList[i]);
						nodeMatrices[startIndex + i].m_instanceMatrix.m_value = glm::translate(node.m_info.m_globalPosition + skeleton.m_data.m_rootPosition) *
							glm::scale(glm::vec3(node.m_info.m_thickness));
						nodeMatrices[startIndex + i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[node.m_data.m_branchHandle],
							1.0f);
					}
				}
			}
				  break;
			case 2: {
				//Node
				for (int i = 0; i < m_allocatedPoints.size(); i++) {
					allocatedPointMatrices[i].m_instanceMatrix.m_value =
						glm::translate(m_allocatedPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
					if (m_allocatedPoints[i].m_nodeHandle >= 0) {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_nodeHandle], 1.0f);
					}
					else {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_treePartHandle], 1.0f);
					}
				}

				for (int i = 0; i < m_predictedBranches.size(); i++) {
					predictedBranchConnectionStarts[i] = m_predictedBranches[i].m_bezierCurve.m_p0;
					predictedBranchConnectionEnds[i] = m_predictedBranches[i].m_bezierCurve.m_p3;
					predictedBranchConnectionColors[i] = glm::vec4(1.0f);
				}
				predictedBranchConnectionInfoList->ApplyConnections(predictedBranchConnectionStarts, predictedBranchConnectionEnds, predictedBranchConnectionColors, predictedBranchWidth);
				for (const auto& skeleton : m_skeletons) {
					const auto& nodeList = skeleton.RefSortedNodeList();
					const auto startIndex = nodeMatrices.size();
					nodeMatrices.resize(startIndex + nodeList.size());
					for (int i = 0; i < nodeList.size(); i++) {
						const auto& node = skeleton.PeekNode(nodeList[i]);
						nodeMatrices[startIndex + i].m_instanceMatrix.m_value = glm::translate(node.m_info.m_globalPosition + skeleton.m_data.m_rootPosition) *
							glm::scale(glm::vec3(node.m_info.m_thickness));
						nodeMatrices[startIndex + i].m_instanceColor = glm::vec4(ecoSysLabLayer->RandomColors()[node.GetHandle()],
							1.0f);
					}
				}
			}
				  break;
			}

			scatterPointMatrices.resize(m_scatteredPoints.size());
			for (int i = 0; i < m_scatteredPoints.size(); i++) {
				scatterPointMatrices[i].m_instanceMatrix.m_value = glm::translate(m_scatteredPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
				scatterPointMatrices[i].m_instanceColor = scatterPointColor;
			}

			scatteredPointConnectionsStarts.resize(m_scatterPointsConnections.size());
			scatteredPointConnectionsEnds.resize(m_scatterPointsConnections.size());
			scatteredPointConnectionColors.resize(m_scatterPointsConnections.size());
			for (int i = 0; i < m_scatterPointsConnections.size(); i++) {
				scatteredPointConnectionsStarts[i] = m_scatterPointsConnections[i].first;
				scatteredPointConnectionsEnds[i] = m_scatterPointsConnections[i].second;
				scatteredPointConnectionColors[i] = scatterPointToBranchConnectionColor;
			}
			scatteredPointConnectionInfoList->ApplyConnections(
				scatteredPointConnectionsStarts,
				scatteredPointConnectionsEnds,
				scatteredPointConnectionColors, connectionWidth
			);


			candidateBranchConnectionStarts.resize(m_candidateBranchConnections.size());
			candidateBranchConnectionEnds.resize(m_candidateBranchConnections.size());
			candidateBranchConnectionColors.resize(m_candidateBranchConnections.size());
			for (int i = 0; i < m_candidateBranchConnections.size(); i++) {
				candidateBranchConnectionStarts[i] = m_candidateBranchConnections[i].first;
				candidateBranchConnectionEnds[i] = m_candidateBranchConnections[i].second;
				candidateBranchConnectionColors[i] = candidateBranchConnectionColor;
			}

			candidateBranchConnectionInfoList->ApplyConnections(
				candidateBranchConnectionStarts,
				candidateBranchConnectionEnds,
				candidateBranchConnectionColors, connectionWidth * 1.5f
			);

			reversedCandidateBranchConnectionStarts.resize(m_reversedCandidateBranchConnections.size());
			reversedCandidateBranchConnectionEnds.resize(m_reversedCandidateBranchConnections.size());
			reversedCandidateBranchConnectionColors.resize(m_reversedCandidateBranchConnections.size());
			for (int i = 0; i < m_reversedCandidateBranchConnections.size(); i++) {
				reversedCandidateBranchConnectionStarts[i] = m_reversedCandidateBranchConnections[i].first;
				reversedCandidateBranchConnectionEnds[i] = m_reversedCandidateBranchConnections[i].second;
				reversedCandidateBranchConnectionColors[i] = reversedCandidateBranchConnectionColor;
			}

			reversedCandidateBranchConnectionInfoList->ApplyConnections(
				reversedCandidateBranchConnectionStarts,
				reversedCandidateBranchConnectionEnds,
				reversedCandidateBranchConnectionColors, connectionWidth * 1.5f
			);

			filteredBranchConnectionStarts.resize(m_filteredBranchConnections.size());
			filteredBranchConnectionEnds.resize(m_filteredBranchConnections.size());
			filteredBranchConnectionColors.resize(m_filteredBranchConnections.size());
			for (int i = 0; i < m_filteredBranchConnections.size(); i++) {
				filteredBranchConnectionStarts[i] = m_filteredBranchConnections[i].first;
				filteredBranchConnectionEnds[i] = m_filteredBranchConnections[i].second;
				filteredBranchConnectionColors[i] = filteredBranchConnectionColor;
			}
			filteredBranchConnectionInfoList->ApplyConnections(
				filteredBranchConnectionStarts,
				filteredBranchConnectionEnds,
				filteredBranchConnectionColors, connectionWidth * 1.75f
			);

			branchConnectionStarts.resize(m_branchConnections.size());
			branchConnectionEnds.resize(m_branchConnections.size());
			branchConnectionColors.resize(m_branchConnections.size());
			for (int i = 0; i < m_branchConnections.size(); i++) {
				branchConnectionStarts[i] = m_branchConnections[i].first;
				branchConnectionEnds[i] = m_branchConnections[i].second;
				branchConnectionColors[i] = branchConnectionColor;
			}
			branchConnectionInfoList->ApplyConnections(
				branchConnectionStarts,
				branchConnectionEnds,
				branchConnectionColors, connectionWidth * 2.0f
			);

			scatterPointToBranchConnectionStarts.resize(
				m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size());
			scatterPointToBranchConnectionEnds.resize(
				m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size());
			scatterPointToBranchConnectionColors.resize(
				m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size());
			for (int i = 0; i < m_scatterPointToBranchStartConnections.size(); i++) {
				scatterPointToBranchConnectionStarts[i] = m_scatterPointToBranchStartConnections[i].first;
				scatterPointToBranchConnectionEnds[i] = m_scatterPointToBranchStartConnections[i].second;
				scatterPointToBranchConnectionColors[i] = scatterPointToBranchConnectionColor;
			}
			for (int i = m_scatterPointToBranchStartConnections.size();
				i < m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size(); i++) {
				scatterPointToBranchConnectionStarts[i] = m_scatterPointToBranchEndConnections[i -
					m_scatterPointToBranchStartConnections.size()].first;
				scatterPointToBranchConnectionEnds[i] = m_scatterPointToBranchEndConnections[i -
					m_scatterPointToBranchStartConnections.size()].second;
			}
			scatterPointToBranchConnectionInfoList->ApplyConnections(
				scatterPointToBranchConnectionStarts,
				scatterPointToBranchConnectionEnds,
				scatterPointToBranchConnectionColors, connectionWidth
			);
		}
		if (!scatterPointMatrices.empty()) {
			if (drawScatteredPoints) {
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
					scatterPointInfoList,
					glm::mat4(1.0f),
					pointSize, gizmoSettings);
			}
			if (drawAllocatedPoints) {
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
					allocatedPointInfoList,
					glm::mat4(1.0f),
					pointSize, gizmoSettings);
			}
			if (drawpredictedBranches)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CONE"), predictedBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawScatteredPointConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), scatteredPointConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);

			if (drawCandidateConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CONE"), candidateBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);

			if (drawReversedCandidateConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), reversedCandidateBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);

			if (drawFilteredConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), filteredBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawBranchConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), branchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);

			if (drawScatterPointToBranchConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), scatterPointToBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawNode) {
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), nodeInfoList,
					glm::mat4(1.0f), nodeSize, gizmoSettings);
			}
		}
	}
}

void TreePointCloud::EstablishConnectivityGraph() {
	m_scatterPointsConnections.clear();
	m_reversedCandidateBranchConnections.clear();
	m_candidateBranchConnections.clear();
	m_filteredBranchConnections.clear();
	m_branchConnections.clear();
	m_scatterPointToBranchStartConnections.clear();
	m_scatterPointToBranchEndConnections.clear();

	for (auto& point : m_scatteredPoints) {
		point.m_neighborScatterPoints.clear();
		point.m_p3.clear();
		point.m_p0.clear();
	}

	for (auto& predictedBranch : m_predictedBranches) {
		predictedBranch.m_pointsToP3.clear();
		predictedBranch.m_p3ToP0.clear();

		predictedBranch.m_pointsToP0.clear();
		predictedBranch.m_p3ToP3.clear();
		predictedBranch.m_p0ToP0.clear();
		predictedBranch.m_p0ToP3.clear();
	}

	//We establish connection between any 2 scatter points.
	for (auto& point : m_scatteredPoints) {
		if (m_scatterPointsConnections.size() > 1000000) {
			EVOENGINE_ERROR("Too much connections!");
			break;
		}
		if (point.m_position.y > m_connectivityGraphSettings.m_maxScatterPointConnectionHeight) continue;
		FindPoints(point.m_position, m_scatterPointsVoxelGrid, m_connectivityGraphSettings.m_pointPointConnectionDetectionRadius,
			[&](const PointData& voxel) {
				if (voxel.m_handle == point.m_handle) return;
				for (const auto& neighbor : point.m_neighborScatterPoints) {
					if (voxel.m_handle == neighbor) return;
				}
				auto& otherPoint = m_scatteredPoints[voxel.m_handle];
				point.m_neighborScatterPoints.emplace_back(voxel.m_handle);
				otherPoint.m_neighborScatterPoints.emplace_back(point.m_handle);
				m_scatterPointsConnections.emplace_back(point.m_position, otherPoint.m_position);
			});
	}

	for (auto& branch : m_predictedBranches) {
		const auto& p0 = branch.m_bezierCurve.m_p0;
		const auto& p3 = branch.m_bezierCurve.m_p3;
		float branchLength = glm::distance(p0, p3);
		//We find scatter points close to the branch p0.
		if (p0.y <= m_connectivityGraphSettings.m_maxScatterPointConnectionHeight) {
			FindPoints(p0, m_scatterPointsVoxelGrid, m_connectivityGraphSettings.m_pointBranchConnectionDetectionRadius,
				[&](const PointData& voxel) {
					{
						auto& otherPoint = m_scatteredPoints[voxel.m_handle];
						bool duplicate = false;
						for (const auto& i : otherPoint.m_p0) {
							if (i.second == branch.m_handle) {
								duplicate = true;
								break;
							}
						}
						if (!duplicate) otherPoint.m_p0.emplace_back(glm::distance(branch.m_bezierCurve.m_p0, voxel.m_position), branch.m_handle);
					}
					if (m_connectivityGraphSettings.m_reverseConnection)
					{
						bool duplicate = false;
						for (const auto& i : branch.m_pointsToP0) {
							if (i.second == voxel.m_handle) {
								duplicate = true;
								break;
							}
						}
						if (!duplicate) branch.m_pointsToP0.emplace_back(glm::distance(branch.m_bezierCurve.m_p0, voxel.m_position), voxel.m_handle);
					}
					m_scatterPointToBranchStartConnections.emplace_back(branch.m_bezierCurve.m_p0,
						voxel.m_position);
				});
		}
		if (p3.y <= m_connectivityGraphSettings.m_maxScatterPointConnectionHeight) {
			//We find branch p3 close to the scatter point.
			FindPoints(p3, m_scatterPointsVoxelGrid, m_connectivityGraphSettings.m_pointBranchConnectionDetectionRadius,
				[&](const PointData& voxel) {
					auto& otherPoint = m_scatteredPoints[voxel.m_handle];
					{
						bool duplicate = false;
						for (const auto& i : branch.m_pointsToP3) {
							if (i.second == voxel.m_handle) {
								duplicate = true;
								break;
							}
						}
						if (!duplicate) branch.m_pointsToP3.emplace_back(glm::distance(branch.m_bezierCurve.m_p3, voxel.m_position), voxel.m_handle);
					}
					if (m_connectivityGraphSettings.m_reverseConnection)
					{
						bool duplicate = false;
						for (const auto& i : otherPoint.m_p3) {
							if (i.second == branch.m_handle) {
								duplicate = true;
								break;
							}
						}
						if (!duplicate) otherPoint.m_p3.emplace_back(glm::distance(branch.m_bezierCurve.m_p3, voxel.m_position), branch.m_handle);
					}
					m_scatterPointToBranchEndConnections.emplace_back(branch.m_bezierCurve.m_p3,
						voxel.m_position);
				});
		}
		//Connect P3 from other branch to this branch's P0
		ForEachBranchEnd(p0, m_branchEndsVoxelGrid, branchLength * m_connectivityGraphSettings.m_branchBranchConnectionMaxLengthRange,
			[&](const BranchEndData& voxel) {
				if (voxel.m_branchHandle == branch.m_handle) return;
				auto& otherBranch = m_predictedBranches[voxel.m_branchHandle];
				if (!voxel.m_isP0) {
					const auto otherBranchP0 = otherBranch.m_bezierCurve.m_p0;
					const auto otherBranchP3 = otherBranch.m_bezierCurve.m_p3;
					if (DirectConnectionCheck(otherBranch.m_bezierCurve, branch.m_bezierCurve, false)) {
						const auto distance = glm::distance(otherBranchP3, p0);
						if (distance <= glm::distance(otherBranchP0, otherBranchP3) * m_connectivityGraphSettings.m_branchBranchConnectionMaxLengthRange) {
							const auto search = branch.m_p3ToP0.find(otherBranch.m_handle);
							if (search == branch.m_p3ToP0.end()) {
								branch.m_p3ToP0[otherBranch.m_handle] = distance;
								m_candidateBranchConnections.emplace_back(otherBranchP3, p0);
								if (m_connectivityGraphSettings.m_reverseConnection)
								{
									otherBranch.m_p0ToP3[branch.m_handle] = distance;
									//m_reversedCandidateBranchConnections.emplace_back(p0, otherBranchP3);
								}
							}
						}
					}
				}
				else if (m_connectivityGraphSettings.m_reverseConnection)
				{
					const auto otherBranchP0 = otherBranch.m_bezierCurve.m_p0;
					const auto otherBranchP3 = otherBranch.m_bezierCurve.m_p3;
					if (DirectConnectionCheck(otherBranch.m_bezierCurve, branch.m_bezierCurve, true)) {
						const auto distance = glm::distance(otherBranchP0, p0);
						if (distance <= glm::distance(otherBranchP0, otherBranchP3) * m_connectivityGraphSettings.m_branchBranchConnectionMaxLengthRange) {
							const auto search = branch.m_p0ToP0.find(otherBranch.m_handle);
							if (search == branch.m_p0ToP0.end()) {
								branch.m_p0ToP0[otherBranch.m_handle] = distance;
								otherBranch.m_p0ToP0[branch.m_handle] = distance;
								m_reversedCandidateBranchConnections.emplace_back(p0, otherBranchP0);
							}
						}
					}
				}
			});


		//Connect P0 from other branch to this branch's P3		
		ForEachBranchEnd(p3, m_branchEndsVoxelGrid, branchLength * m_connectivityGraphSettings.m_branchBranchConnectionMaxLengthRange,
			[&](const BranchEndData& voxel) {
				if (voxel.m_branchHandle == branch.m_handle) return;
				auto& otherBranch = m_predictedBranches[voxel.m_branchHandle];
				if (voxel.m_isP0) {
					const auto otherBranchP0 = otherBranch.m_bezierCurve.m_p0;
					const auto otherBranchP3 = otherBranch.m_bezierCurve.m_p3;
					if (DirectConnectionCheck(branch.m_bezierCurve, otherBranch.m_bezierCurve, false)) {
						const auto distance = glm::distance(otherBranchP0, p3);
						if (distance <= glm::distance(otherBranchP0, otherBranchP3) * m_connectivityGraphSettings.m_branchBranchConnectionMaxLengthRange) {
							{
								const auto search = otherBranch.m_p3ToP0.find(branch.m_handle);
								if (search == otherBranch.m_p3ToP0.end()) {
									otherBranch.m_p3ToP0[branch.m_handle] = distance;
									m_candidateBranchConnections.emplace_back(otherBranchP0, p3);
									if (m_connectivityGraphSettings.m_reverseConnection)
									{
										branch.m_p0ToP3[otherBranch.m_handle] = distance;
										//m_reversedCandidateBranchConnections.emplace_back(p3, otherBranchP0);
									}
								}
							}
						}
					}
				}
				else if (m_connectivityGraphSettings.m_reverseConnection)
				{
					const auto otherBranchP0 = otherBranch.m_bezierCurve.m_p0;
					const auto otherBranchP3 = otherBranch.m_bezierCurve.m_p3;
					if (DirectConnectionCheck(branch.m_bezierCurve, otherBranch.m_bezierCurve, true)) {
						const auto distance = glm::distance(otherBranchP3, p3);
						if (distance <= glm::distance(otherBranchP0, otherBranchP3) * m_connectivityGraphSettings.m_branchBranchConnectionMaxLengthRange) {
							const auto search = otherBranch.m_p3ToP3.find(branch.m_handle);
							if (search == otherBranch.m_p3ToP3.end()) {
								otherBranch.m_p3ToP3[branch.m_handle] = distance;
								branch.m_p3ToP3[otherBranch.m_handle] = distance;
								m_reversedCandidateBranchConnections.emplace_back(p3, otherBranchP3);
							}
						}
					}
				}
			});

	}
	//We search branch connections via scatter points start from p0.
	for (auto& predictedBranch : m_predictedBranches) {
		std::unordered_set<PointHandle> visitedPoints;
		std::vector<PointHandle> processingPoints;
		float distanceL = FLT_MAX;
		for (const auto& i : predictedBranch.m_pointsToP3)
		{
			processingPoints.emplace_back(i.second);
			auto distance = glm::distance(predictedBranch.m_bezierCurve.m_p3, m_scatteredPoints[i.second].m_position);
			if (distance < distanceL) distanceL = distance;
		}
		for (const auto& i : processingPoints) {
			visitedPoints.emplace(i);
		}
		while (!processingPoints.empty()) {
			auto currentPointHandle = processingPoints.back();
			visitedPoints.emplace(currentPointHandle);
			processingPoints.pop_back();
			auto& currentPoint = m_scatteredPoints[currentPointHandle];
			for (const auto& branchInfo : currentPoint.m_p0) {
				if (predictedBranch.m_handle == branchInfo.second) continue;
				bool skip = false;
				for (const auto& i : predictedBranch.m_p3ToP0) {
					if (branchInfo.second == i.first) {
						skip = true;
						break;
					}
				}
				if (skip) continue;
				auto& otherBranch = m_predictedBranches[branchInfo.second];
				auto pA = predictedBranch.m_bezierCurve.m_p3;
				auto pB = predictedBranch.m_bezierCurve.m_p0;
				auto otherPA = otherBranch.m_bezierCurve.m_p3;
				auto otherPB = otherBranch.m_bezierCurve.m_p0;
				const auto dotP = glm::dot(glm::normalize(otherPB - otherPA),
					glm::normalize(pB - pA));
				if (dotP < glm::cos(glm::radians(m_connectivityGraphSettings.m_indirectConnectionAngleLimit))) continue;
				float distance = distanceL + branchInfo.first;
				const auto search = predictedBranch.m_p3ToP0.find(branchInfo.second);
				if (search == predictedBranch.m_p3ToP0.end() || search->second > distance)
				{
					predictedBranch.m_p3ToP0[branchInfo.second] = distance;
					m_candidateBranchConnections.emplace_back(pA, otherPB);
				}
			}
			/*
				if (m_connectivityGraphSettings.m_reverseConnection)
				{
					for (const auto& branchInfo : currentPoint.m_p3) {
						if (predictedBranch.m_handle == branchInfo.second) continue;
						bool skip = false;
						for (const auto& i : predictedBranch.m_p3ToP3) {
							if (branchInfo.second == i.first) {
								skip = true;
								break;
							}
						}
						if (skip) continue;
						auto& otherBranch = m_predictedBranches[branchInfo.second];
						auto pA = predictedBranch.m_bezierCurve.m_p3;
						auto pB = predictedBranch.m_bezierCurve.m_p0;
						auto otherPA = otherBranch.m_bezierCurve.m_p0;
						auto otherPB = otherBranch.m_bezierCurve.m_p3;
						const auto dotP = glm::dot(glm::normalize(otherPB - otherPA),
							glm::normalize(pB - pA));
						if (dotP > glm::cos(glm::radians(m_connectivityGraphSettings.m_indirectConnectionAngleLimit))) continue;
						const auto dotP2 = glm::dot(glm::normalize(pB - pA), glm::normalize(otherPA - pA));
						if (dotP2 > 3) continue;

						float distance = distanceL + branchInfo.first;
						const auto search = predictedBranch.m_p3ToP3.find(branchInfo.second);
						if (search == predictedBranch.m_p3ToP3.end() || search->second > distance)
						{
							predictedBranch.m_p3ToP3[branchInfo.second] = distance;
							m_reversedCandidateBranchConnections.emplace_back(pA, otherPB);
						}
					}
				}
				*/
			for (const auto& neighborHandle : currentPoint.m_neighborScatterPoints) {
				if (visitedPoints.find(neighborHandle) == visitedPoints.end()) processingPoints.emplace_back(neighborHandle);
			}
		}
	}
	/*
	if (m_connectivityGraphSettings.m_reverseConnection)
	{
		for (auto& predictedBranch : m_predictedBranches) {
			std::unordered_set<PointHandle> visitedPoints;
			std::vector<PointHandle> processingPoints;
			float distanceL = FLT_MAX;
			for (const auto& i : predictedBranch.m_pointsToP0)
			{
				processingPoints.emplace_back(i.second);
				auto distance = glm::distance(predictedBranch.m_bezierCurve.m_p0, m_scatteredPoints[i.second].m_position);
				if (distance < distanceL) distanceL = distance;
			}
			for (const auto& i : processingPoints) {
				visitedPoints.emplace(i);
			}
			while (!processingPoints.empty()) {
				auto currentPointHandle = processingPoints.back();
				visitedPoints.emplace(currentPointHandle);
				processingPoints.pop_back();
				auto& currentPoint = m_scatteredPoints[currentPointHandle];
				for (const auto& neighborHandle : currentPoint.m_neighborScatterPoints) {
					if (visitedPoints.find(neighborHandle) != visitedPoints.end()) continue;
					auto& neighbor = m_scatteredPoints[neighborHandle];
					//We stop search if the point is junction point.
					for (const auto& branchInfo : neighbor.m_p3) {
						if (predictedBranch.m_handle == branchInfo.second) continue;
						bool skip = false;
						for (const auto& i : predictedBranch.m_p0ToP3) {
							if (branchInfo.second == i.first) {
								skip = true;
								break;
							}
						}
						if (skip) continue;
						auto& parentCandidateBranch = m_predictedBranches[branchInfo.second];
						auto pA = predictedBranch.m_bezierCurve.m_p0;
						auto pB = predictedBranch.m_bezierCurve.m_p3;
						auto otherPA = parentCandidateBranch.m_bezierCurve.m_p0;
						auto otherPB = parentCandidateBranch.m_bezierCurve.m_p3;
						const auto dotP = glm::dot(glm::normalize(otherPB - otherPA),
							glm::normalize(pB - pA));
						if (dotP > glm::cos(glm::radians(m_connectivityGraphSettings.m_indirectConnectionAngleLimit))) continue;
						const auto dotP2 = glm::dot(glm::normalize(pB - pA), glm::normalize(otherPA - pA));
						if (dotP2 > 0) continue;

						float distance = distanceL + branchInfo.first;
						const auto search = predictedBranch.m_p0ToP3.find(branchInfo.second);
						if (search == predictedBranch.m_p0ToP3.end() || search->second > distance)
						{
							predictedBranch.m_p0ToP3[branchInfo.second] = distance;
							m_candidateBranchConnections.emplace_back(pA, otherPB);
						}
					}
					if (m_connectivityGraphSettings.m_reverseConnection)
					{
						for (const auto& branchInfo : neighbor.m_p0) {
							if (predictedBranch.m_handle == branchInfo.second) continue;
							bool skip = false;
							for (const auto& i : predictedBranch.m_p0ToP0) {
								if (branchInfo.second == i.first) {
									skip = true;
									break;
								}
							}
							if (skip) continue;
							auto& parentCandidateBranch = m_predictedBranches[branchInfo.second];
							auto pA = predictedBranch.m_bezierCurve.m_p0;
							auto pB = predictedBranch.m_bezierCurve.m_p3;
							auto otherPA = parentCandidateBranch.m_bezierCurve.m_p3;
							auto otherPB = parentCandidateBranch.m_bezierCurve.m_p0;
							const auto dotP = glm::dot(glm::normalize(otherPB - otherPA),
								glm::normalize(pB - pA));
							if (dotP > glm::cos(glm::radians(m_connectivityGraphSettings.m_indirectConnectionAngleLimit))) continue;
							const auto dotP2 = glm::dot(glm::normalize(pB - pA), glm::normalize(otherPA - pA));
							if (dotP2 > 0) continue;
							float distance = distanceL + branchInfo.first;
							const auto search = predictedBranch.m_p0ToP0.find(branchInfo.second);
							if (search == predictedBranch.m_p0ToP0.end() || search->second > distance)
							{
								predictedBranch.m_p0ToP0[branchInfo.second] = distance;
								m_reversedCandidateBranchConnections.emplace_back(pA, otherPB);
							}
						}
					}
					processingPoints.emplace_back(neighborHandle);
				}
			}
		}
	}

	*/
}

void TreePointCloud::BuildSkeletons() {
	m_skeletons.clear();
	std::unordered_set<BranchHandle> allocatedBranchHandles;
	m_filteredBranchConnections.clear();
	m_branchConnections.clear();
	m_operatingBranches.resize(m_predictedBranches.size());
	for (int i = 0; i < m_predictedBranches.size(); i++) {
		CloneOperatingBranch(m_reconstructionSettings, m_operatingBranches[i], m_predictedBranches[i]);
	}
	std::vector<std::pair<glm::vec3, BranchHandle>> rootBranchHandles{};
	std::unordered_set<BranchHandle> rootBranchHandleSet{};
	//Branch is shortened after this.
	for (auto& branch : m_operatingBranches) {
		branch.m_chainNodeHandles.clear();
		branch.m_rootDistance = 0.0f;
		branch.m_distanceToParentBranch = 0.0f;
		branch.m_bestDistance = FLT_MAX;
		const auto branchStart = branch.m_bezierCurve.m_p0;
		auto shortenedP0 = branch.m_bezierCurve.GetPoint(m_reconstructionSettings.m_branchShortening);
		auto shortenedP3 = branch.m_bezierCurve.GetPoint(1.0f - m_reconstructionSettings.m_branchShortening);
		auto shortenedLength = glm::distance(shortenedP0, shortenedP3);
		if (branchStart.y <= m_reconstructionSettings.m_minHeight) {
			branch.m_bezierCurve.m_p0.y = 0.0f;
			rootBranchHandles.emplace_back(branch.m_bezierCurve.m_p0, branch.m_handle);
			rootBranchHandleSet.emplace(branch.m_handle);
		}
		else {
			branch.m_bezierCurve.m_p0 = shortenedP0;
		}
		branch.m_bezierCurve.m_p3 = shortenedP3;
		branch.m_bezierCurve.m_p1 = branch.m_bezierCurve.m_p0 +
			branch.m_bezierCurve.GetAxis(m_reconstructionSettings.m_branchShortening) *
			shortenedLength * 0.25f;
		branch.m_bezierCurve.m_p2 = branch.m_bezierCurve.m_p3 -
			branch.m_bezierCurve.GetAxis(1.0f - m_reconstructionSettings.m_branchShortening) *
			shortenedLength * 0.25f;
	}

	bool branchRemoved = true;
	while (branchRemoved)
	{
		branchRemoved = false;
		std::vector<BranchHandle> removeList{};
		for (int i = 0; i < m_operatingBranches.size(); i++)
		{
			if (rootBranchHandleSet.find(i) != rootBranchHandleSet.end()) continue;
			auto& operatingBranch = m_operatingBranches[i];
			if (!operatingBranch.m_orphan && operatingBranch.m_parentCandidates.empty())
			{
				operatingBranch.m_orphan = true;
				removeList.emplace_back(i);
			}
		}
		if (!removeList.empty())
		{
			branchRemoved = true;
			for (auto& operatingBranch : m_operatingBranches)
			{
				for (int i = 0; i < operatingBranch.m_parentCandidates.size(); i++)
				{
					for (const auto& removeHandle : removeList) {
						if (operatingBranch.m_parentCandidates[i].first == removeHandle)
						{
							operatingBranch.m_parentCandidates.erase(operatingBranch.m_parentCandidates.begin() + i);
							i--;
							break;
						}
					}
				}
			}
		}
	}
	for (auto& operatingBranch : m_operatingBranches)
	{
		for (const auto& parentCandidate : operatingBranch.m_parentCandidates) {
			const auto& parentBranch = m_predictedBranches[parentCandidate.first];
			m_filteredBranchConnections.emplace_back(operatingBranch.m_bezierCurve.m_p0, parentBranch.m_bezierCurve.m_p3);
		}
	}

	for (const auto& rootBranchHandle : rootBranchHandles) {
		auto& skeleton = m_skeletons.emplace_back();
		skeleton.m_data.m_rootPosition = rootBranchHandle.first;
		auto& processingBranch = m_operatingBranches[rootBranchHandle.second];
		processingBranch.m_skeletonIndex = m_skeletons.size() - 1;
		processingBranch.m_used = true;
		int prevNodeHandle = 0;
		processingBranch.m_chainNodeHandles.emplace_back(prevNodeHandle);
		float chainLength = processingBranch.m_bezierCurve.GetLength();
		int chainAmount = glm::max(2, static_cast<int>(chainLength /
			m_reconstructionSettings.m_internodeLength));
		for (int i = 1; i < chainAmount; i++) {
			prevNodeHandle = skeleton.Extend(prevNodeHandle, false);
			processingBranch.m_chainNodeHandles.emplace_back(prevNodeHandle);
		}
		ApplyCurve(processingBranch);
	}

	std::multimap<float, BranchHandle> heightSortedBranches{};
	for (const auto& i : m_operatingBranches)
	{
		heightSortedBranches.insert({ i.m_bezierCurve.m_p0.y, i.m_handle });
	}

	bool newBranchAllocated = true;
	while (newBranchAllocated)
	{
		newBranchAllocated = false;
		for (const auto& branchPair : heightSortedBranches)
		{
			auto& childBranch = m_operatingBranches[branchPair.second];
			if (childBranch.m_orphan || childBranch.m_used || childBranch.m_parentCandidates.empty()) continue;
			BranchHandle bestParentHandle = -1;
			float bestDistance = FLT_MAX;
			float bestParentDistance = FLT_MAX;
			float bestRootDistance = FLT_MAX;
			int maxIndex = -1;
			for(int i = 0; i < childBranch.m_parentCandidates.size(); i++)
			{
				const auto& parentCandidate = childBranch.m_parentCandidates[i];
				const auto& parentBranch = m_operatingBranches[parentCandidate.first];
				if(!parentBranch.m_used || parentBranch.m_childHandles.size() >= m_reconstructionSettings.m_maxChildSize) continue;
				float distance = parentCandidate.second;
				float rootDistance = parentBranch.m_rootDistance + glm::distance(parentBranch.m_bezierCurve.m_p0, parentBranch.m_bezierCurve.m_p3) + distance;
				if(m_reconstructionSettings.m_useRootDistance)
				{
					distance = rootDistance;
				}
				if(distance < bestDistance)
				{
					bestParentHandle = parentCandidate.first;
					bestRootDistance = rootDistance;
					bestParentDistance = parentCandidate.second;
					maxIndex = i;
					bestDistance = distance;
				}
			}
			if(maxIndex != -1)
			{
				childBranch.m_parentCandidates[maxIndex] = childBranch.m_parentCandidates.back();
				childBranch.m_parentCandidates.pop_back();
				newBranchAllocated = true;
				Link(branchPair.second, bestParentHandle);
				childBranch.m_rootDistance = bestRootDistance;
				childBranch.m_bestDistance = bestDistance;
				childBranch.m_distanceToParentBranch = bestParentDistance;
			}
		}
	}
	bool optimized = true;
	int iteration = 0;
	while(optimized && iteration < m_reconstructionSettings.m_optimizationTimeout)
	{
		optimized = false;
		for (auto& childBranch : m_operatingBranches)
		{
			if (childBranch.m_orphan || childBranch.m_parentHandle == -1 || childBranch.m_parentCandidates.empty()) continue;
			BranchHandle bestParentHandle = -1;
			float bestDistance = childBranch.m_bestDistance;
			float bestParentDistance = FLT_MAX;
			float bestRootDistance = childBranch.m_rootDistance;
			int maxIndex = -1;
			for (int i = 0; i < childBranch.m_parentCandidates.size(); i++)
			{
				const auto& parentCandidate = childBranch.m_parentCandidates[i];
				const auto& parentBranch = m_operatingBranches[parentCandidate.first];
				if (!parentBranch.m_used || parentBranch.m_childHandles.size() >= m_reconstructionSettings.m_maxChildSize) continue;
				float distance = parentCandidate.second;
				float rootDistance = parentBranch.m_rootDistance + glm::distance(parentBranch.m_bezierCurve.m_p0, parentBranch.m_bezierCurve.m_p3) + distance;
				if (m_reconstructionSettings.m_useRootDistance)
				{
					distance = rootDistance;
				}
				if (distance < bestDistance)
				{
					bestParentHandle = parentCandidate.first;
					bestRootDistance = rootDistance;
					bestParentDistance = parentCandidate.second;
					maxIndex = i;
					bestDistance = distance;
				}
			}
			std::vector<BranchHandle> subTreeBranchList{};
			GetSortedBranchList(childBranch.m_handle, subTreeBranchList);
			if(maxIndex != -1)
			{
				for(const auto& branchHandle : subTreeBranchList)
				{
					if (branchHandle == bestParentHandle) {
						maxIndex = -1;
						break;
					}
				}
			}
			if (maxIndex != -1)
			{
				childBranch.m_parentCandidates[maxIndex] = childBranch.m_parentCandidates.back();
				childBranch.m_parentCandidates.pop_back();
				newBranchAllocated = true;
				Unlink(childBranch.m_handle, childBranch.m_parentHandle);
				Link(childBranch.m_handle, bestParentHandle);
				childBranch.m_rootDistance = bestRootDistance;
				childBranch.m_bestDistance = bestDistance;
				childBranch.m_distanceToParentBranch = bestParentDistance;
				optimized = true;
			}
		}
		if(optimized)
		{
			CalculateBranchRootDistance(rootBranchHandles);
		}
		iteration++;
	}
	CalculateBranchRootDistance(rootBranchHandles);

	for(const auto& operatingBranch : m_operatingBranches)
	{
		if (operatingBranch.m_parentHandle != -1) {
			m_branchConnections.emplace_back(m_predictedBranches[operatingBranch.m_handle].m_bezierCurve.m_p0,
				m_predictedBranches[operatingBranch.m_parentHandle].m_bezierCurve.m_p3);
		}
	}

	//Assign apical branch.
	for (const auto& rootBranchHandle : rootBranchHandles)
	{
		std::vector<BranchHandle> sortedBranchList{};
		GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
		for (auto it = sortedBranchList.rbegin(); it != sortedBranchList.rend(); ++it)
		{
			auto& operatingBranch = m_operatingBranches[*it];
			int maxDescendentSize = -1;
			operatingBranch.m_largestChildHandle = -1;
			for(const auto& childHandle : operatingBranch.m_childHandles)
			{
				auto& childBranch = m_operatingBranches[childHandle];
				operatingBranch.m_descendentSize += childBranch.m_descendentSize + 1;
				if(childBranch.m_descendentSize >= maxDescendentSize)
				{
					maxDescendentSize = childBranch.m_descendentSize;
					operatingBranch.m_largestChildHandle = childHandle;
				}
			}
			for (const auto& childHandle : operatingBranch.m_childHandles)
			{
				auto& childBranch = m_operatingBranches[childHandle];
				if(childHandle == operatingBranch.m_largestChildHandle)
				{
					childBranch.m_apical = true;
				}else
				{
					childBranch.m_apical = false;
				}
			}
		}
	}

	//Smoothing
	for (int i = 0; i < m_reconstructionSettings.m_smoothIteration; i++) {
		for (const auto& rootBranchHandle : rootBranchHandles)
		{
			std::vector<BranchHandle> sortedBranchList{};
			GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
			for (const auto& branchHandle : sortedBranchList)
			{
				auto& branch = m_operatingBranches[branchHandle];
				if (branch.m_parentHandle != -1 && branch.m_largestChildHandle != -1)
				{
					const auto& parentBranch = m_operatingBranches[branch.m_parentHandle];
					if(parentBranch.m_largestChildHandle != branchHandle) continue;
					const auto& childBranch = m_operatingBranches[branch.m_largestChildHandle];
					const auto parentCenter = (parentBranch.m_bezierCurve.m_p0 + parentBranch.m_bezierCurve.m_p3) * 0.5f;
					const auto childCenter = (childBranch.m_bezierCurve.m_p0 + childBranch.m_bezierCurve.m_p3) * 0.5f;
					const auto center = (branch.m_bezierCurve.m_p0 + branch.m_bezierCurve.m_p3) * 0.5f;
					const auto desiredCenter = (parentCenter + childCenter) * 0.5f;
					auto diff = (desiredCenter - center);
					branch.m_bezierCurve.m_p0 = branch.m_bezierCurve.m_p0 + diff * m_reconstructionSettings.m_positionSmoothing;
					branch.m_bezierCurve.m_p1 = branch.m_bezierCurve.m_p1 + diff * m_reconstructionSettings.m_positionSmoothing;
					branch.m_bezierCurve.m_p2 = branch.m_bezierCurve.m_p2 + diff * m_reconstructionSettings.m_positionSmoothing;
					branch.m_bezierCurve.m_p3 = branch.m_bezierCurve.m_p3 + diff * m_reconstructionSettings.m_positionSmoothing;
				}
			}
		}
		for (const auto& rootBranchHandle : rootBranchHandles)
		{
			std::vector<BranchHandle> sortedBranchList{};
			GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
			for (const auto& branchHandle : sortedBranchList)
			{
				auto& branch = m_operatingBranches[branchHandle];
				if (branch.m_parentHandle != -1 && branch.m_largestChildHandle != -1)
				{
					const auto& parentBranch = m_operatingBranches[branch.m_parentHandle];
					const auto& childBranch = m_operatingBranches[branch.m_largestChildHandle];
					const auto parentCenter = (parentBranch.m_bezierCurve.m_p0 + parentBranch.m_bezierCurve.m_p3) * 0.5f;
					const auto childCenter = (childBranch.m_bezierCurve.m_p0 + childBranch.m_bezierCurve.m_p3) * 0.5f;
					const auto center = (branch.m_bezierCurve.m_p0 + branch.m_bezierCurve.m_p3) * 0.5f;
					const auto length = glm::distance(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3);
					//const auto currentDirection = glm::normalize(branch.m_bezierCurve.m_p3 - branch.m_bezierCurve.m_p0);

					const auto desiredDirection = glm::normalize(parentCenter - childCenter);

					const auto desiredP0 = center + desiredDirection * length * 0.5f;
					const auto desiredP3 = center - desiredDirection * length * 0.5f;
					const auto desiredP1 = glm::mix(desiredP0, desiredP3, 0.25f);
					const auto desiredP2 = glm::mix(desiredP0, desiredP3, 0.75f);

					branch.m_bezierCurve.m_p0 = glm::mix(branch.m_bezierCurve.m_p0, desiredP0, m_reconstructionSettings.m_directionSmoothing);
					branch.m_bezierCurve.m_p1 = glm::mix(branch.m_bezierCurve.m_p1, desiredP1, m_reconstructionSettings.m_directionSmoothing);
					branch.m_bezierCurve.m_p2 = glm::mix(branch.m_bezierCurve.m_p2, desiredP2, m_reconstructionSettings.m_directionSmoothing);
					branch.m_bezierCurve.m_p3 = glm::mix(branch.m_bezierCurve.m_p3, desiredP3, m_reconstructionSettings.m_directionSmoothing);
				}
			}
		}
	}
	for (const auto& rootBranchHandle : rootBranchHandles)
	{
		ConnectBranches(rootBranchHandle.second);
	}

	CalculateSkeletonGraphs();

	for (auto& allocatedPoint : m_allocatedPoints) {
		const auto& treePart = m_treeParts[allocatedPoint.m_treePartHandle];
		float minDistance = 999.f;
		NodeHandle closestNodeHandle = -1;
		BranchHandle closestBranchHandle = -1;
		int closestSkeletonIndex = -1;
		for (const auto& branchHandle : treePart.m_branchHandles) {
			auto& branch = m_operatingBranches[branchHandle];
			for (const auto& nodeHandle : branch.m_chainNodeHandles) {
				auto& node = m_skeletons[branch.m_skeletonIndex].RefNode(nodeHandle);
				auto distance = glm::distance(node.m_info.m_globalPosition, allocatedPoint.m_position);
				if (distance < minDistance) {
					minDistance = distance;
					closestNodeHandle = nodeHandle;
					closestSkeletonIndex = branch.m_skeletonIndex;
					closestBranchHandle = branchHandle;
				}
			}
		}
		allocatedPoint.m_nodeHandle = closestNodeHandle;
		allocatedPoint.m_branchHandle = closestBranchHandle;
		allocatedPoint.m_skeletonIndex = closestSkeletonIndex;
		if (allocatedPoint.m_skeletonIndex != -1)
			m_skeletons[allocatedPoint.m_skeletonIndex].RefNode(
				closestNodeHandle).m_data.m_allocatedPoints.emplace_back(allocatedPoint.m_handle);
	}
	if (m_skeletons.size() > 1) {
		for (int i = 0; i < m_skeletons.size(); i++)
		{
			auto& skeleton = m_skeletons[i];
			bool remove = false;
			if (skeleton.RefSortedNodeList().size() < m_reconstructionSettings.m_minimumNodeCount)
			{
				remove = true;
			}
			else {
				for (int j = 0; j < m_skeletons.size(); j++)
				{
					if (j == i) continue;
					auto& otherSkeleton = m_skeletons[j];
					if (glm::distance(skeleton.m_data.m_rootPosition, otherSkeleton.m_data.m_rootPosition) < m_reconstructionSettings.m_minimumTreeDistance)
					{
						if (skeleton.RefSortedNodeList().size() < otherSkeleton.RefSortedNodeList().size()) remove = true;
					}
				}
			}

			if (remove)
			{
				m_skeletons.erase(m_skeletons.begin() + i);
				i--;
			}
		}
	}

	SpaceColonization();
}

void TreePointCloud::SpaceColonization()
{
	if (m_reconstructionSettings.m_spaceColonizationFactor == 0.0f) return;

	Jobs::ParallelFor(m_skeletons.size(), [&](unsigned i)
		{
			auto& skeleton = m_skeletons[i];
			const auto& sortedInternodeList = skeleton.RefSortedNodeList();
			float maxEndDistance = 0.0f;
			for (const auto& internodeHandle : sortedInternodeList)
			{
				auto& internode = skeleton.RefNode(internodeHandle);
				const auto distance = internode.m_info.m_endDistance + internode.m_info.m_length;
				if (internode.GetParentHandle() == -1)
				{
					skeleton.m_data.m_maxEndDistance = distance;
					maxEndDistance = distance * m_reconstructionSettings.m_spaceColonizationFactor;
				}
				internode.m_data.m_regrowth = distance <= maxEndDistance;
			}
		}
	);

	//Register voxel grid.
	const float removalDistance = m_reconstructionSettings.m_spaceColonizationRemovalDistanceFactor * m_reconstructionSettings.m_internodeLength;
	const float detectionDistance = m_reconstructionSettings.m_spaceColonizationDetectionDistanceFactor * m_reconstructionSettings.m_internodeLength;

	m_spaceColonizationVoxelGrid.Initialize(removalDistance, m_min, m_max);
	for (auto& point : m_scatteredPoints) {
		PointData voxel;
		voxel.m_handle = -1;
		voxel.m_position = point.m_position;
		m_spaceColonizationVoxelGrid.Ref(voxel.m_position).emplace_back(voxel);
	}
	for (auto& point : m_allocatedPoints) {
		PointData voxel;
		voxel.m_handle = -1;
		voxel.m_position = point.m_position;
		m_spaceColonizationVoxelGrid.Ref(voxel.m_position).emplace_back(voxel);
	}

	VoxelGrid<std::vector<PointData>> internodeEndGrid{};
	internodeEndGrid.Initialize(detectionDistance, m_min, m_max);
	for (int skeletonIndex = 0; skeletonIndex < m_skeletons.size(); skeletonIndex++)
	{
		auto& skeleton = m_skeletons[skeletonIndex];
		const auto& sortedInternodeList = skeleton.RefSortedNodeList();
		for (const auto& internodeHandle : sortedInternodeList)
		{
			auto& internode = skeleton.PeekNode(internodeHandle);
			if (!internode.m_data.m_regrowth) continue;
			PointData voxel;
			voxel.m_handle = internodeHandle;
			voxel.m_index = skeletonIndex;
			voxel.m_position = internode.m_data.m_globalEndPosition;
			voxel.m_direction = internode.m_info.m_globalDirection;
			internodeEndGrid.Ref(voxel.m_position).emplace_back(voxel);
		}
	}

	const auto dotMin = glm::cos(glm::radians(m_reconstructionSettings.m_spaceColonizationTheta));
	bool newBranchGrown = true;
	int timeout = 0;
	while(newBranchGrown && timeout < m_reconstructionSettings.m_spaceColonizationTimeout)
	{
		newBranchGrown = false;
		timeout++;
		//1. Remove markers with occupancy zone.
		for (auto& skeleton : m_skeletons)
		{
			const auto& sortedInternodeList = skeleton.RefSortedNodeList();
			for (const auto& internodeHandle : sortedInternodeList)
			{
				auto& internode = skeleton.RefNode(internodeHandle);
				internode.m_data.m_markerSize = 0;
				internode.m_data.m_regrowDirection = glm::vec3(0.0f);
				const auto internodeEndPosition = internode.m_data.m_globalEndPosition;
				m_spaceColonizationVoxelGrid.ForEach(internodeEndPosition, removalDistance,
					[&](std::vector<PointData>& voxels)
					{
						for(int i = 0; i < voxels.size(); i++)
						{
							auto& marker = voxels[i];
							const auto diff = marker.m_position - internodeEndPosition;
							const auto distance = glm::length(diff);
							if(distance < removalDistance)
							{
								voxels[i] = voxels.back();
								voxels.pop_back();
								i--;
							}
						}
					}
				);
			}
		}

		//2. Allocate markers to node with perception volume.
		for(auto& voxel : m_spaceColonizationVoxelGrid.RefData())
		{
			for (auto& point : voxel)
			{
				point.m_minDistance = FLT_MAX;
				point.m_handle = -1;
				point.m_index = -1;
				point.m_direction = glm::vec3(0.0f);
				internodeEndGrid.ForEach(point.m_position, detectionDistance,
					[&](const std::vector<PointData>& voxels)
					{
						for (const auto& internodeEnd : voxels)
						{
							const auto diff = point.m_position - internodeEnd.m_position;
							const auto distance = glm::length(diff);
							const auto direction = glm::normalize(diff);
							if (distance < detectionDistance 
								&& glm::dot(direction, internodeEnd.m_direction) > dotMin
								&& distance < point.m_minDistance)
							{
								point.m_minDistance = distance;
								point.m_handle = internodeEnd.m_handle;
								point.m_index = internodeEnd.m_index;
								point.m_direction = diff;
							}
						}
					}
				);
			}
		}

		//3. Calculate new direction for each internode.
		for (auto& voxel : m_spaceColonizationVoxelGrid.RefData())
		{
			for (auto& point : voxel)
			{
				if(point.m_handle != -1)
				{
					auto& internode = m_skeletons[point.m_index].RefNode(point.m_handle);
					internode.m_data.m_markerSize++;
					internode.m_data.m_regrowDirection += point.m_direction;
				}
			}
		}

		//4. Grow and add new internodes to the internodeEndGrid.
		for (int skeletonIndex = 0; skeletonIndex < m_skeletons.size(); skeletonIndex++)
		{
			auto& skeleton = m_skeletons[skeletonIndex];
			const auto& sortedInternodeList = skeleton.RefSortedNodeList();
			for (const auto& internodeHandle : sortedInternodeList)
			{
				auto& internode = skeleton.PeekNode(internodeHandle);
				if (!internode.m_data.m_regrowth || internode.m_data.m_markerSize == 0) continue;
				if(internode.m_info.m_rootDistance > skeleton.m_data.m_maxEndDistance) continue;
				newBranchGrown = true;
				const auto newInternodeHandle = skeleton.Extend(internodeHandle, !internode.RefChildHandles().empty());
				auto& oldInternode = skeleton.RefNode(internodeHandle);
				auto& newInternode = skeleton.RefNode(newInternodeHandle);
				newInternode.m_info.m_globalPosition = oldInternode.m_info.GetGlobalEndPosition();
				newInternode.m_info.m_length = m_reconstructionSettings.m_internodeLength;
				newInternode.m_info.m_globalDirection = glm::normalize(oldInternode.m_data.m_regrowDirection);
				newInternode.m_data.m_globalEndPosition = oldInternode.m_data.m_globalEndPosition + newInternode.m_info.m_length * newInternode.m_info.m_globalDirection;
				newInternode.m_data.m_regrowth = true;
				PointData voxel;
				voxel.m_handle = newInternodeHandle;
				voxel.m_index = skeletonIndex;
				voxel.m_position = newInternode.m_data.m_globalEndPosition;
				voxel.m_direction = newInternode.m_info.m_globalDirection;
				internodeEndGrid.Ref(voxel.m_position).emplace_back(voxel);
			}
		}
		for (auto& skeleton : m_skeletons) {
			skeleton.SortLists();
			skeleton.CalculateDistance();
		}
	}
	CalculateSkeletonGraphs();
}

void TreePointCloud::CalculateBranchRootDistance(
	const std::vector<std::pair<glm::vec3, BranchHandle>>& rootBranchHandles)
{
	for (const auto& rootBranchHandle : rootBranchHandles)
	{
		std::vector<BranchHandle> sortedBranchList{};
		GetSortedBranchList(rootBranchHandle.second, sortedBranchList);
		for(const auto branchHandle : sortedBranchList)
		{
			auto& branch = m_operatingBranches[branchHandle];
			if(branch.m_parentHandle == -1)
			{
				branch.m_rootDistance = 0.0f;
			}else
			{
				const auto& parentBranch = m_operatingBranches[branch.m_parentHandle];
				branch.m_rootDistance = parentBranch.m_rootDistance + glm::distance(parentBranch.m_bezierCurve.m_p0, parentBranch.m_bezierCurve.m_p3) + branch.m_distanceToParentBranch;
				branch.m_skeletonIndex = parentBranch.m_skeletonIndex;
			}
		}
	}
}

void TreePointCloud::CalculateSkeletonGraphs()
{
	for (auto& skeleton : m_skeletons)
	{
		skeleton.SortLists();
		auto& sortedNodeList = skeleton.RefSortedNodeList();
		auto& rootNode = skeleton.RefNode(0);
		rootNode.m_info.m_globalRotation = rootNode.m_info.m_regulatedGlobalRotation = glm::quatLookAt(
			glm::vec3(0, 1, 0), glm::vec3(0, 0, -1));
		rootNode.m_info.m_globalPosition = glm::vec3(0.0f);
		for (const auto& nodeHandle : sortedNodeList) {
			auto& node = skeleton.RefNode(nodeHandle);
			auto& nodeInfo = node.m_info;
			auto& nodeData = node.m_data;
			if (nodeHandle == 0) continue;

			auto& parentNode = skeleton.RefNode(node.GetParentHandle());
			auto diff = nodeData.m_globalEndPosition - parentNode.m_data.m_globalEndPosition;
			auto front = glm::normalize(diff);
			auto parentUp = parentNode.m_info.m_globalRotation * glm::vec3(0, 1, 0);
			auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentUp), front));
			nodeInfo.m_globalRotation = glm::quatLookAt(front, regulatedUp);
			nodeInfo.m_length = glm::length(diff);

		}

		for (auto i = sortedNodeList.rbegin(); i != sortedNodeList.rend(); ++i) {
			auto& node = skeleton.RefNode(*i);
			auto& nodeData = node.m_data;
			auto& childHandles = node.RefChildHandles();
			if (childHandles.empty())
			{
				nodeData.m_draftThickness = m_reconstructionSettings.m_endNodeThickness;
			}
			else {
				float childThicknessCollection = 0.0f;
				for (const auto& childHandle : childHandles) {
					const auto& childNode = skeleton.RefNode(childHandle);
					childThicknessCollection += glm::pow(childNode.m_data.m_draftThickness + m_reconstructionSettings.m_thicknessAccumulationFactor,
						1.0f / m_reconstructionSettings.m_thicknessSumFactor);
				}
				nodeData.m_draftThickness = glm::pow(childThicknessCollection,
					m_reconstructionSettings.m_thicknessSumFactor);
			}
		}
		const auto rootNodeThickness = skeleton.PeekNode(0).m_data.m_draftThickness;
		if (rootNodeThickness < m_reconstructionSettings.m_minimumRootThickness)
		{
			float multiplierFactor = m_reconstructionSettings.m_minimumRootThickness / rootNodeThickness;
			for (const auto& handle : sortedNodeList)
			{
				auto& nodeData = skeleton.RefNode(handle).m_data;
				nodeData.m_draftThickness *= multiplierFactor;
			}
		}
		skeleton.CalculateDistance();
		for (auto i = sortedNodeList.rbegin(); i != sortedNodeList.rend(); ++i) {
			auto& node = skeleton.RefNode(*i);
			auto& nodeData = node.m_data;
			auto& nodeInfo = node.m_info;
			if (nodeInfo.m_rootDistance >= m_reconstructionSettings.m_overrideThicknessRootDistance)
			{
				nodeInfo.m_thickness = nodeData.m_draftThickness;
			}
			if (m_reconstructionSettings.m_limitParentThickness)
			{
				auto& childHandles = node.RefChildHandles();
				float maxChildThickness = 0.0f;
				for (const auto& childHandle : childHandles)
				{
					maxChildThickness = glm::max(maxChildThickness, skeleton.PeekNode(childHandle).m_info.m_thickness);
				}
				nodeInfo.m_thickness = glm::max(nodeInfo.m_thickness, maxChildThickness);
			}
		}

		CalculateNodeTransforms(skeleton);
		skeleton.CalculateFlows();
	}
}

void TreePointCloud::ClearMeshes() const {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name.substr(0, 4) == "Tree") {
			scene->DeleteEntity(child);
		}
	}
}

void TreePointCloud::OnCreate()
{
	m_treeMeshGeneratorSettings.m_enableFoliage = true;
	m_treeMeshGeneratorSettings.m_enableTwig = true;
	m_treeMeshGeneratorSettings.m_foliageOverrideSettings.m_leafSize = { 0.0125f, 0.0125f };
	m_treeMeshGeneratorSettings.m_foliageOverrideSettings.m_positionVariance = 0.06f;
	m_treeMeshGeneratorSettings.m_foliageOverrideSettings.m_leafCountPerInternode = 15;
	m_treeMeshGeneratorSettings.m_foliageOverrideSettings.m_rotationVariance = 1.0f;
	m_treeMeshGeneratorSettings.m_foliageOverrideSettings.m_minRootDistance = 1.15f;
	m_treeMeshGeneratorSettings.m_foliageOverrideSettings.m_maxEndDistance = 0.1f;
}

void TreePointCloud::FormGeometryEntity() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	const auto& meshGeneratorSettings = m_treeMeshGeneratorSettings;
	ClearMeshes();
	for (int i = 0; i < m_skeletons.size(); i++) {
		Entity treeEntity;
		treeEntity = scene->CreateEntity("Tree " + std::to_string(i));
		scene->SetParent(treeEntity, self);
		GlobalTransform gt{};
		gt.SetPosition(m_skeletons[i].m_data.m_rootPosition);
		scene->SetDataComponent(treeEntity, gt);

		if (meshGeneratorSettings.m_enableBranch) {
			Entity branchEntity;
			branchEntity = scene->CreateEntity("Branch Mesh");
			scene->SetParent(branchEntity, treeEntity);
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;

			switch (meshGeneratorSettings.m_branchMeshType)
			{
			case 0:
			{
				CylindricalMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> meshGenerator;
				meshGenerator.Generate(m_skeletons[i], vertices, indices, meshGeneratorSettings, [&](float xFactor, float yFactor)
					{
						return 1.0f;
					});
			}
			break;
			case 1:
			{
				 
				VoxelMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> meshGenerator;
				meshGenerator.Generate(m_skeletons[i], vertices, indices,
					meshGeneratorSettings );
			}
			break;
			default: break;
			}

			auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			VertexAttributes attributes{};
			attributes.m_texCoord = true;
			mesh->SetVertices(attributes, vertices, indices);
			auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
			if (meshGeneratorSettings.m_presentationOverride) {
				material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
			}
			else {
				material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
			}
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
			meshRenderer->m_mesh = mesh;
			meshRenderer->m_material = material;
		}
		if (meshGeneratorSettings.m_enableTwig)
		{
			Entity twigEntity;
			twigEntity = scene->CreateEntity("Twig Mesh");
			scene->SetParent(twigEntity, treeEntity);
			std::vector<glm::uint> twigSegments;
			std::vector<StrandPoint> twigPoints;
			const auto& shootSkeleton = m_skeletons[i];
			const auto& internodeList = shootSkeleton.RefSortedNodeList();
			for (int internodeHandle : internodeList)
			{
				const auto& internode = shootSkeleton.PeekNode(internodeHandle);
				const auto& internodeInfo = internode.m_info;
				std::vector<std::vector<glm::vec4>> twigs{};
				if (internodeInfo.m_thickness < meshGeneratorSettings.m_twigParameters.m_maxNodeThickness
					&& internodeInfo.m_rootDistance > meshGeneratorSettings.m_twigParameters.m_minRootDistance
					&& internodeInfo.m_endDistance < meshGeneratorSettings.m_twigParameters.m_maxEndDistance)
				{
					int twigCount = internodeInfo.m_length / meshGeneratorSettings.m_twigParameters.m_unitDistance;
					twigs.resize(twigCount);

					auto desiredGlobalRotation = internodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
						glm::radians(meshGeneratorSettings.m_twigParameters.m_branchingAngle), 0.0f,
						glm::radians(glm::radians(
							glm::linearRand(0.0f,
								360.0f)))));

					glm::vec3 directionStart = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
					glm::vec3 directionEnd = directionStart;

					glm::vec3 positionStart = internodeInfo.m_globalPosition;
					glm::vec3 positionEnd =
						positionStart + internodeInfo.m_length * (meshGeneratorSettings.m_smoothness ? 1.0f - meshGeneratorSettings.m_baseControlPointRatio * 0.5f : 1.0f) * internodeInfo.m_globalDirection;

					BezierCurve curve = BezierCurve(
						positionStart,
						positionStart +
						(meshGeneratorSettings.m_smoothness ? internodeInfo.m_length * meshGeneratorSettings.m_baseControlPointRatio : 0.0f) * directionStart,
						positionEnd -
						(meshGeneratorSettings.m_smoothness ? internodeInfo.m_length * meshGeneratorSettings.m_branchControlPointRatio : 0.0f) * directionEnd,
						positionEnd);

					for (int twigIndex = 0; twigIndex < twigCount; twigIndex++) {
						glm::vec3 positionWalker = curve.GetPoint(static_cast<float>(twigIndex) / twigCount);
						twigs[twigIndex].resize(meshGeneratorSettings.m_twigParameters.m_segmentSize);
						const float rollAngle = glm::radians(glm::linearRand(0.0f, 360.0f));
						for (int twigPointIndex = 0; twigPointIndex < meshGeneratorSettings.m_twigParameters.m_segmentSize; twigPointIndex++)
						{
							twigs[twigIndex][twigPointIndex] = glm::vec4(positionWalker, meshGeneratorSettings.m_twigParameters.m_thickness);
							desiredGlobalRotation = internodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
								glm::radians(glm::gaussRand(0.f, meshGeneratorSettings.m_twigParameters.m_apicalAngleVariance) + meshGeneratorSettings.m_twigParameters.m_branchingAngle), 0.0f,
								rollAngle));

							auto twigFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
							positionWalker = positionWalker + twigFront * meshGeneratorSettings.m_twigParameters.m_segmentLength;
						}
					}

				}

				for (const auto& twig : twigs) {
					const auto twigSegmentSize = twig.size();
					const auto twigControlPointSize = twig.size() + 3;
					const auto totalTwigPointSize = twigPoints.size();
					const auto totalTwigSegmentSize = twigSegments.size();
					twigPoints.resize(totalTwigPointSize + twigControlPointSize);
					twigSegments.resize(totalTwigSegmentSize + twigSegmentSize);

					for (int i = 0; i < twigControlPointSize; i++) {
						auto& p = twigPoints[totalTwigPointSize + i];
						p.m_position = glm::vec3(twig[glm::clamp(i - 2, 0, static_cast<int>(twigSegmentSize - 1))]);
						p.m_thickness = twig[glm::clamp(i - 2, 0, static_cast<int>(twigSegmentSize - 1))].w;
					}
					twigPoints[totalTwigPointSize].m_position = glm::vec3(twig[0]) * 2.0f - glm::vec3(twig[1]);
					twigPoints[totalTwigPointSize].m_thickness = twig[0].w * 2.0f - twig[1].w;

					twigPoints[totalTwigPointSize + twigControlPointSize - 1].m_position = glm::vec3(twig[twigSegmentSize - 1]) * 2.0f - glm::vec3(twig[twigSegmentSize - 2]);
					twigPoints[totalTwigPointSize + twigControlPointSize - 1].m_thickness = twig[twigSegmentSize - 1].w * 2.0f - twig[twigSegmentSize - 2].w;


					for (int i = 0; i < twigSegmentSize; i++) {
						twigSegments[totalTwigSegmentSize + i] = totalTwigPointSize + i;
					}
				}
			}

			auto strands = ProjectManager::CreateTemporaryAsset<Strands>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			if (meshGeneratorSettings.m_foliageOverride)
			{
				material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
			}
			else {
				material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
			}
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
			StrandPointAttributes strandPointAttributes{};
			strands->SetSegments(strandPointAttributes, twigSegments, twigPoints);
			auto strandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(twigEntity).lock();
			strandsRenderer->m_strands = strands;
			strandsRenderer->m_material = material;
		}
		if (meshGeneratorSettings.m_enableFoliage)
		{
			Entity foliageEntity;
			foliageEntity = scene->CreateEntity("Foliage Mesh");
			scene->SetParent(foliageEntity, treeEntity);
			const auto& shootSkeleton = m_skeletons[i];
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;
			auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
			auto& quadTriangles = quadMesh->UnsafeGetTriangles();
			auto quadVerticesSize = quadMesh->GetVerticesAmount();
			size_t offset = 0;

			const auto& nodeList = shootSkeleton.RefSortedNodeList();
			for (const auto& internodeHandle : nodeList) {
				const auto& internode = shootSkeleton.PeekNode(internodeHandle);
				const auto& internodeInfo = internode.m_info;

				const auto& foliageOverrideSettings = meshGeneratorSettings.m_foliageOverrideSettings;
				if (internodeInfo.m_thickness < foliageOverrideSettings.m_maxNodeThickness
					&& internodeInfo.m_rootDistance > foliageOverrideSettings.m_minRootDistance
					&& internodeInfo.m_endDistance < foliageOverrideSettings.m_maxEndDistance) {
					for (int i = 0; i < foliageOverrideSettings.m_leafCountPerInternode; i++)
					{
						auto leafSize = foliageOverrideSettings.m_leafSize;
						glm::quat rotation = internodeInfo.m_globalDirection * glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
						auto front = rotation * glm::vec3(0, 0, -1);
						auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.y * 1.5f) + glm::sphericalRand(1.0f) * glm::linearRand(0.0f, foliageOverrideSettings.m_positionVariance);
						auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(leafSize.x, 1.0f, leafSize.y));

						auto& matrix = leafTransform;
						Vertex archetype;
						for (auto i = 0; i < quadMesh->GetVerticesAmount(); i++) {
							archetype.m_position =
								matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_position, 1.0f);
							archetype.m_normal = glm::normalize(glm::vec3(
								matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
							archetype.m_tangent = glm::normalize(glm::vec3(
								matrix *
								glm::vec4(quadMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
							archetype.m_texCoord =
								quadMesh->UnsafeGetVertices()[i].m_texCoord;
							vertices.push_back(archetype);
						}
						for (auto triangle : quadTriangles) {
							triangle.x += offset;
							triangle.y += offset;
							triangle.z += offset;
							indices.push_back(triangle.x);
							indices.push_back(triangle.y);
							indices.push_back(triangle.z);
						}
						offset += quadVerticesSize;
					}
				}

			}

			auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			VertexAttributes vertexAttributes{};
			vertexAttributes.m_texCoord = true;
			mesh->SetVertices(vertexAttributes, vertices, indices);
			if (meshGeneratorSettings.m_foliageOverride)
			{
				material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_foliageOverrideSettings.m_leafColor;
			}
			else {
				material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
			}
			material->m_materialProperties.m_roughness = 0.0f;
			auto texRef = meshGeneratorSettings.m_foliageAlbedoTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetAlbedoTexture(texRef.Get<Texture2D>());

			}
			auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliageEntity).lock();
			meshRenderer->m_mesh = mesh;
			meshRenderer->m_material = material;

		}
	}
}

std::vector<std::shared_ptr<Mesh>> TreePointCloud::GenerateForestBranchMeshes() const
{
	std::vector<std::shared_ptr<Mesh>> meshes{};
	for (int i = 0; i < m_skeletons.size(); i++) {
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		CylindricalMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> meshGenerator;
		meshGenerator.Generate(m_skeletons[i], vertices, indices, m_treeMeshGeneratorSettings, [&](float xFactor, float yFactor)
			{
				return 1.0f;
			});
		Jobs::ParallelFor(vertices.size(), [&](unsigned j)
			{
				vertices[j].m_position += m_skeletons[i].m_data.m_rootPosition;
			});
		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		VertexAttributes attributes{};
		attributes.m_texCoord = true;
		mesh->SetVertices(attributes, vertices, indices);
		meshes.emplace_back(mesh);
	}
	return meshes;
}

std::vector<std::shared_ptr<Mesh>> TreePointCloud::GenerateFoliageMeshes() const
{
	std::vector<std::shared_ptr<Mesh>> meshes{};
	for (int i = 0; i < m_skeletons.size(); i++) {
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;

		auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
		auto& quadTriangles = quadMesh->UnsafeGetTriangles();
		auto quadVerticesSize = quadMesh->GetVerticesAmount();
		size_t offset = 0;

		const auto& nodeList = m_skeletons[i].RefSortedNodeList();
		for (const auto& internodeHandle : nodeList) {
			const auto& internode = m_skeletons[i].PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;

			const auto& foliageOverrideSettings = m_treeMeshGeneratorSettings.m_foliageOverrideSettings;
			if (internodeInfo.m_thickness < foliageOverrideSettings.m_maxNodeThickness
				&& internodeInfo.m_rootDistance > foliageOverrideSettings.m_minRootDistance
				&& internodeInfo.m_endDistance < foliageOverrideSettings.m_maxEndDistance) {
				for (int i = 0; i < foliageOverrideSettings.m_leafCountPerInternode; i++)
				{
					auto leafSize = foliageOverrideSettings.m_leafSize;
					glm::quat rotation = internodeInfo.m_globalDirection * glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
					auto front = rotation * glm::vec3(0, 0, -1);
					auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.y * 1.5f) + glm::sphericalRand(1.0f) * glm::linearRand(0.0f, foliageOverrideSettings.m_positionVariance);
					auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(leafSize.x, 1.0f, leafSize.y));

					auto& matrix = leafTransform;
					Vertex archetype;
					for (auto i = 0; i < quadMesh->GetVerticesAmount(); i++) {
						archetype.m_position =
							matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_position, 1.0f);
						archetype.m_normal = glm::normalize(glm::vec3(
							matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
						archetype.m_tangent = glm::normalize(glm::vec3(
							matrix *
							glm::vec4(quadMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
						archetype.m_texCoord =
							quadMesh->UnsafeGetVertices()[i].m_texCoord;
						vertices.push_back(archetype);
					}
					for (auto triangle : quadTriangles) {
						triangle.x += offset;
						triangle.y += offset;
						triangle.z += offset;
						indices.push_back(triangle.x);
						indices.push_back(triangle.y);
						indices.push_back(triangle.z);
					}
					offset += quadVerticesSize;
				}
			}

		}
		Jobs::ParallelFor(vertices.size(), [&](unsigned j)
			{
				vertices[j].m_position += m_skeletons[i].m_data.m_rootPosition;
			});
		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		VertexAttributes attributes{};
		attributes.m_texCoord = true;
		mesh->SetVertices(attributes, vertices, indices);
		meshes.emplace_back(mesh);
	}
	return meshes;
}


void ConnectivityGraphSettings::OnInspect() {
	//ImGui::Checkbox("Allow Reverse connections", &m_reverseConnection);
	if(ImGui::Button("Load reduced connection settings"))
	{
		m_pointPointConnectionDetectionRadius = 0.05f;
		m_pointBranchConnectionDetectionRadius = 0.1f;
		m_branchBranchConnectionMaxLengthRange = 15.0f;
		m_directionConnectionAngleLimit = 30.0f;
		m_indirectConnectionAngleLimit = 15.0f;
		m_maxScatterPointConnectionHeight = 1.5f;
		m_parallelShiftCheck = true;
	}
	if (ImGui::Button("Load default connection settings"))
	{
		m_pointPointConnectionDetectionRadius = 0.05f;
		m_pointBranchConnectionDetectionRadius = 0.1f;
		m_branchBranchConnectionMaxLengthRange = 5.0f;
		m_directionConnectionAngleLimit = 65.0f;
		m_indirectConnectionAngleLimit = 65.0f;
		m_maxScatterPointConnectionHeight = 1.5f;
		m_parallelShiftCheck = true;
	}
	if (ImGui::Button("Load max connection settings"))
	{
		m_pointPointConnectionDetectionRadius = 0.05f;
		m_pointBranchConnectionDetectionRadius = 0.1f;
		m_branchBranchConnectionMaxLengthRange = 10.0f;
		m_directionConnectionAngleLimit = 90.0f;
		m_indirectConnectionAngleLimit = 90.0f;
		m_maxScatterPointConnectionHeight = 10.f;
		m_parallelShiftCheck = false;
	}

	ImGui::DragFloat("Point-point connection max height", &m_maxScatterPointConnectionHeight, 0.01f, 0.01f, 3.0f);
	ImGui::DragFloat("Point-point detection radius", &m_pointPointConnectionDetectionRadius, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Point-branch detection radius", &m_pointBranchConnectionDetectionRadius, 0.01f, 0.01f, 2.0f);
	ImGui::DragFloat("Branch-branch detection range", &m_branchBranchConnectionMaxLengthRange, 0.01f, 0.01f, 2.0f);
	ImGui::DragFloat("Direct connection angle limit", &m_directionConnectionAngleLimit, 0.01f, 0.0f, 180.0f);
	ImGui::DragFloat("Indirect connection angle limit", &m_indirectConnectionAngleLimit, 0.01f, 0.0f, 180.0f);

	ImGui::Checkbox("Zigzag check", &m_zigzagCheck);
	if (m_zigzagCheck)
	{
		ImGui::DragFloat("Zigzag branch shortening", &m_zigzagBranchShortening, 0.01f, 0.0f, 0.5f);
	}
	ImGui::Checkbox("Parallel shift check", &m_parallelShiftCheck);
	if (m_parallelShiftCheck) ImGui::DragFloat("Parallel Shift range limit", &m_parallelShiftLimitRange, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Point existence check radius", &m_pointCheckRadius, 0.01f, 0.0f, 1.0f);
}

void TreePointCloud::CloneOperatingBranch(const ReconstructionSettings& reconstructionSettings, OperatorBranch& operatorBranch, const PredictedBranch& target) {
	operatorBranch.m_color = target.m_color;
	operatorBranch.m_treePartHandle = target.m_treePartHandle;
	operatorBranch.m_handle = target.m_handle;
	operatorBranch.m_bezierCurve = target.m_bezierCurve;
	operatorBranch.m_thickness = target.m_branchThickness;
	operatorBranch.m_parentHandle = -1;
	operatorBranch.m_childHandles.clear();
	operatorBranch.m_orphan = false;
	operatorBranch.m_parentCandidates.clear();
	int count = 0;
	for (const auto& data : target.m_p3ToP0)
	{
		operatorBranch.m_parentCandidates.emplace_back(data.first, data.second);
		count++;
		if(count > reconstructionSettings.m_maxParentCandidateSize) break;
	}
	operatorBranch.m_distanceToParentBranch = 0.0f;
	operatorBranch.m_bestDistance = FLT_MAX;
	operatorBranch.m_rootDistance = 0.0f;
	operatorBranch.m_descendentSize = 0;
	operatorBranch.m_skeletonIndex = -1;
	operatorBranch.m_used = false;
	operatorBranch.m_chainNodeHandles.clear();
}



void ReconstructionSettings::OnInspect() {
	ImGui::DragFloat("Internode length", &m_internodeLength, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Root node max height", &m_minHeight, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Tree distance limit", &m_minimumTreeDistance, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Branch shortening", &m_branchShortening, 0.01f, 0.01f, 0.4f);
	ImGui::DragInt("Max parent candidate size", &m_maxParentCandidateSize, 1, 2, 10);
	ImGui::DragInt("Max child size", &m_maxChildSize, 1, 2, 10);

	ImGui::DragFloat("Override thickness root distance", &m_overrideThicknessRootDistance, 0.01f, 0.01f, 0.5f);
	ImGui::DragFloat("Space colonization factor", &m_spaceColonizationFactor, 0.01f, 0.f, 1.0f);
	if(m_spaceColonizationFactor > 0.0f)
	{
		ImGui::DragInt("Space colonization timeout", &m_spaceColonizationTimeout, 1, 0, 500);
		ImGui::DragFloat("Space colonization removal distance", &m_spaceColonizationRemovalDistanceFactor, 0.1f, 0.f, 10.0f);
		ImGui::DragFloat("Space colonization detection distance", &m_spaceColonizationDetectionDistanceFactor, 0.1f, 0.f, 20.0f);
		ImGui::DragFloat("Space colonization perception theta", &m_spaceColonizationTheta, 0.1f, 0.f, 90.0f);
	}
	ImGui::DragFloat("End node thickness", &m_endNodeThickness, 0.001f, 0.001f, 1.0f);
	ImGui::DragFloat("Thickness sum factor", &m_thicknessSumFactor, 0.01f, 0.0f, 2.0f);
	ImGui::DragFloat("Thickness accumulation factor", &m_thicknessAccumulationFactor, 0.00001f, 0.0f, 1.0f, "%.5f");
	ImGui::Checkbox("Limit parent thickness", &m_limitParentThickness);
	ImGui::DragFloat("Minimum root thickness", &m_minimumRootThickness, 0.001f, 0.0f, 1.0f, "%.3f");
	ImGui::DragInt("Minimum node count", &m_minimumNodeCount, 1, 0, 100);

	ImGui::DragInt("Node back track limit", &m_nodeBackTrackLimit, 1, 0, 100);
	ImGui::DragInt("Branch back track limit", &m_branchBackTrackLimit, 1, 0, 10);

	ImGui::Checkbox("Use root distance", &m_useRootDistance);

	ImGui::DragInt("Optimization timeout", &m_optimizationTimeout, 1, 0, 100);

	ImGui::DragFloat("Direction smoothing", &m_directionSmoothing, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Position smoothing", &m_positionSmoothing, 0.01f, 0.0f, 1.0f);
	ImGui::DragInt("Smoothing iteration", &m_smoothIteration, 1, 0, 100);
	/*
	ImGui::Checkbox("Candidate Search", &m_candidateSearch);
	if (m_candidateSearch) ImGui::DragInt("Candidate Search limit", &m_candidateSearchLimit, 1, 0, 10);
	ImGui::Checkbox("Force connect all branches", &m_forceConnectAllBranches);
	*/
}
