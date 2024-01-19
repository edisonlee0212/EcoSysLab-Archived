#include "TreePipeMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"
#include "TreeMeshGenerator.hpp"
#include "opensubdiv/bfr/faceSurface.h"
#include "regression/bfr_evaluate/bfrSurfaceEvaluator.h"
using namespace EcoSysLab;

void TreePipeMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{

	ImGui::Checkbox("Auto set level", &m_autoLevel);
	if (!m_autoLevel) ImGui::DragInt("Voxel subdivision level", &m_voxelSubdivisionLevel, 1, 5, 16);
	else ImGui::DragFloat("Min Cube size", &m_marchingCubeRadius, 0.0001, 0.001f, 1.0f);
	ImGui::DragInt("Smooth iteration", &m_voxelSmoothIteration, 0, 0, 10);
	if (m_voxelSmoothIteration == 0) ImGui::Checkbox("Remove duplicate", &m_removeDuplicate);

	ImGui::DragInt("Marching cube min cell size", &m_minimumParticleSizeForMarchingCube, 1, 0, 1000);
	ImGui::DragInt("Cylindrical max cell size", &m_maximumParticleSizeForCylindrical, 1, 0, 1000);

	ImGui::ColorEdit4("Marching cube color", &m_marchingCubeColor.x);
	ImGui::ColorEdit4("Cylindrical color", &m_cylindricalColor.x);
	ImGui::ColorEdit3("Vertex color", &m_vertexColor.x);

}

int roundInDir(float val, int dir)
{
	if (dir > 0)
	{
		return int(std::ceilf(val));
	}
	else
	{
		return int(std::floorf(val));
	}
}

glm::ivec3 roundInDir(glm::vec3 val, glm::ivec3 dir)
{
	return glm::ivec3(roundInDir(val[0], dir[0]), roundInDir(val[1], dir[1]), roundInDir(val[2], dir[2]));
}

std::vector<glm::ivec3> voxelizeLineSeg(glm::vec3 start, glm::vec3 end, float voxelSideLength)
{
	// Based on Amanatides, J., & Woo, A. (1987, August). A fast voxel traversal algorithm for ray tracing. In Eurographics (Vol. 87, No. 3, pp. 3-10).
	std::vector<glm::ivec3> retVal;
	glm::vec3 dir = end - start;
	glm::vec3 dirInVoxels = dir / voxelSideLength;
	glm::ivec3 step = glm::sign(dir);

	// determine voxel of start point
	glm::vec3 posInVoxels = start / voxelSideLength;
	glm::ivec3 startVoxel = glm::floor(posInVoxels);
	retVal.push_back(startVoxel);
	glm::ivec3 endVoxel = glm::floor(end / voxelSideLength);

	// compute t deltas
	glm::vec3 tDelta = glm::vec3(voxelSideLength) / glm::abs(dir);

	// compute max value for t within voxel
	glm::ivec3 nextBorder = roundInDir(posInVoxels, step);
	glm::vec3 distToNextBorder = glm::vec3(nextBorder) - posInVoxels;
	glm::vec3 tMax = glm::abs(distToNextBorder / dirInVoxels);
	if (dirInVoxels.x <= glm::epsilon<float>())tMax.x = 0.0f;
	if (dirInVoxels.y <= glm::epsilon<float>())tMax.y = 0.0f;
	if (dirInVoxels.z <= glm::epsilon<float>())tMax.z = 0.0f;
	// min and max for debug assert
	glm::vec3 minVoxel = glm::min(startVoxel, endVoxel);
	glm::vec3 maxVoxel = glm::max(startVoxel, endVoxel);

	// now traverse voxels until we reach the end point
	glm::ivec3 voxel = startVoxel;
	while (voxel != endVoxel)
	{
		size_t minIndex = 0;

		for (size_t i = 1; i < 3; i++)
		{
			if (tMax[i] < tMax[minIndex])
			{
				minIndex = i;
			}
		}

		// update tMax and determine next voxel;
		voxel[minIndex] += step[minIndex];


		// check that we do not run out of range
		// This can happen due to numerical inaccuracies when adding tDelta
		for (size_t dim = 0; dim < 3; dim++)
		{
			if (!(minVoxel[dim] <= voxel[dim] && voxel[dim] <= maxVoxel[dim]))
			{
				retVal.push_back(endVoxel);
				return retVal;
			}
		}

		retVal.push_back(voxel);
		tMax[minIndex] += tDelta[minIndex];

	}

	return retVal;
}

void TreePipeMeshGenerator::Generate(
	const TreeModel& treeModel, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	const auto& skeleton = treeModel.PeekShootSkeleton();
	const auto& pipeGroup = skeleton.m_data.m_pipeGroup;

	for (const auto& pipe : pipeGroup.PeekPipes())
	{
		for (const auto& pipeSegmentHandle : pipe.PeekPipeSegmentHandles())
		{
			const auto& pipeSegment = pipeGroup.PeekPipeSegment(pipeSegmentHandle);
			//Get interpolated position on pipe segment. Example to get middle point here:
			const auto middlePoint = treeModel.InterpolatePipeSegmentPosition(pipeSegmentHandle, 0.5f);
			//Get interpolated direction on pipe segment. The axis is a normalized vector that points from the point close to root to end (Bottom up)
			const auto middleAxis = treeModel.InterpolatePipeSegmentAxis(pipeSegmentHandle, 0.5f);


			const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
			//To access the user's defined constraints (attractors, etc.)
			const auto& profileConstraints = node.m_data.m_profileConstraints;

			//To access the position of the start of the pipe segment within a profile:
			const auto parentHandle = node.GetParentHandle();
			if (parentHandle == -1)
			{
				//If the node is the first node, the corresponding pipe segment will also be the first one in the pipe.
				assert(pipeSegmentHandle == pipe.PeekPipeSegmentHandles().front());
				//If current node is the first node of the tree, it's front profile will be used to calculate the start position of the first segment of the pipe.
				const auto& startProfile = node.m_data.m_frontProfile;

				//To get the particle that belongs to this pipe segment:
				const auto& startParticle = startProfile.PeekParticle(pipeSegment.m_data.m_frontProfileParticleHandle);

				//To access the boundary of the profile:
				const auto& startProfileBoundaryEdges = startProfile.PeekBoundaryEdges();
				for (const auto& startProfileBoundaryEdgeParticleHandles : startProfileBoundaryEdges)
				{
					const auto& particle1 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.first);
					const auto& particle2 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.second);
					const auto particle1Position = particle1.GetPosition();
					const auto particle2Position = particle2.GetPosition();
				}
			}
			else
			{
				//For other nodes, it's parent's back profile will be used to calculate the end position of the segment.
				const auto& startProfile = skeleton.PeekNode(parentHandle).m_data.m_backProfile;

				//You will also need to access the prev segment in order to retrieve the particle:
				const auto& prevPipeSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetPrevHandle());
				const auto& startParticle = startProfile.PeekParticle(prevPipeSegment.m_data.m_backProfileParticleHandle);

				//To access the boundary of the profile:
				const auto& startProfileBoundaryEdges = startProfile.PeekBoundaryEdges();
				for (const auto& startProfileBoundaryEdgeParticleHandles : startProfileBoundaryEdges)
				{
					const auto& particle1 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.first);
					const auto& particle2 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.second);
					const auto particle1Position = particle1.GetPosition();
					const auto particle2Position = particle2.GetPosition();
				}
			}

			const auto& endProfile = node.m_data.m_backProfile;
			const auto& endProfileBoundaryEdges = endProfile.PeekBoundaryEdges();

			//The position of the back particle corresponds to the end of the pipe segment.
			const auto& endParticle = endProfile.PeekParticle(pipeSegment.m_data.m_backProfileParticleHandle);

			//To iterate through all boundary edges of the back profile:
			for (const auto& endProfileBoundaryEdgeParticleHandles : endProfileBoundaryEdges)
			{
				const auto& particle1 = endProfile.PeekParticle(endProfileBoundaryEdgeParticleHandles.first);
				const auto& particle2 = endProfile.PeekParticle(endProfileBoundaryEdgeParticleHandles.second);
				const auto particle1Position = particle1.GetPosition();
				const auto particle2Position = particle2.GetPosition();
			}
		}
	}

	//OpenSubdiv
	Bfr::Surface<float> surface{};

	// first compute extreme points
	glm::vec3 min = glm::vec3(std::numeric_limits<float>::infinity());
	glm::vec3 max = glm::vec3(-std::numeric_limits<float>::infinity());

	for (auto& seg : pipeGroup.PeekPipeSegments())
	{
		min = glm::min(seg.m_info.m_globalPosition, min);
		max = glm::max(seg.m_info.m_globalPosition, max);
	}

	for (auto& pipe : pipeGroup.PeekPipes())
	{
		min = glm::min(pipe.m_info.m_baseInfo.m_globalPosition, min);
		max = glm::max(pipe.m_info.m_baseInfo.m_globalPosition, max);
	}

	const auto boxSize = max - min;
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

			octree.Reset(maxRadius, subdivisionLevel, (min + max) * 0.5f);
	}
	else {
		octree.Reset(glm::max((boxSize.x, boxSize.y), glm::max(boxSize.y, boxSize.z)) * 0.5f,
			glm::clamp(settings.m_voxelSubdivisionLevel, 4, 16), (min + max) / 2.0f);
	}

	std::cout << "voxel size: " << settings.m_marchingCubeRadius << std::endl;

	// loop over each strand in the pipe group
	for (auto& pipe : pipeGroup.PeekPipes())
	{
		glm::vec3 segStart = pipe.m_info.m_baseInfo.m_globalPosition;

		for (auto& segHandle : pipe.PeekPipeSegmentHandles())
		{
			auto& seg = pipeGroup.PeekPipeSegment(segHandle);
			glm::vec3 segEnd = seg.m_info.m_globalPosition;

			std::vector<glm::ivec3> pipeVoxels = voxelizeLineSeg(segStart, segEnd, settings.m_marchingCubeRadius);

			// insert each voxel from the segment into the octree
			for (auto& voxel : pipeVoxels)
			{
				octree.Occupy((glm::vec3(voxel) + glm::vec3(0.5f)) * settings.m_marchingCubeRadius, [](OctreeNode&) {});
			}

			segStart = segEnd;
		}

	}

	octree.TriangulateField(vertices, indices, settings.m_removeDuplicate, settings.m_voxelSmoothIteration);

	// finally set vertex colors (does not work yet)
	for (Vertex& v : vertices)
	{
		v.m_color = glm::vec4(settings.m_vertexColor, 1.0f);
	}
}

void TreePipeMeshGenerator::Generate2(const TreeModel& treeModel, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	const auto& skeleton = treeModel.PeekShootSkeleton();
	const auto& pipeGroup = skeleton.m_data.m_pipeGroup;
	// first compute extreme points
	auto min = glm::vec3(std::numeric_limits<float>::infinity());
	auto max = glm::vec3(-std::numeric_limits<float>::infinity());
	for (const auto& pipeSegment : pipeGroup.PeekPipeSegments())
	{
		const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
		const auto& profile = node.m_data.m_backProfile;
		if (profile.PeekParticles().size() < settings.m_minimumParticleSizeForMarchingCube) continue;
		min = glm::min(pipeSegment.m_info.m_globalPosition, min);
		max = glm::max(pipeSegment.m_info.m_globalPosition, max);
	}
	min -= glm::vec3(0.1f);
	max += glm::vec3(0.1f);
	const auto boxSize = max - min;
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
			octree.Reset(maxRadius, subdivisionLevel, (min + max) * 0.5f);
	}
	else {
		octree.Reset(glm::max((boxSize.x, boxSize.y), glm::max(boxSize.y, boxSize.z)) * 0.5f,
			glm::clamp(settings.m_voxelSubdivisionLevel, 4, 16), (min + max) / 2.0f);
	}
	float subdivisionLength = settings.m_marchingCubeRadius * 0.5f;
	for (const auto& pipeSegment : pipeGroup.PeekPipeSegments())
	{
		const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
		const auto& profile = node.m_data.m_backProfile;
		if (profile.PeekParticles().size() < settings.m_minimumParticleSizeForMarchingCube) continue;
		//Get interpolated position on pipe segment. Example to get middle point here:
		const auto startPosition = treeModel.InterpolatePipeSegmentPosition(pipeSegment.GetHandle(), 0.0f);
		const auto endPosition = treeModel.InterpolatePipeSegmentPosition(pipeSegment.GetHandle(), 1.0f);
		const auto distance = glm::distance(startPosition, endPosition);
		const auto stepSize = glm::max(1, static_cast<int>(distance / subdivisionLength));
		for (int step = 0; step < stepSize; step++)
		{
			const auto a = treeModel.InterpolatePipeSegmentPosition(pipeSegment.GetHandle(), static_cast<float>(step) / stepSize);
			octree.Occupy(a, [](OctreeNode&) {});
		}
	}
	octree.TriangulateField(vertices, indices, settings.m_removeDuplicate, settings.m_voxelSmoothIteration);
	for(auto& i : vertices) i.m_color = settings.m_marchingCubeColor;

	const auto& sortedInternodeList = skeleton.RefSortedNodeList();
	std::vector<std::vector<RingSegment>> ringsList;
	std::map<NodeHandle, int> steps{};
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

		if (internode.m_data.m_backProfile.PeekParticles().size() > settings.m_maximumParticleSizeForCylindrical) return;

		glm::vec3 p[4];
		glm::vec3 f[4];

		float t[4];
		p[2] = internode.m_data.m_adjustedGlobalPosition;
		f[2] = internode.m_data.m_adjustedGlobalRotation * glm::vec3(0, 0, -1);
		t[2] = glm::sqrt(static_cast<float>(internode.m_data.m_backProfile.PeekParticles().size())) * internode.m_data.m_pipeCellRadius;
		if (internode.GetParentHandle() == -1)
		{
			p[1] = internode.m_info.m_globalPosition;
			p[0] = p[1] * 2.0f - p[2];

			f[1] = internode.m_info.m_globalRotation * glm::vec3(0, 0, -1);
			f[0] = f[1] * 2.0f - f[2];

			t[2] = glm::sqrt(static_cast<float>(internode.m_data.m_backProfile.PeekParticles().size())) * internode.m_data.m_pipeCellRadius;
			t[0] = t[1] * 2.0f - t[2];
		}
		else if (internode.GetParentHandle() == 0)
		{
			p[0] = internode.m_info.m_globalPosition;
			p[1] = skeleton.PeekNode(0).m_data.m_adjustedGlobalPosition;

			f[0] = internode.m_info.m_globalRotation * glm::vec3(0, 0, -1);
			f[1] = skeleton.PeekNode(0).m_data.m_adjustedGlobalRotation * glm::vec3(0, 0, -1);

			t[0] = glm::sqrt(static_cast<float>(internode.m_data.m_backProfile.PeekParticles().size())) * internode.m_data.m_pipeCellRadius;
			t[1] = glm::sqrt(static_cast<float>(skeleton.PeekNode(0).m_data.m_backProfile.PeekParticles().size())) * internode.m_data.m_pipeCellRadius;
		}
		else
		{
			p[1] = skeleton.PeekNode(internode.GetParentHandle()).m_data.m_adjustedGlobalPosition;
			p[0] = skeleton.PeekNode(skeleton.PeekNode(internode.GetParentHandle()).GetParentHandle()).m_data.m_adjustedGlobalPosition;

			f[1] = skeleton.PeekNode(internode.GetParentHandle()).m_data.m_adjustedGlobalRotation * glm::vec3(0, 0, -1);
			f[0] = skeleton.PeekNode(skeleton.PeekNode(internode.GetParentHandle()).GetParentHandle()).m_data.m_adjustedGlobalRotation * glm::vec3(0, 0, -1);

			t[1] = glm::sqrt(static_cast<float>(skeleton.PeekNode(internode.GetParentHandle()).m_data.m_backProfile.PeekParticles().size())) * internode.m_data.m_pipeCellRadius;
			t[0] = glm::sqrt(static_cast<float>(skeleton.PeekNode(skeleton.PeekNode(internode.GetParentHandle()).GetParentHandle()).m_data.m_backProfile.PeekParticles().size())) * internode.m_data.m_pipeCellRadius;
		}
		if (internode.IsEndNode())
		{
			p[3] = p[2] * 2.0f - p[1];

			f[3] = f[2] * 2.0f - f[1];

			t[3] = t[2] * 2.0f - t[1];
		}
		else
		{
			for (const auto childHandle : internode.RefChildHandles())
			{
				const auto& childInternode = skeleton.PeekNode(childHandle);
				if (childInternode.m_data.m_maxChild)
				{
					p[3] = childInternode.m_data.m_adjustedGlobalPosition;

					f[3] = childInternode.m_data.m_adjustedGlobalRotation * glm::vec3(0, 0, -1);

					t[3] = glm::sqrt(static_cast<float>(childInternode.m_data.m_backProfile.PeekParticles().size())) * internode.m_data.m_pipeCellRadius;
				}
			}
		}
#pragma region Subdivision internode here.
		float thicknessStart, thicknessEnd, thicknessStartT, thicknessEndT;
		Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessStart, thicknessStartT, 0.0f);
		Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessEnd, thicknessEndT, 1.0f);
		const auto diameter = glm::max(thicknessStart, thicknessEnd) * 2.0f * glm::pi<float>();
		int step = diameter / settings.m_xSubdivision;
		if (step < 4)
			step = 4;
		if (step % 2 != 0)
			++step;

		tempSteps[threadIndex].emplace_back(internodeHandle, step);
		int amount = glm::max(4.0f, internodeInfo.m_length / (internodeInfo.m_thickness >= settings.m_trunkThickness ? settings.m_trunkYSubdivision : settings.m_branchYSubdivision));
		if (amount % 2 != 0)
			++amount;

		for (int ringIndex = 1; ringIndex <= amount; ringIndex++) {
			const float a = static_cast<float>(ringIndex - 1) / amount;
			const float b = static_cast<float>(ringIndex) / amount;
			glm::vec3 startPosition, endPosition, startAxis, endAxis, tempStart, tempEnd;
			Strands::CubicInterpolation(p[0], p[1], p[2], p[3], startPosition, tempStart, a);
			Strands::CubicInterpolation(p[0], p[1], p[2], p[3], endPosition, tempEnd, b);
			Strands::CubicInterpolation(f[0], f[1], f[2], f[3], startAxis, tempStart, a);
			Strands::CubicInterpolation(f[0], f[1], f[2], f[3], endAxis, tempEnd, b);
			Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessStart, thicknessStartT, a);
			Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessEnd, thicknessEndT, b);
			rings.emplace_back(
				startPosition, endPosition,
				startAxis,
				endAxis,
				thicknessStart, thicknessEnd);
		}
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
	for (int internodeIndex = 0; internodeIndex < sortedInternodeList.size(); internodeIndex++) {
		auto internodeHandle = sortedInternodeList[internodeIndex];
		const auto& internode = skeleton.PeekNode(internodeHandle);
		if (internode.m_data.m_backProfile.PeekParticles().size() > settings.m_maximumParticleSizeForCylindrical) continue;
		const auto& internodeInfo = internode.m_info;
		auto parentInternodeHandle = internode.GetParentHandle();
		bool continuous = false;
		const glm::vec3 up = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
		glm::vec3 parentUp = up;
		if (parentInternodeHandle != -1)
		{
			const auto& parentInternode = skeleton.PeekNode(parentInternodeHandle);
			parentUp = parentInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			if (parentInternode.m_data.m_backProfile.PeekParticles().size() <= settings.m_maximumParticleSizeForCylindrical) continuous = true;
		}
		auto& rings = ringsList[internodeIndex];
		if (rings.empty()) {
			continue;
		}
		// For stitching
		const int step = steps[internodeHandle];
		int pStep = step;
		if (!continuous)
		{
			pStep = steps[parentInternodeHandle];
		}
		float angleStep = 360.0f / static_cast<float>(step);
		float pAngleStep = 360.0f / static_cast<float>(pStep);
		int vertexIndex = vertices.size();
		Vertex archetype;
		archetype.m_color = settings.m_cylindricalColor;
		const auto flowHandle = internode.GetFlowHandle();
		archetype.m_vertexInfo1 = internodeHandle + 1;
		archetype.m_vertexInfo2 = flowHandle + 1;
		float textureXStep = 1.0f / pStep * 4.0f;
		if (!continuous) {
			for (int p = 0; p < pStep; p++) {
				auto& ring = rings.at(0);
				auto direction = ring.GetDirection(
					parentUp, pAngleStep * p, true);
				archetype.m_position = ring.m_startPosition + direction * ring.m_startRadius;
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
				auto& ring = rings.at(ringIndex);
				auto direction = ring.GetDirection(
					up, angleStep * s, false);
				archetype.m_position = ring.m_endPosition + direction * ring.m_endRadius;
				const auto x =
					s < (step / 2) ? s * textureXStep : (step - s) * textureXStep;
				const auto y = ringIndex % 2 == 0 ? 1.0f : 0.0f;
				archetype.m_texCoord = glm::vec2(x, y);
				vertices.push_back(archetype);
			}
			if (ringIndex == 0)
			{
				if (continuous) {
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
				if (!continuous) vertexIndex += pStep;
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
