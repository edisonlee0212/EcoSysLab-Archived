#include "TreePipeNode.hpp"
using namespace EcoSysLab;

void TreePipeNode::PackTask(const PipeModelParameters& pipeModelParameters, bool parallel)
{
	m_frontParticlePhysics2D.m_parallel = parallel;

	const auto iterations = pipeModelParameters.m_maxSimulationIterationCellFactor * m_frontParticlePhysics2D.RefParticles().size();
	for (int i = 0; i < iterations; i++) {
		const auto checkpoint = i > 1 && i % pipeModelParameters.m_stabilizationCheckIteration == 0;
		if (checkpoint)
		{
			if (m_frontParticlePhysics2D.GetMaxMovementSinceCheckpoint() < pipeModelParameters.m_stabilizationMovementDistance) break;
		}
		m_frontParticlePhysics2D.Simulate(1, [&](auto& particle)
			{
				//Apply gravity
				particle.SetPosition(particle.GetPosition() - m_frontParticlePhysics2D.GetMassCenter());
				if (glm::length(particle.GetPosition()) > 0.0f) {
					const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
					particle.SetAcceleration(acceleration);
				}
			}, checkpoint
		);
		if (pipeModelParameters.m_timeout > 0 && i > pipeModelParameters.m_timeout) break;
	}

}

void TreePipeNode::MergeTask(const PipeModelParameters& pipeModelParameters)
{
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto childrenEntities = scene->GetChildren(owner);
	std::vector<std::shared_ptr<TreePipeNode>> childrenNodes;

	std::shared_ptr<TreePipeNode> mainChildNode{};
	for (auto childEntity : childrenEntities)
	{
		if (scene->HasPrivateComponent<TreePipeNode>(childEntity))
		{
			const auto childNode = scene->GetOrSetPrivateComponent<TreePipeNode>(childEntity).lock();
			if (childNode->m_apical) mainChildNode = childNode;
			childrenNodes.push_back(childNode);
		}
	}
	for (const auto& childNode : childrenNodes) childNode->Wait();
	if (!childrenNodes.empty() && !mainChildNode) mainChildNode = childrenNodes.front();
	m_centerDirectionRadius = 0.0f;
	if (childrenNodes.empty())
	{
		for (int i = 0; i < m_frontParticlePhysics2D.RefParticles().size(); i++)
		{
			const auto nextParticlePosition = m_frontParticlePhysics2D.FindAvailablePosition(glm::circularRand(1.0f));
			m_frontParticlePhysics2D.RefParticle(i).SetPosition(nextParticlePosition);
			m_backParticlePhysics2D.RefParticle(i).SetPosition(nextParticlePosition);
			m_frontParticlePhysics2D.RefParticle(i).SetColor(glm::vec4(1.0f));
			m_backParticlePhysics2D.RefParticle(i).SetColor(glm::vec4(1.0f));
		}
		m_needPacking = true;
		return;
	}

	if (childrenNodes.size() == 1)
	{
		//Copy from child flow start to self flow start
		const auto& childNode = childrenNodes.front();
		const auto& childPhysics2D = childNode->m_frontParticlePhysics2D;
		assert(m_frontParticlePhysics2D.PeekParticles().size() == childPhysics2D.PeekParticles().size());
		assert(m_frontParticlePhysics2D.PeekParticles().size() == childPhysics2D.PeekParticles().size());
		for (const auto& childParticle : childPhysics2D.PeekParticles())
		{
			const auto nodeStartParticleHandle = m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
			const auto nodeEndParticleHandle = m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
			auto& nodeStartParticle = m_frontParticlePhysics2D.RefParticle(nodeStartParticleHandle);
			auto& nodeEndParticle = m_backParticlePhysics2D.RefParticle(nodeEndParticleHandle);
			nodeStartParticle.SetColor(childParticle.GetColor());
			nodeStartParticle.SetPosition(childParticle.GetPosition());
			nodeEndParticle.SetColor(childParticle.GetColor());
			nodeEndParticle.SetPosition(childParticle.GetPosition());

			nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
		}
		return;
	}
	m_needPacking = true;
	const auto& mainChildPhysics2D = mainChildNode->m_frontParticlePhysics2D;
	for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
	{
		const auto nodeStartParticleHandle = m_frontParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
		const auto nodeEndParticleHandle = m_backParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
		auto& nodeStartParticle = m_frontParticlePhysics2D.RefParticle(nodeStartParticleHandle);
		auto& nodeEndParticle = m_backParticlePhysics2D.RefParticle(nodeEndParticleHandle);
		nodeStartParticle.SetColor(mainChildParticle.GetColor());
		nodeStartParticle.SetPosition(mainChildParticle.GetPosition());
		nodeEndParticle.SetColor(mainChildParticle.GetColor());
		nodeEndParticle.SetPosition(mainChildParticle.GetPosition());

		nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
	}
	const auto nodeGlobalTransform = scene->GetDataComponent<GlobalTransform>(owner);
	bool needSimulation = false;
	for (const auto& childNode : childrenNodes)
	{
		if (childNode == mainChildNode) continue;
		auto& childPhysics2D = childNode->m_frontParticlePhysics2D;
		if (!childNode->m_split)
		{
			needSimulation = true;
			auto childNodeGlobalTransform = scene->GetDataComponent<GlobalTransform>(childNode->GetOwner());
			const auto childNodeFront = glm::inverse(nodeGlobalTransform.GetRotation()) * childNodeGlobalTransform.GetRotation() * glm::vec3(0, 0, -1);
			auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
			if (glm::isnan(direction.x) || glm::isnan(direction.y))
			{
				direction = glm::vec2(1, 0);
			}
			childNode->m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-direction);
			const auto mainChildRadius = mainChildPhysics2D.GetDistanceToCenter(direction);
			auto offset = glm::vec2(0.0f);
			offset = (mainChildRadius - childNode->m_centerDirectionRadius + 2.0f) * direction;
			childNode->m_offset = offset;
			for (auto& childParticle : childPhysics2D.RefParticles())
			{
				childParticle.SetPosition(childParticle.GetPosition() + offset);

				const auto nodeStartParticleHandle = m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
				auto& nodeStartParticle = m_frontParticlePhysics2D.RefParticle(nodeStartParticleHandle);
				auto& nodeEndParticle = m_backParticlePhysics2D.RefParticle(nodeEndParticleHandle);
				nodeStartParticle.SetColor(childParticle.GetColor());
				nodeStartParticle.SetPosition(childParticle.GetPosition());
				nodeEndParticle.SetColor(childParticle.GetColor());
				nodeEndParticle.SetPosition(childParticle.GetPosition());
				nodeStartParticle.m_enable = true;
				nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
			}
		}
		else
		{
			for (auto& childParticle : childPhysics2D.RefParticles())
			{
				const auto nodeStartParticleHandle = m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
				auto& nodeStartParticle = m_frontParticlePhysics2D.RefParticle(nodeStartParticleHandle);
				auto& nodeEndParticle = m_backParticlePhysics2D.RefParticle(nodeEndParticleHandle);
				nodeStartParticle.SetColor(childParticle.GetColor());
				nodeStartParticle.SetPosition(glm::vec3(0.0f));
				nodeEndParticle.SetColor(childParticle.GetColor());
				nodeEndParticle.SetPosition(glm::vec3(0.0f));
				nodeStartParticle.m_enable = false;
				nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
			}
		}
	}
	if(needSimulation) PackTask(pipeModelParameters, false);
	for (const auto& childNode : childrenNodes)
	{
		if (childNode == mainChildNode) continue;
		if (childNode->m_split)
		{
			auto& childPhysics2D = childNode->m_frontParticlePhysics2D;
			auto childNodeGlobalTransform = scene->GetDataComponent<GlobalTransform>(childNode->GetOwner());
			const auto childNodeFront = glm::inverse(nodeGlobalTransform.GetRotation()) * childNodeGlobalTransform.GetRotation() * glm::vec3(0, 0, -1);
			auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
			if (glm::isnan(direction.x) || glm::isnan(direction.y))
			{
				direction = glm::vec2(1, 0);
			}
			//TODO: Change this to avoid calculating disabled particles!
			childNode->m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-direction);
			const auto centerRadius = m_frontParticlePhysics2D.GetDistanceToCenter(direction);
			auto offset = glm::vec2(0.0f);
			offset = (centerRadius + childNode->m_centerDirectionRadius + 2.0f) * direction;
			childNode->m_offset = offset;
			for (auto& childParticle : childPhysics2D.RefParticles())
			{
				childParticle.SetPosition(childParticle.GetPosition() + offset);

				const auto nodeStartParticleHandle = m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
				auto& nodeStartParticle = m_frontParticlePhysics2D.RefParticle(nodeStartParticleHandle);
				auto& nodeEndParticle = m_backParticlePhysics2D.RefParticle(nodeEndParticleHandle);
				nodeStartParticle.SetColor(childParticle.GetColor());
				nodeStartParticle.SetPosition(childParticle.GetPosition());
				nodeEndParticle.SetColor(childParticle.GetColor());
				nodeEndParticle.SetPosition(childParticle.GetPosition());

				nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
			}
		}
	}
	CopyFrontToBackTask();
	m_frontParticlePhysics2D.EnableAllParticles();
	/*
	for (const auto& childNode : childrenNodes)
	{
		if (childNode == mainChildNode) continue;
		auto& childPhysics2D = childNode->m_frontParticlePhysics2D;
		auto childNodeGlobalTransform = scene->GetDataComponent<GlobalTransform>(childNode->GetOwner());
		const auto childNodeFront = glm::inverse(nodeGlobalTransform.GetRotation()) * childNodeGlobalTransform.GetRotation() * glm::vec3(0, 0, -1);
		auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
		if (glm::isnan(direction.x) || glm::isnan(direction.y))
		{
			direction = glm::vec2(1, 0);
		}
		childNode->m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-direction);
		const auto mainChildRadius = mainChildPhysics2D.GetDistanceToCenter(direction);
		auto offset = glm::vec2(0.0f);
		if(childNode->m_split)
		{
			offset = (mainChildRadius + childNode->m_centerDirectionRadius + 2.0f) * direction;
		}else
		{
			offset = (mainChildRadius - childNode->m_centerDirectionRadius) * direction;
		}
		childNode->m_offset = offset;
		for (auto& childParticle : childPhysics2D.RefParticles())
		{
			childParticle.SetPosition(childParticle.GetPosition() + offset);

			const auto nodeStartParticleHandle = m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
			const auto nodeEndParticleHandle = m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
			auto& nodeStartParticle = m_frontParticlePhysics2D.RefParticle(nodeStartParticleHandle);
			auto& nodeEndParticle = m_backParticlePhysics2D.RefParticle(nodeEndParticleHandle);
			nodeStartParticle.SetColor(childParticle.GetColor());
			nodeStartParticle.SetPosition(childParticle.GetPosition());
			nodeEndParticle.SetColor(childParticle.GetColor());
			nodeEndParticle.SetPosition(childParticle.GetPosition());

			nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
		}
		index++;
	}*/

}

void TreePipeNode::CopyFrontToBackTask()
{
	for (int i = 0; i < m_frontParticlePhysics2D.RefParticles().size(); i++)
	{
		m_backParticlePhysics2D.RefParticle(i).SetPosition(m_frontParticlePhysics2D.RefParticle(i).GetPosition());
	}
}

void TreePipeNode::CalculateShiftTask(const PipeModelParameters& pipeModelParameters)
{
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto childrenEntities = scene->GetChildren(owner);
	std::vector<std::shared_ptr<TreePipeNode>> childrenNodes{};

	std::shared_ptr<TreePipeNode> mainChildNode{};
	for (auto childEntity : childrenEntities)
	{
		if (scene->HasPrivateComponent<TreePipeNode>(childEntity))
		{
			const auto childNode = scene->GetOrSetPrivateComponent<TreePipeNode>(childEntity).lock();
			if (childNode->m_apical) mainChildNode = childNode;
			childrenNodes.push_back(childNode);
		}
	}
	if (childrenNodes.size() <= 1) return;
	if (mainChildNode)
	{
		const auto& mainChildPhysics2D = mainChildNode->m_frontParticlePhysics2D;
		auto sum = glm::vec2(0.0f);
		for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
		{
			const auto nodeStartParticleHandle = m_frontParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
			auto& nodeStartParticle = m_frontParticlePhysics2D.RefParticle(nodeStartParticleHandle);
			sum += nodeStartParticle.GetPosition();
		}
		m_shift = sum / static_cast<float>(mainChildPhysics2D.PeekParticles().size());
	}
}

void TreePipeNode::InsertInterpolation(const float a)
{
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto parent = scene->GetParent(owner);
	const Entity middleEntity = scene->CreateEntity("Profile");
	const auto tpn = scene->GetOrSetPrivateComponent<TreePipeNode>(middleEntity).lock();

	tpn->m_apical = m_apical;
	const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(middleEntity).lock();
	mmr->m_mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
	mmr->m_material = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock()->m_material;


	for (const auto& [pipeHandle, prevParticleHandle] : m_frontParticleMap)
	{
		const auto newFrontParticleHandle = tpn->m_frontParticlePhysics2D.AllocateParticle();
		auto& newFrontParticle = tpn->m_frontParticlePhysics2D.RefParticle(newFrontParticleHandle);
		tpn->m_frontParticleMap.insert({ pipeHandle, newFrontParticleHandle });

		newFrontParticle.m_data.m_pipeHandle = pipeHandle;
		const auto& prevParticle = m_frontParticlePhysics2D.PeekParticle(prevParticleHandle);
		const auto& nextParticle = m_backParticlePhysics2D.PeekParticle(m_backParticleMap.at(pipeHandle));
		newFrontParticle.SetPosition(glm::mix(prevParticle.GetPosition(), nextParticle.GetPosition(), a));
		newFrontParticle.SetColor(glm::mix(prevParticle.GetColor(), nextParticle.GetColor(), a));
		newFrontParticle.SetDamping(glm::mix(prevParticle.GetDamping(), nextParticle.GetDamping(), a));

		const auto newBackParticleHandle = tpn->m_backParticlePhysics2D.AllocateParticle();
		auto& newBackParticle = tpn->m_backParticlePhysics2D.RefParticle(newBackParticleHandle);
		tpn->m_backParticleMap.insert({ pipeHandle, newBackParticleHandle });

		newBackParticle.m_data.m_pipeHandle = pipeHandle;
		newBackParticle.SetPosition(glm::mix(prevParticle.GetPosition(), nextParticle.GetPosition(), a));
		newBackParticle.SetColor(glm::mix(prevParticle.GetColor(), nextParticle.GetColor(), a));
		newBackParticle.SetDamping(glm::mix(prevParticle.GetDamping(), nextParticle.GetDamping(), a));
	}

	const auto ownerGlobalTransform = scene->GetDataComponent<GlobalTransform>(owner);
	auto parentGlobalTransform = scene->GetDataComponent<GlobalTransform>(parent);

	GlobalTransform globalTransform;
	globalTransform.SetValue(glm::mix(ownerGlobalTransform.GetPosition(), parentGlobalTransform.GetPosition(), a),
		glm::mix(ownerGlobalTransform.GetEulerRotation(), scene->HasPrivateComponent<TreePipeNode>(parent) ? parentGlobalTransform.GetEulerRotation() : ownerGlobalTransform.GetEulerRotation(), a),
		glm::vec3(0.02f));

	scene->SetParent(middleEntity, parent);
	scene->SetParent(owner, middleEntity);
	scene->SetDataComponent(middleEntity, globalTransform);
	scene->SetDataComponent(owner, ownerGlobalTransform);

}

void TreePipeNode::OnCreate()
{
	m_needPacking = false;
	m_frontParticlePhysics2D = {};
	m_frontParticleMap = {};
	m_backParticlePhysics2D = {};
	m_backParticleMap = {};

	m_frontControlPointDistance = 0.0f;
	m_backControlPointDistance = 0.0f;
	m_centerDirectionRadius = 0.0f;
	m_apical = false;
}

void TreePipeNode::OnDestroy()
{
	Wait();
	m_needPacking = false;
	m_frontParticlePhysics2D = {};
	m_frontParticleMap = {};
	m_backParticlePhysics2D = {};
	m_backParticleMap = {};

	m_frontControlPointDistance = 0.0f;
	m_backControlPointDistance = 0.0f;
	m_centerDirectionRadius = 0.0f;
	m_apical = false;
}

void TreePipeNode::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static bool displayPhysics = true;
	static bool showGrid = false;
	ImGui::Checkbox("Split", &m_split);

	ImGui::DragFloat2("Offset", &m_offset.x);
	ImGui::DragFloat2("Shift", &m_shift.x);
	ImGui::Checkbox("Show Physics", &displayPhysics);
	if (displayPhysics) ImGui::Checkbox("Show Grid", &showGrid);
	if (displayPhysics)
	{
		static float a = 0.0f;
		ImGui::DragFloat("Preset A", &a, 0.01f, 0.01f, 0.99f);
		if (ImGui::Button("Insert a"))
		{
			InsertInterpolation(a);
		}
		ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
		const std::string frontTag = "Front Profile";
		if (ImGui::Begin(frontTag.c_str()))
		{
			m_frontParticlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
		const std::string backTag = "Back Profile";
		if (ImGui::Begin(backTag.c_str()))
		{
			m_backParticlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
	}
}

void TreePipeNode::CalculateProfile(const PipeModelParameters& pipeModelParameters, const bool scheduling)
{
	if (scheduling) {
		m_tasks.emplace_back(Jobs::AddTask([&](unsigned threadIndex) {
			MergeTask(pipeModelParameters);
			if (!m_needPacking) return;
			m_needPacking = false;
			if (m_frontParticlePhysics2D.PeekParticles().size() <= 1) return;
			PackTask(pipeModelParameters, !scheduling);
			if (GetScene()->GetChildren(GetOwner()).empty()) CopyFrontToBackTask();
			CalculateShiftTask(pipeModelParameters);
			}
		)
		);
	}
	else
	{
		MergeTask(pipeModelParameters);
		PackTask(pipeModelParameters, !scheduling);
		if (GetScene()->GetChildren(GetOwner()).empty()) CopyFrontToBackTask();
		CalculateShiftTask(pipeModelParameters);
	}
}


void TreePipeNode::Wait()
{
	if (m_tasks.empty()) return;
	for (const auto& i : m_tasks)
	{
		i.wait();
	}
	m_tasks.clear();
}
