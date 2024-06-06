#include "BillboardCloud.hpp"
using namespace EvoEngine;

typedef int VIndex;
struct ConnectivityGraph
{
	struct V
	{
		std::vector<std::pair<VIndex, float>> m_connectedVertices;
		VIndex m_index = -1;
	};

	std::vector<V> m_vs;

	void EstablishConnectivityGraph(const BillboardCloud::Element& element);



	struct ConnectedComponent
	{
		struct LocalMaximum
		{
			VIndex m_vertexIndex = -1;
			std::vector<VIndex> m_path;
			std::unordered_set<VIndex> m_pathSet;
		};
		std::vector<VIndex> m_vs;
		std::unordered_map<VIndex, int> m_paths;
		//Vertices in current component grouped by paths.
		std::unordered_map<VIndex, int> m_groups;
		std::vector<LocalMaximum> m_localMaximums;
		VIndex m_localMinimum = -1;
		std::vector<glm::vec3> m_pathColors;
		std::unordered_set<VIndex> m_verticesSet;
	};
	std::vector<ConnectedComponent> m_connectedComponents;

};

void ConnectivityGraph::EstablishConnectivityGraph(const BillboardCloud::Element& element)
{
	m_vs.clear();
	m_vs.resize(element.m_vertices.size());
	for (int vi = 0; vi < m_vs.size(); vi++) m_vs[vi].m_index = vi;

	for (const auto& triangle : element.m_triangles)
	{
		const auto& vertex0 = element.m_vertices[triangle.x];
		const auto& vertex1 = element.m_vertices[triangle.y];
		const auto& vertex2 = element.m_vertices[triangle.z];

		auto& v0 = m_vs[triangle.x];
		auto& v1 = m_vs[triangle.y];
		auto& v2 = m_vs[triangle.z];
		{
			bool findY = false;
			bool findZ = false;
			for (const auto& connectedIndex : v0.m_connectedVertices)
			{
				if (connectedIndex.first == triangle.y) findY = true;
				if (connectedIndex.first == triangle.z) findZ = true;
			}
			if (!findY) v0.m_connectedVertices.emplace_back(triangle.y, glm::distance(vertex0.m_position, vertex1.m_position));
			if (!findZ) v0.m_connectedVertices.emplace_back(triangle.z, glm::distance(vertex0.m_position, vertex2.m_position));
		}
		{
			bool findX = false;
			bool findZ = false;
			for (const auto& connectedIndex : v1.m_connectedVertices)
			{
				if (connectedIndex.first == triangle.x) findX = true;
				if (connectedIndex.first == triangle.z) findZ = true;
			}
			if (!findX) v1.m_connectedVertices.emplace_back(triangle.x, glm::distance(vertex1.m_position, vertex0.m_position));
			if (!findZ) v1.m_connectedVertices.emplace_back(triangle.z, glm::distance(vertex1.m_position, vertex2.m_position));
		}
		{
			bool findX = false;
			bool findY = false;
			for (const auto& connectedIndex : v2.m_connectedVertices)
			{
				if (connectedIndex.first == triangle.x) findX = true;
				if (connectedIndex.first == triangle.y) findY = true;
			}
			if (!findX) v2.m_connectedVertices.emplace_back(triangle.x, glm::distance(vertex2.m_position, vertex0.m_position));
			if (!findY) v2.m_connectedVertices.emplace_back(triangle.y, glm::distance(vertex2.m_position, vertex1.m_position));
		}
	}
}


std::vector<std::vector<unsigned>> BillboardCloud::Element::CalculateLevelSets(const glm::vec3& direction)
{
	ConnectivityGraph connectivityGraph{};
	connectivityGraph.EstablishConnectivityGraph(*this);
	std::vector<std::vector<unsigned>> retVal;
	struct VNode
	{
		VIndex m_vertexIndex = -1;
		float m_distance = 0.f;
	};
	struct VNodeComparator
	{
		bool operator()(const VNode& left, const VNode& right) const
		{
			return left.m_distance > right.m_distance;
		}
	};

	{
		std::vector<float> dist;
		std::vector<VIndex> prev;
		std::vector<bool> visited;
		dist.resize(m_vertices.size());
		prev.resize(m_vertices.size());
		visited.resize(m_vertices.size());
		std::fill(prev.begin(), prev.end(), -1);
		std::fill(dist.begin(), dist.end(), FLT_MAX);
		std::fill(visited.begin(), visited.end(), false);
		
		bool distUpdated = true;
		std::vector<std::vector<VIndex>> components;
		while (distUpdated) {
			distUpdated = false;
			std::vector<bool> updated;
			updated.resize(dist.size());
			std::fill(updated.begin(), updated.end(), false);
			
			VIndex seedVertexIndex = -1;
			float minHeight = FLT_MAX;
			for (const auto& triangle : m_triangles)
			{
				const auto& vertex0 = m_vertices[triangle.x];
				const auto& vertex1 = m_vertices[triangle.y];
				const auto& vertex2 = m_vertices[triangle.z];

				if (!connectivityGraph.m_vs[triangle.x].m_connectedVertices.empty()
					&& !visited[triangle.x]
					&& glm::dot(direction, vertex0.m_position) < minHeight)
				{
					seedVertexIndex = triangle.x;
					minHeight = vertex0.m_position.y;
				}
				if (!connectivityGraph.m_vs[triangle.y].m_connectedVertices.empty()
					&& !visited[triangle.y]
					&& glm::dot(direction, vertex1.m_position) < minHeight)
				{
					seedVertexIndex = triangle.y;
					minHeight = vertex1.m_position.y;
				}
				if (!connectivityGraph.m_vs[triangle.z].m_connectedVertices.empty()
					&& !visited[triangle.z]
					&& glm::dot(direction, vertex2.m_position) < minHeight)
				{
					seedVertexIndex = triangle.z;
					minHeight = vertex2.m_position.y;
				}
			}
			if(seedVertexIndex == -1) break;
			dist[seedVertexIndex] = 0;
			updated[seedVertexIndex] = true;

			std::priority_queue<VNode, std::vector<VNode>, VNodeComparator> q;
			q.push({seedVertexIndex, 0});
			/*
			for (const auto& v : connectivityGraph.m_vs)
			{
				if (!visited[v.m_index] && !v.m_connectedVertices.empty())
				{
					q.push({v.m_index, FLT_MAX});
				}
			}
			*/
			while (!q.empty())
			{
				const auto node = q.top();
				if(node.m_distance == FLT_MAX) break;
				const VIndex u = node.m_vertexIndex;
				q.pop();
				if(visited[node.m_vertexIndex]) continue;
				for (const auto& neighbor : connectivityGraph.m_vs[u].m_connectedVertices)
				{
					const float newDist = dist[u] + neighbor.second;
					if (dist[neighbor.first] > newDist)
					{
						prev[neighbor.first] = u;
						dist[neighbor.first] = newDist;
						q.push({neighbor.first, newDist});
						distUpdated = true;
						updated[neighbor.first] = true;
					}
				}
				visited[u] = true;
			}
			float maxDistance = 0.f;
			//Establish connected component for current group.
			components.emplace_back();
			auto& component = components.back();
			for (VIndex vertexIndex = 0; vertexIndex < m_vertices.size(); vertexIndex++)
			{
				if (updated[vertexIndex])
				{
					maxDistance = glm::max(maxDistance, dist[vertexIndex]);
					component.emplace_back(vertexIndex);
				}
			}
		}
		EVOENGINE_LOG("Distance calculation finished!");
		for (auto& component : components)
		{
			ConnectivityGraph::ConnectedComponent currentConnectedComponent{};
			for (const auto& vertexIndex : component)
			{
				currentConnectedComponent.m_vs.emplace_back(vertexIndex);
				currentConnectedComponent.m_verticesSet.insert(vertexIndex);
				//Find local maximum and local minimum.
				const auto currentDist = dist[vertexIndex];
				if (currentDist == 0.f) {
					currentConnectedComponent.m_localMinimum = vertexIndex;
				}
				else //if(currentDist > maxDistance * 0.1f)
				{
					bool localMaximum = true;
					for (auto& connectivity : connectivityGraph.m_vs[vertexIndex].m_connectedVertices)
					{
						if (currentDist < dist[connectivity.first]) localMaximum = false;
					}
					if (localMaximum)
					{
						ConnectivityGraph::ConnectedComponent::LocalMaximum localMaximum{};
						localMaximum.m_vertexIndex = vertexIndex;
						currentConnectedComponent.m_localMaximums.emplace_back(std::move(localMaximum));
					}
				}
			}

			//Establish path between every local maximum and local minimum

			for (int pathIndex = 0; pathIndex < currentConnectedComponent.m_localMaximums.size(); pathIndex++)
			{
				auto& localMaximum = currentConnectedComponent.m_localMaximums.at(pathIndex);
				localMaximum.m_path.emplace_back(localMaximum.m_vertexIndex);
				localMaximum.m_pathSet.insert(localMaximum.m_vertexIndex);
				currentConnectedComponent.m_paths.insert({ localMaximum.m_vertexIndex, pathIndex });
				VIndex prevIndex = prev[localMaximum.m_vertexIndex];
				while (prevIndex != -1)
				{
					localMaximum.m_path.emplace_back(prevIndex);
					localMaximum.m_pathSet.insert(prevIndex);
					currentConnectedComponent.m_paths.insert({ prevIndex, pathIndex });
					prevIndex = prev[prevIndex];
				}
			}
			if (!currentConnectedComponent.m_vs.empty()) connectivityGraph.m_connectedComponents.emplace_back(std::move(currentConnectedComponent));
		}
		EVOENGINE_LOG("Path calculation finished!");
	}

	Jobs::RunParallelFor(connectivityGraph.m_connectedComponents.size(), [&](const unsigned connectedComponentIndex)
		{
			auto& connectedComponent = connectivityGraph.m_connectedComponents.at(connectedComponentIndex);
			connectedComponent.m_pathColors.resize(connectedComponent.m_localMaximums.size());
			for (auto& pathColor : connectedComponent.m_pathColors)
			{
				pathColor = glm::abs(glm::sphericalRand(1.f));
			}
			//Color path
			for (int pathIndex = 0; pathIndex < connectedComponent.m_localMaximums.size(); pathIndex++)
			{
				auto& localMaximum = connectedComponent.m_localMaximums.at(pathIndex);
				for (const auto& vertexIndex : localMaximum.m_path)
				{
					m_vertices.at(vertexIndex).m_color = glm::vec4(connectedComponent.m_pathColors.at(pathIndex), 1.f);
				}
			}
			//Color rest vertices;
			for (const auto& vertexIndex : connectedComponent.m_vs)
			{

				//If current vertex is on any of the path, skip.
				if (connectedComponent.m_paths.find(vertexIndex) != connectedComponent.m_paths.end()) continue;
				//Perform BFS search, find closest path.
				std::priority_queue<VNode, std::vector<VNode>, VNodeComparator> waitList;
				VNode base;
				base.m_vertexIndex = vertexIndex;
				base.m_distance = 0.f;
				waitList.push(base);
				std::vector<bool> nodeVisited;
				nodeVisited.resize(m_vertices.size());
				std::fill(nodeVisited.begin(), nodeVisited.end(), false);
				while (!waitList.empty())
				{
					auto node = waitList.top();
					waitList.pop();
					nodeVisited[node.m_vertexIndex] = true;
					const auto search = connectedComponent.m_paths.find(node.m_vertexIndex);
					if (search != connectedComponent.m_paths.end())
					{
						const auto pathIndex = search->second;
						m_vertices.at(vertexIndex).m_color = glm::vec4(connectedComponent.m_pathColors.at(pathIndex), 1.f);
						break;
					}

					for (const auto& neighbor : connectivityGraph.m_vs.at(node.m_vertexIndex).m_connectedVertices)
					{
						//If visited, skip.
						if (nodeVisited[neighbor.first]) continue;

						//If neighbor is not in current group, skip.
						if (connectedComponent.m_verticesSet.find(neighbor.first) == connectedComponent.m_verticesSet.end()) continue;

						//Add current neighbor to priority queue.
						VNode newNode;
						newNode.m_vertexIndex = neighbor.first;
						newNode.m_distance = node.m_distance + neighbor.second;
						waitList.push(newNode);
					}
				}
			}
		});

	EVOENGINE_LOG("Group coloring finished!");
	return retVal;
}
