#include "BillboardCloud.hpp"
using namespace EvoEngine;

typedef int VIndex;
struct ConnectivityGraph
{
	struct V
	{
		std::vector<std::pair<unsigned, float>> m_connectedVertices;
		VIndex m_index = -1;
	};

	std::vector<V> m_vs;

	void EstablishConnectivityGraph(const BillboardCloud::Element& element);
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
	ConnectivityGraph connectivityGraph;
	connectivityGraph.EstablishConnectivityGraph(*this);
	std::vector<std::vector<unsigned>> retVal;
	std::vector<float> dist;
	std::vector<bool> visited;
	visited.resize(m_vertices.size());
	dist.resize(m_vertices.size());
	std::fill(dist.begin(), dist.end(), FLT_MAX);
	std::fill(visited.begin(), visited.end(), false);
	std::vector<VIndex> q;
	for (const auto& v : connectivityGraph.m_vs)
	{
		if (!v.m_connectedVertices.empty())
		{
			q.emplace_back(v.m_index);
		}
	}
	bool distUpdated = true;
	while (distUpdated && !q.empty()) {
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

		
		dist[seedVertexIndex] = 0;
		updated[seedVertexIndex] = true;
		while (!q.empty())
		{
			float minDistance = FLT_MAX;
			int minDistanceIndex = -1;
			for (int qi = 0; qi < q.size(); qi++)
			{
				const VIndex testIndex = q[qi];
				const auto testDist = dist[testIndex];
				if (testDist < minDistance)
				{
					minDistanceIndex = qi;
					minDistance = testDist;
				}
			}
			if (minDistanceIndex == -1) break;
			const VIndex u = q[minDistanceIndex];
			q[minDistanceIndex] = q.back();
			q.pop_back();
			for (const auto& neighbor : connectivityGraph.m_vs[u].m_connectedVertices)
			{
				if(visited[neighbor.first]) continue;
				const float newDist = dist[u] + neighbor.second;
				if (dist[neighbor.first] > newDist)
				{
					dist[neighbor.first] = newDist;
					distUpdated = true;
					updated[neighbor.first] = true;
				}
			}
			visited[u] = true;
		}


		float maxDistance = 0.f;
		std::vector<VIndex> currentGroup;
		for(VIndex vertexIndex = 0; vertexIndex < m_vertices.size(); vertexIndex++)
		{
			if(updated[vertexIndex])
			{
				maxDistance = glm::max(maxDistance, dist[vertexIndex]);
				currentGroup.emplace_back(vertexIndex);
			}
		}
		for(const auto& vertexIndex : currentGroup)
		{
			m_vertices[vertexIndex].m_color = glm::vec4(glm::vec3(dist[vertexIndex] / maxDistance), 1.f);
		}
	}

	for(VIndex vertexIndex = 0; vertexIndex < m_vertices.size(); vertexIndex++)
	{
		const auto currentDist = dist[vertexIndex];
		if(currentDist == 0.f) m_vertices[vertexIndex].m_color = glm::vec4(0.f, 0.f, 1.f, 1.f);
		else
		{
			bool localMaximum = true;
			for(auto& connectivity : connectivityGraph.m_vs[vertexIndex].m_connectedVertices)
			{
				if(currentDist < dist[connectivity.first]) localMaximum = false;
			}
			if(localMaximum)
			{
				m_vertices[vertexIndex].m_color = glm::vec4(1.f, 0.f, 0.f, 1.f);
			}
		}
	}

	return retVal;
}
