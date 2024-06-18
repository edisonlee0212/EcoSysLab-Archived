#include "BillboardCloud.hpp"
using namespace evo_engine;

typedef int VIndex;
typedef int TIndex;
typedef int CIndex;
struct ConnectivityGraph {
  struct V {
    enum class VType { Default, LocalExtremum, SaddlePoint };
    VType m_type = VType::Default;
    std::vector<std::pair<VIndex, float>> m_connectedVertices;
    VIndex m_vertexIndex = -1;
    float m_ind = -1;
    std::vector<TIndex> m_connectedTriangles;
    int m_contourIndex = -1;
    float m_distanceToSourcePoint = FLT_MAX;
    VIndex m_sourceIndex = -1;
    VIndex m_prevIndex = -1;
    int m_connectedComponentId = -1;
  };

  std::vector<V> m_vs;

  void EstablishConnectivityGraph(const BillboardCloud::Element& element);

  struct ConnectedComponent {
    std::vector<VIndex> m_vs;
    std::unordered_set<VIndex> verticesSet;
    VIndex m_sourceIndex = -1;
    CIndex m_index = -1;

    std::unordered_map<VIndex, int> m_groups;
  };
  std::vector<ConnectedComponent> m_connectedComponents;
  std::vector<std::vector<VIndex>> m_contourTable;
};

void ConnectivityGraph::EstablishConnectivityGraph(const BillboardCloud::Element& element) {
  m_vs.clear();
  m_vs.resize(element.vertices.size());
  for (int vi = 0; vi < m_vs.size(); vi++) {
    m_vs[vi] = {};
    m_vs[vi].m_vertexIndex = vi;
    m_vs[vi].m_distanceToSourcePoint = FLT_MAX;
  }
  for (TIndex triangleIndex = 0; triangleIndex < element.triangles.size(); triangleIndex++) {
    const auto& triangle = element.triangles.at(triangleIndex);

    const auto& vertex0 = element.vertices[triangle.x];
    const auto& vertex1 = element.vertices[triangle.y];
    const auto& vertex2 = element.vertices[triangle.z];

    auto& v0 = m_vs[triangle.x];
    auto& v1 = m_vs[triangle.y];
    auto& v2 = m_vs[triangle.z];

    v0.m_connectedTriangles.emplace_back(triangleIndex);
    v1.m_connectedTriangles.emplace_back(triangleIndex);
    v2.m_connectedTriangles.emplace_back(triangleIndex);

    {
      bool findY = false;
      bool findZ = false;
      for (const auto& connectedIndex : v0.m_connectedVertices) {
        if (connectedIndex.first == triangle.y)
          findY = true;
        if (connectedIndex.first == triangle.z)
          findZ = true;
      }
      if (!findY)
        v0.m_connectedVertices.emplace_back(triangle.y, glm::distance(vertex0.position, vertex1.position));
      if (!findZ)
        v0.m_connectedVertices.emplace_back(triangle.z, glm::distance(vertex0.position, vertex2.position));
    }
    {
      bool findX = false;
      bool findZ = false;
      for (const auto& connectedIndex : v1.m_connectedVertices) {
        if (connectedIndex.first == triangle.x)
          findX = true;
        if (connectedIndex.first == triangle.z)
          findZ = true;
      }
      if (!findX)
        v1.m_connectedVertices.emplace_back(triangle.x, glm::distance(vertex1.position, vertex0.position));
      if (!findZ)
        v1.m_connectedVertices.emplace_back(triangle.z, glm::distance(vertex1.position, vertex2.position));
    }
    {
      bool findX = false;
      bool findY = false;
      for (const auto& connectedIndex : v2.m_connectedVertices) {
        if (connectedIndex.first == triangle.x)
          findX = true;
        if (connectedIndex.first == triangle.y)
          findY = true;
      }
      if (!findX)
        v2.m_connectedVertices.emplace_back(triangle.x, glm::distance(vertex2.position, vertex0.position));
      if (!findY)
        v2.m_connectedVertices.emplace_back(triangle.y, glm::distance(vertex2.position, vertex1.position));
    }
  }
}

std::vector<std::vector<unsigned>> BillboardCloud::Element::CalculateLevelSets(const glm::vec3& direction) {
  ConnectivityGraph connectivityGraph{};
  connectivityGraph.EstablishConnectivityGraph(*this);
  std::vector<std::vector<unsigned>> retVal;
  struct VNode {
    VIndex m_vertexIndex = -1;
    float m_distance = 0.f;
  };
  struct VNodeComparator {
    bool operator()(const VNode& left, const VNode& right) const {
      return left.m_distance > right.m_distance;
    }
  };
  std::vector<bool> visited;
  visited.resize(vertices.size());
  std::fill(visited.begin(), visited.end(), false);

  bool distUpdated = true;
  std::vector<bool> updated;
  updated.resize(vertices.size());
  while (distUpdated) {
    distUpdated = false;
    std::fill(updated.begin(), updated.end(), false);

    VIndex seedVertexIndex = -1;
    float minHeight = FLT_MAX;
    for (const auto& triangle : triangles) {
      const auto& vertex0 = vertices[triangle.x];
      const auto& vertex1 = vertices[triangle.y];
      const auto& vertex2 = vertices[triangle.z];

      if (!connectivityGraph.m_vs[triangle.x].m_connectedVertices.empty() && !visited[triangle.x] &&
          glm::dot(direction, vertex0.position) < minHeight) {
        seedVertexIndex = triangle.x;
        minHeight = vertex0.position.y;
      }
      if (!connectivityGraph.m_vs[triangle.y].m_connectedVertices.empty() && !visited[triangle.y] &&
          glm::dot(direction, vertex1.position) < minHeight) {
        seedVertexIndex = triangle.y;
        minHeight = vertex1.position.y;
      }
      if (!connectivityGraph.m_vs[triangle.z].m_connectedVertices.empty() && !visited[triangle.z] &&
          glm::dot(direction, vertex2.position) < minHeight) {
        seedVertexIndex = triangle.z;
        minHeight = vertex2.position.y;
      }
    }
    if (seedVertexIndex == -1)
      break;
    connectivityGraph.m_vs[seedVertexIndex].m_distanceToSourcePoint = 0;
    int numberOfContours = 1;
    connectivityGraph.m_vs[seedVertexIndex].m_contourIndex = numberOfContours - 1;

    updated[seedVertexIndex] = true;

    std::priority_queue<VNode, std::vector<VNode>, VNodeComparator> q;
    q.push({seedVertexIndex, 0});

    while (!q.empty()) {
      const auto node = q.top();
      if (node.m_distance == FLT_MAX)
        break;
      const VIndex u = node.m_vertexIndex;
      q.pop();
      if (visited[node.m_vertexIndex])
        continue;
      auto& vu = connectivityGraph.m_vs[u];
      for (const auto& neighbor : vu.m_connectedVertices) {
        const float newDist = vu.m_distanceToSourcePoint + neighbor.second;
        auto& neighborV = connectivityGraph.m_vs[neighbor.first];
        if (neighborV.m_distanceToSourcePoint > newDist) {
          neighborV.m_prevIndex = u;
          neighborV.m_distanceToSourcePoint = newDist;
          neighborV.m_contourIndex = vu.m_contourIndex;
          q.push({neighbor.first, newDist});
          distUpdated = true;
          updated[neighbor.first] = true;
        }
      }
      visited[u] = true;

      int signChangeCount = 0;
      for (const auto& triangleIndex : vu.m_connectedTriangles) {
        const auto& triangle = triangles.at(triangleIndex);
        const auto distU = vu.m_distanceToSourcePoint;
        if (static_cast<int>(triangle.x) == u) {
          const auto& distY = connectivityGraph.m_vs[triangle.y].m_distanceToSourcePoint;
          const auto& distZ = connectivityGraph.m_vs[triangle.z].m_distanceToSourcePoint;
          if ((distY > distU && distZ < distU) || (distY < distU && distZ > distU)) {
            signChangeCount++;
          }
        } else if (static_cast<int>(triangle.y) == u) {
          const auto& distX = connectivityGraph.m_vs[triangle.x].m_distanceToSourcePoint;
          const auto& distZ = connectivityGraph.m_vs[triangle.z].m_distanceToSourcePoint;
          if ((distX > distU && distZ < distU) || (distX < distU && distZ > distU)) {
            signChangeCount++;
          }
        } else if (static_cast<int>(triangle.z) == u) {
          const auto& distX = connectivityGraph.m_vs[triangle.x].m_distanceToSourcePoint;
          const auto& distY = connectivityGraph.m_vs[triangle.y].m_distanceToSourcePoint;
          if ((distX > distU && distY < distU) || (distX < distU && distY > distU)) {
            signChangeCount++;
          }
        }
      }
      vu.m_ind = 1.f - static_cast<float>(signChangeCount) / 2.f;
      if (vu.m_ind == 1.f) {
        vu.m_type = ConnectivityGraph::V::VType::LocalExtremum;

      } else if (vu.m_ind < 0) {
        vu.m_type = ConnectivityGraph::V::VType::SaddlePoint;

        // Split into sub-contours
        std::vector<VIndex> unprocessedNeighbors;
        unprocessedNeighbors.resize(vu.m_connectedVertices.size());
        for (int neighborIndex = 0; neighborIndex < vu.m_connectedVertices.size(); neighborIndex++) {
          unprocessedNeighbors.at(neighborIndex) = vu.m_connectedVertices[neighborIndex].first;
        }
        std::vector<std::pair<bool, std::vector<VIndex>>> groups;
        while (!unprocessedNeighbors.empty()) {
          // Create a new group
          groups.emplace_back();
          auto& group = groups.back();
          auto nextNeighbor = unprocessedNeighbors.back();

          std::vector<VIndex> waitList;
          waitList.emplace_back(nextNeighbor);
          // Assign sign to current group
          group.first = connectivityGraph.m_vs[nextNeighbor].m_distanceToSourcePoint > vu.m_distanceToSourcePoint;
          while (!waitList.empty()) {
            auto walker = waitList.back();
            waitList.pop_back();
            // Add current walker into group and remove it from unprocessed list
            group.second.emplace_back(walker);
            for (int unprocessedVertexIndex = 0; unprocessedVertexIndex < unprocessedNeighbors.size();
                 unprocessedVertexIndex++) {
              if (unprocessedNeighbors[unprocessedVertexIndex] == walker) {
                unprocessedNeighbors[unprocessedVertexIndex] = unprocessedNeighbors.back();
                unprocessedNeighbors.pop_back();
                break;
              }
            }
            // Try to find another adjacent vertex
            for (const auto& triangleIndex : vu.m_connectedTriangles) {
              const auto& triangle = triangles.at(triangleIndex);
              for (int v0 = 0; v0 < 3; v0++) {
                if (static_cast<int>(triangle[v0]) == u) {
                  if (static_cast<int>(triangle[(v0 + 1) % 3]) == walker) {
                    const int target = static_cast<int>(triangle[(v0 + 2) % 3]);
                    // If target is not processed
                    bool unprocessed = false;
                    for (const auto& unprocessedIndex : unprocessedNeighbors) {
                      if (unprocessedIndex == target) {
                        unprocessed = true;
                      }
                    }
                    if (unprocessed) {
                      // And it has the same sign as current group...
                      if (group.first &&
                          connectivityGraph.m_vs[target].m_distanceToSourcePoint > vu.m_distanceToSourcePoint) {
                        waitList.emplace_back(target);
                      } else if (!group.first &&
                                 connectivityGraph.m_vs[target].m_distanceToSourcePoint < vu.m_distanceToSourcePoint) {
                        waitList.emplace_back(target);
                      }
                    }
                  } else if (static_cast<int>(triangle[(v0 + 2) % 3]) == walker) {
                    const int target = static_cast<int>(triangle[(v0 + 1) % 3]);
                    // If target is not processed
                    bool unprocessed = false;
                    for (const auto& unprocessedIndex : unprocessedNeighbors) {
                      if (unprocessedIndex == target) {
                        unprocessed = true;
                      }
                    }
                    if (unprocessed) {
                      // And it has the same sign as current group...
                      if (group.first &&
                          connectivityGraph.m_vs[target].m_distanceToSourcePoint > vu.m_distanceToSourcePoint) {
                        waitList.emplace_back(target);
                      } else if (!group.first &&
                                 connectivityGraph.m_vs[target].m_distanceToSourcePoint < vu.m_distanceToSourcePoint) {
                        waitList.emplace_back(target);
                      }
                    }
                  }
                }
              }
            }
          }
        }

        for (const auto& group : groups) {
          if (group.first) {
            for (const auto& vertexIndex : group.second) {
              connectivityGraph.m_vs[vertexIndex].m_contourIndex = numberOfContours;
            }
            numberOfContours++;
          }
        }
      } else {
        vu.m_type = ConnectivityGraph::V::VType::Default;
      }
    }
    float maxDistance = 0.f;
    // Establish connected component for current group.

    ConnectivityGraph::ConnectedComponent component{};
    component.m_index = connectivityGraph.m_connectedComponents.size();
    component.m_sourceIndex = seedVertexIndex;
    for (VIndex vertexIndex = 0; vertexIndex < vertices.size(); vertexIndex++) {
      if (updated[vertexIndex]) {
        maxDistance = glm::max(maxDistance, connectivityGraph.m_vs[vertexIndex].m_distanceToSourcePoint);
        component.m_vs.emplace_back(vertexIndex);
        component.verticesSet.insert(vertexIndex);
      }
    }

    std::vector<glm::vec3> contourColors;
    contourColors.resize(numberOfContours);
    for (auto& contourColor : contourColors) {
      contourColor = glm::abs(glm::sphericalRand(1.f));
    }
    for (const auto& vertexIndex : component.m_vs) {
      auto& v = connectivityGraph.m_vs[vertexIndex];
      v.m_sourceIndex = seedVertexIndex;
      v.m_connectedComponentId = component.m_index;

      switch (connectivityGraph.m_vs[vertexIndex].m_type) {
        case ConnectivityGraph::V::VType::Default:
          vertices[vertexIndex].color = glm::vec4(1.f);
          break;
        case ConnectivityGraph::V::VType::LocalExtremum:
          vertices[vertexIndex].color = glm::vec4(1.f, 0.f, 0.f, 1.f);
          break;
        case ConnectivityGraph::V::VType::SaddlePoint:
          vertices[vertexIndex].color = glm::vec4(0.f, 0.f, 1.f, 1.f);
          break;
      }
      vertices[vertexIndex].color =
          glm::vec4(glm::vec3(glm::mod(v.m_distanceToSourcePoint / maxDistance * 20.f, 1.f)), 1.f);
      // vertices[vertexIndex].color = glm::vec4(contourColors[v.m_contourIndex], 1.f);
    }
    if (!component.m_vs.empty())
      connectivityGraph.m_connectedComponents.emplace_back(std::move(component));
  }
  EVOENGINE_LOG("Distance calculation finished!");

  return retVal;
}
