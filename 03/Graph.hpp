#pragma once
#include <cmath>
#include <vector>

struct Edge {
    int to;
    double cost;
};
using Graph = std::vector<std::vector<Edge>>;

inline bool ValidNode(const Graph& graph, int id) {
    return id >= 0 && id < static_cast<int>(graph.size());
}

inline bool ValidGraph(const Graph& graph) {
    for (const auto& neighbors : graph) {
        for (const Edge& edge : neighbors) {
            if (!ValidNode(graph, edge.to) ||
                !std::isfinite(edge.cost) || edge.cost < 0.0) {
                return false;
            }
        }
    }
    return true;
}

inline void AddUndirected(Graph& graph, int a, int b, double cost) {
    // Précondition : a et b sont des indices valides.
    graph.at(a).push_back({ b, cost });
    graph.at(b).push_back({ a, cost });
}