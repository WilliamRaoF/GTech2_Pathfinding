#pragma once
#include "Graph.hpp"
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <queue>
#include <tuple>

// Ce fichier remplace la version du jour 1.
enum class Status { Found, NoPath, InvalidInput, OutsideNavMesh };
struct SearchResult {
    Status status = Status::InvalidInput;
    std::vector<int> nodes;
    double cost = std::numeric_limits<double>::infinity();
    int expanded = 0;
    double microseconds = 0.0;
};
using Heuristic = std::function<double(int)>;

inline SearchResult Search(const Graph& graph, int start, int goal,
    const Heuristic& heuristic) {
    using Clock = std::chrono::steady_clock;
    const auto begin = Clock::now();
    SearchResult result;
    auto finish = [&]() {
        result.microseconds =
            std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
        return result;
        };
    if (!ValidGraph(graph) || !ValidNode(graph, start) ||
        !ValidNode(graph, goal) || !heuristic) return finish();

    // Pré-calcul pratique pour valider les valeurs et ne pas les recalculer.
    // La cohérence ou l'admissibilité reste à justifier séparément.
    std::vector<double> h(graph.size());
    for (int i = 0; i < static_cast<int>(graph.size()); ++i) {
        h[i] = heuristic(i);
        if (!std::isfinite(h[i]) || h[i] < 0.0) return finish();
    }
    if (std::abs(h[goal]) > 1e-9) return finish();

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> g(graph.size(), infinity);
    std::vector<int> parent(graph.size(), -1);
    // Ordre de comparaison : f, puis g, puis identifiant.
    using Entry = std::tuple<double, double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    g[start] = 0.0;
    open.push({ h[start], 0.0, start });
    while (!open.empty()) {
        const auto [priority, savedG, u] = open.top();
        open.pop();
        (void)priority;
        if (savedG > g[u]) continue;
        if (u == goal) {
            result.status = Status::Found;
            result.cost = g[goal];
            for (int v = goal; v != -1; v = parent[v]) result.nodes.push_back(v);
            std::reverse(result.nodes.begin(), result.nodes.end());
            return finish();
        }
        ++result.expanded;
        for (const Edge& edge : graph[u]) {
            const double candidate = g[u] + edge.cost;
            if (candidate < g[edge.to]) {
                g[edge.to] = candidate;
                parent[edge.to] = u;
                open.push({ candidate + h[edge.to], candidate, edge.to });
            }
        }
    }
    result.status = Status::NoPath;
    return finish();
}

inline SearchResult Dijkstra(const Graph& graph, int start, int goal) {
    return Search(graph, start, goal, [](int) { return 0.0; });
}
inline SearchResult AStar(const Graph& graph, int start, int goal,
    const Heuristic& heuristic) {
    return Search(graph, start, goal, heuristic);
}