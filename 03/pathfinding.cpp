#include "NavMesh.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    NavMesh mesh = MakeDemoMesh();

    // Deux portails et quatre sommets de graphe.
    assert(mesh.portals.size() == 2);
    assert(mesh.graph.size() == 4);

    // Nombre de voisins attendu.
    assert(mesh.graph[0].size() == 1);
    assert(mesh.graph[1].size() == 2);
    assert(mesh.graph[2].size() == 1);
    assert(mesh.graph[3].empty());

    // Recherche du portail dans les deux sens.
    const Portal* forward = mesh.Between(0, 1);
    const Portal* backward = mesh.Between(1, 0);

    assert(forward != nullptr);
    assert(backward == forward);
    assert(mesh.Between(0, 3) == nullptr);

    // Milieux des deux portails.
    assert(SamePoint(forward->Middle(), Point{4,1}));

    const Portal* second = mesh.Between(1, 2);
    assert(second != nullptr);
    assert(SamePoint(second->Middle(), Point{5,2}));

    // Toutes les connexions de cette carte coûtent 3.
    for (const auto& edges : mesh.graph) {
        for (const Edge& edge : edges) {
            assert(std::abs(edge.cost - 3.0) < 1e-4);
        }
    }

    // Reconstruire ne doit pas doubler les connexions.
    mesh.BuildGraph();

    assert(mesh.portals.size() == 2);
    assert(mesh.graph[0].size() == 1);
    assert(mesh.graph[1].size() == 2);
    assert(mesh.graph[2].size() == 1);

    // Rejeter un portail de longueur nulle.
    NavMesh invalid = mesh;
    invalid.portals.push_back({0, 1, {4,1}, {4,1}});

    bool rejected = false;

    try {
        invalid.BuildGraph();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);

    // Rejeter un indice inexistant.
    invalid = mesh;
    invalid.portals.push_back({0, 99, {4,0}, {4,2}});

    rejected = false;

    try {
        invalid.BuildGraph();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);

    // Rejeter un portail hors des régions.
    invalid = mesh;
    invalid.portals.push_back({0, 1, {20,0}, {20,2}});

    rejected = false;

    try {
        invalid.BuildGraph();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);

    std::cout << "Tests partie 6 OK\n";

    // Afficher le graphe obtenu.
    for (int u = 0; u < static_cast<int>(mesh.graph.size()); ++u) {
        for (const Edge& edge : mesh.graph[u]) {
            std::cout << u << " -> " << edge.to
                      << " : cout " << edge.cost << '\n';
        }
    }
}
