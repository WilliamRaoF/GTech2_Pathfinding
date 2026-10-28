#include "NavMesh.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    const NavMesh mesh = MakeDemoMesh();

    // Même requête avec les deux algorithmes.
    const auto a = FindNavPath(
        mesh, { 1,1 }, { 5,5 }, true
    );

    const auto d = FindNavPath(
        mesh, { 1,1 }, { 5,5 }, false
    );

    assert(a.search.status == Status::Found);
    assert(d.search.status == Status::Found);

    assert(std::abs(a.search.cost - d.search.cost) < 1e-4);

    // Coût du graphe : 0 -> 1 -> 2.
    assert(std::abs(a.search.cost - 6.0) < 1e-4);

    // Trajet complet, avec raccordements départ et arrivée.
    assert(std::abs(a.routeCost - 8.0) < 1e-4);

    assert((a.search.nodes == std::vector<int>{0, 1, 2}));

    const std::vector<Point> expectedPoints = {
        {1,1}, // Départ
        {2,1}, // Centre région 0
        {4,1}, // Milieu portail 0-1
        {5,1}, // Centre région 1
        {5,2}, // Milieu portail 1-2
        {5,4}, // Centre région 2
        {5,5} // Arrivée
    };

    assert(a.points.size() == expectedPoints.size());

    for (std::size_t i = 0; i < expectedPoints.size(); ++i) {
        assert(SamePoint(a.points[i], expectedPoints[i]));
    }

    // Arrivée sur l'île : localisable, mais inaccessible.
    const auto island =
        FindNavPath(mesh, { 1,1 }, { 9,1 });

    assert(island.search.status == Status::NoPath);
    assert(island.points.empty());
    assert(std::isinf(island.routeCost));

    // Arrivée extérieure.
    const auto outside =
        FindNavPath(mesh, { 1,1 }, { 3,3 });

    assert(outside.search.status == Status::OutsideNavMesh);
    assert(outside.points.empty());

    // Deux positions dans une même région.
    const auto local =
        FindNavPath(mesh, { 1,1 }, { 3,1 });

    assert(local.search.status == Status::Found);
    assert(local.points.size() == 2);
    assert(std::abs(local.search.cost) < 1e-4);
    assert(std::abs(local.routeCost - 2.0) < 1e-4);

    // Départ et arrivée identiques.
    const auto same =
        FindNavPath(mesh, { 1,1 }, { 1,1 });

    assert(same.search.status == Status::Found);
    assert(same.points.size() == 1);
    assert(same.routeCost == 0.0);

    // Coordonnées invalides.
    const double infinity =
        std::numeric_limits<double>::infinity();

    const auto invalid =
        FindNavPath(mesh, { infinity,1 }, { 5,5 });

    assert(invalid.search.status == Status::InvalidInput);

    // Graphe et portails volontairement désynchronisés.
    NavMesh broken = mesh;
    broken.portals.clear();

    const auto inconsistent =
        FindNavPath(broken, { 1,1 }, { 5,5 });

    assert(inconsistent.search.status == Status::InvalidInput);
    assert(inconsistent.points.empty());

    std::cout << "Tests partie 7 OK\n";
    std::cout << "Cout du graphe : " << a.search.cost << '\n';
    std::cout << "Longueur du trajet : " << a.routeCost << '\n';

    for (Point p : a.points) {
        std::cout << '(' << p.x << ',' << p.z << ")\n";
    }
}
