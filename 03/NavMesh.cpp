#include "NavMesh.hpp"
#include <stdexcept>

Polygon MakePolygon(std::vector<Point> vertices) {
    if (vertices.size() < 3)
        throw std::invalid_argument("Au moins trois sommets requis.");

    Point sum{};
    for (std::size_t i = 0;i < vertices.size();++i) {
        if (!Finite(vertices[i]))
            throw std::invalid_argument("Coordonnees invalides.");
        for (std::size_t j = 0;j < i;++j) {
            if (SamePoint(vertices[i], vertices[j]))
                throw std::invalid_argument("Sommet repete.");
        }
        sum = sum + vertices[i];
    }
    const Point center = sum * (1.0 / static_cast<double>(vertices.size()));
    return { vertices,center };
}

bool Contains(const Polygon& polygon, Point point) {
    if (!Finite(point) || polygon.vertices.size() < 3) return false;
    constexpr double epsilon = 1e-8;
    bool positive = false, negative = false;
    const auto& vertices = polygon.vertices;
    for (std::size_t i = 0;i < vertices.size();++i) {
        const Point a = vertices[i];
        const Point b = vertices[(i + 1) % vertices.size()];
        const double side = Cross(b - a, point - a);
        if (side >= epsilon) positive = true;
        if (side < -epsilon) negative = true;
        if (positive && negative) return false;
    }
    return true;
}

int NavMesh::Locate(Point p) const {
    if (!Finite(p)) return -1;
    for (int i = 0;i < static_cast<int>(polygons.size());++i) {
        if (Contains(polygons[i], p)) return i;
    }
    return -1;
}

// Retrouver le portail entre deux régions, dans les deux sens.
const Portal* NavMesh::Between(int a, int b) const {
    for (const Portal& portal : portals) {
        const bool forward =
            portal.a == a && portal.b == b;

        const bool backward =
            portal.a == b && portal.b == a;

        if (forward || backward) {
            return &portal;
        }
    }

    return nullptr;
}

void NavMesh::BuildGraph() {
    graph.assign(polygons.size(), {});
    for (const Portal& portal : portals) {
        // Les indices doivent désigner deux régions différentes.
        if (!ValidNode(graph, portal.a) ||
            !ValidNode(graph, portal.b) ||
            portal.a == portal.b) {
            throw std::invalid_argument(
                "Indices de portail invalides."
            );
        }
        // Le portail doit avoir des coordonnées finies
        // et une longueur non nulle.
        if (!Finite(portal.left) ||
            !Finite(portal.right) ||
            Distance(portal.left, portal.right) < 1e-8) {
            throw std::invalid_argument(
                "Portail invalide ou de longueur nulle."
            );
        }

        const Polygon& regionA = polygons[portal.a];
        const Polygon& regionB = polygons[portal.b];

        // Les deux extrémités doivent appartenir aux deux régions.
        if (!Contains(regionA, portal.left) ||
            !Contains(regionA, portal.right) ||
            !Contains(regionB, portal.left) ||
            !Contains(regionB, portal.right)) {
            throw std::invalid_argument(
                "Le portail n'appartient pas aux deux regions."
            );
        }

        const Point middle = portal.Middle();

        const double cost =
            Distance(regionA.center, middle)
            + Distance(middle, regionB.center);

        AddUndirected(graph, portal.a, portal.b, cost);
    }
}


NavMesh MakeDemoMesh() {
    NavMesh mesh;
    mesh.polygons = {
        // Région 0.
        MakePolygon({{0,0}, {4,0}, {4,2}, {0,2}}),
        // Région 1.
        MakePolygon({{4,0}, {6,0}, {6,2}, {4,2}}),
        // Région 2.
        MakePolygon({{4,2}, {6,2}, {6,6}, {4,6}}),
        // Région 3 : île.
        MakePolygon({{8,0}, {10,0}, {10,2}, {8,2}})
    };

    mesh.portals = {
        // Régions 0 et 1
        {0, 1, {4,0}, {4,2}},
        // Régions 1 et 2
        {1, 2, {4,2}, {6,2}}
    };
    mesh.BuildGraph();
    return mesh;
}
