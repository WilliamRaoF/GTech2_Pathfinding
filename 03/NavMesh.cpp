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
