#include "NavMesh.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    const Polygon rectangle = MakePolygon({ {0,0},{4,0},{4,2},{0,2} });
    assert(SamePoint(rectangle.center, Point{ 2,1 }));
    assert(Contains(rectangle, { 2,1 }));
    assert(!Contains(rectangle, { 5,1 }));
    assert(Contains(rectangle, { 4,1 }));
    assert(Contains(rectangle, { 0,0 }));

    auto vertices = rectangle.vertices;
    std::reverse(vertices.begin(), vertices.end());
    const Polygon reversed = MakePolygon(vertices);
    assert(Contains(reversed, { 2,1 }));
    assert(!Contains(reversed, { 5,1 }));
    assert(Contains(reversed, { 4,1 }));
    assert(Contains(reversed, { 0,0 }));

    NavMesh mesh;
    mesh.polygons = { rectangle,
        MakePolygon({{4,0},{6,0},{6,2},{4,2}}),
        MakePolygon({{4,2},{6,2},{6,6},{4,6}}),
        MakePolygon({{8,0},{10,0},{10,2},{8,2}}) };
    assert(mesh.Locate({ 1,1 }) == 0);
    assert(mesh.Locate({ 5,5 }) == 2);
    assert(mesh.Locate({ 3,3 }) == -1);
    assert(mesh.Locate({ 9,1 }) == 3);
    assert(mesh.Locate({ 4,1 }) == 0);

    const double infinity = std::numeric_limits<double>::infinity();
    assert(!Contains(rectangle, { infinity,1 }));
    assert(mesh.Locate({ infinity,1 }) == -1);

    bool rejected = false;
    try { MakePolygon({ {0,0},{1,0} }); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);

    rejected = false;
    try { MakePolygon({ {0,0},{4,0},{4,2},{0,0} }); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);

    rejected = false;
    try { MakePolygon({ {0,0},{4,0},{infinity,2} }); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);

    std::cout << "Tests partie 5 OK\n";
}
