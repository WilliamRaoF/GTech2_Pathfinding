#pragma once
#include "Search.hpp"
#include <cmath>
#include <stdexcept>

struct Point {
    double x = 0.0, z = 0.0;
};

inline Point operator+(Point a, Point b) { return { a.x + b.x,a.z + b.z }; }
inline Point operator-(Point a, Point b) { return { a.x - b.x,a.z - b.z }; }
inline Point operator*(Point a, double s) { return { a.x * s,a.z * s }; }
inline double Cross(Point a, Point b) { return a.x * b.z - a.z * b.x; }
inline double Length(Point a) { return std::hypot(a.x, a.z); }
inline double Distance(Point a, Point b) { return Length(a - b); }
inline bool Finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.z); }
inline bool SamePoint(Point a, Point b) { return Distance(a, b) < 1e-8; }

struct Polygon {
    std::vector<Point> vertices;
    Point center;
};
struct Portal {
    int a, b;
    Point left, right;
    Point Middle() const { return (left + right) * 0.5; }
};

// À définir après les structures : étape 5.
bool Contains(const Polygon& polygon, Point point);
Polygon MakePolygon(std::vector<Point> vertices);

struct NavMesh {
    std::vector<Polygon> polygons;
    std::vector<Portal> portals;
    Graph graph;

    int Locate(Point p) const;                 // Étape 5
    const Portal* Between(int a, int b) const;  // Étape 6
    void BuildGraph();                        // Étape 6
};

struct NavPath {
    SearchResult search;
    std::vector<Point> points;
    double routeCost = std::numeric_limits<double>::infinity();
};

inline NavMesh MakeDemoMesh();                 // Étape 6
inline NavPath FindNavPath(const NavMesh& mesh, Point start,
    Point goal, bool useAStar = true); // Étape 7
