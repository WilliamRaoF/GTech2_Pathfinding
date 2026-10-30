#include "Agent.hpp"
#include <algorithm>
#include <utility>

bool SegmentInside(const NavMesh& mesh, Point a, Point b) {
    if (!Finite(a) || !Finite(b)) return false;
    std::vector<std::pair<double,double>> intervals;
    const Point direction=b-a;
    for (const Polygon& polygon : mesh.polygons) {
        const auto& v=polygon.vertices;
        double orientation=Cross(v[1]-v[0],v[2]-v[1])>0 ? 1.0 : -1.0;
        double low=0.0, high=1.0;
        bool valid=true;
        for (std::size_t i=0; i<v.size(); ++i) {
            Point edge=v[(i+1)%v.size()]-v[i];
            double base=orientation*Cross(edge,a-v[i]);
            double slope=orientation*Cross(edge,direction);
            // base + slope*t >= -epsilon
            if (std::abs(slope)<1e-12) {
                if (base < -1e-9) { valid=false; break; }
            } else {
                double bound=(-1e-9-base)/slope;
                if (slope>0) low=std::max(low,bound);
                else high=std::min(high,bound);
            }
        }
        if (valid && low<=high) intervals.push_back({low,high});
    }
    std::sort(intervals.begin(),intervals.end());
    double covered=0.0;
    for (auto [low,high] : intervals) {
        if (low>covered+1e-8) return false;
        covered=std::max(covered,high);
        if (covered>=1.0-1e-8) return true;
    }
    return false;
}

const char* StateName(EnemyState state) {
    switch (state) {
        case EnemyState::Idle: return "Idle";
        case EnemyState::Chase: return "Chase";
        case EnemyState::Windup: return "Windup";
        case EnemyState::Recover: return "Recover";
        case EnemyState::Dead: return "Dead";
    }
    return "Unknown";
}
