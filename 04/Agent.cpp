#include "Agent.hpp"
#include <algorithm>
#include <stdexcept>

void NavAgent::Tick(double dt) { requestCooldown=std::max(0.0,requestCooldown-dt); }

void NavAgent::Clear() { path=NavPath{}; next=0; hasGoal=false; }

bool NavAgent::Finished() const { return next>=path.points.size(); }

void NavAgent::Plan(const NavMesh& mesh, Point goal) {
    if (requestCooldown > 0.0) return;
    bool changed = !hasGoal || Distance(goal, lastGoal) > 0.75;
    bool retry = path.search.status != Status::Found;
    bool stoppedFar = Finished() && Distance(position, goal) > 0.1;
    if (!changed && !retry && !stoppedFar) return;
    path = FindNavPath(mesh, position, goal, useAStar);
    next = path.points.size() > 1 ? 1 : path.points.size();
    lastGoal = goal;
    hasGoal = true;
    requestCooldown = 0.3;
    ++requests;
}
void NavAgent::Follow(double dt) {
    if (path.search.status != Status::Found || dt <= 0 || speed <= 0) return;
    double budget = speed * dt;
    while (!Finished() && budget > 0) {
        Point delta = path.points[next] - position;
        double distance = Length(delta);
        if (distance < 1e-8) { ++next; continue; }
        if (distance <= budget) {
            position = path.points[next++];
            budget -= distance;
        }
        else {
            position = position + delta * (budget / distance);
            budget = 0;
        }
    }
}

void Enemy::Update(const NavMesh&, Target&, double) {
    // TODO : perception, priorités, automate, poursuite et attaque.
    throw std::logic_error("A implementer : Enemy::Update");
}
