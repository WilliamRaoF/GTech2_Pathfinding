#include "Agent.hpp"
#include <algorithm>
#include <stdexcept>

void NavAgent::Tick(double dt) { requestCooldown=std::max(0.0,requestCooldown-dt); }

void NavAgent::Clear() { path=NavPath{}; next=0; hasGoal=false; }

bool NavAgent::Finished() const { return next>=path.points.size(); }

void NavAgent::Plan(const NavMesh&, Point) {
    // TODO : politique de recalcul, appel à FindNavPath et compteur.
    throw std::logic_error("A implementer : NavAgent::Plan");
}

void NavAgent::Follow(double) {
    // TODO : budget de distance, suivi de plusieurs points, aucun dépassement.
    throw std::logic_error("A implementer : NavAgent::Follow");
}

void Enemy::Update(const NavMesh&, Target&, double) {
    // TODO : perception, priorités, automate, poursuite et attaque.
    throw std::logic_error("A implementer : Enemy::Update");
}
