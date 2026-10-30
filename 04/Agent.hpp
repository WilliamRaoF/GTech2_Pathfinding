#pragma once
#include "NavMesh.hpp"

struct NavAgent {
    Point position;
    double speed = 2.0;
    NavPath path;
    std::size_t next = 0;
    double requestCooldown = 0.0;
    Point lastGoal;
    bool hasGoal = false;
    int requests = 0;
    bool useAStar = true;

    void Tick(double dt);
    void Clear();
    bool Finished() const;
    void Plan(const NavMesh& mesh, Point goal);
    void Follow(double dt);
};

enum class EnemyState { Idle, Chase, Windup, Recover, Dead };
struct Target { Point position; int hp = 100; };
struct Enemy {
    NavAgent nav;
    EnemyState state = EnemyState::Idle;
    int hp = 30;
    int damage = 10;
    double detectRange = 8.0, loseRange = 10.0, attackRange = 0.65;
    double windupDuration = 0.5, recoverDuration = 0.8;
    double timer = 0.0;

    void Update(const NavMesh& mesh, Target& target, double dt);
};
