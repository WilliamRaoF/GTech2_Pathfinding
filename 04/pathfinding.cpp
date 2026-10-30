#include "Agent.hpp"
#include <cassert>
#include <iostream>
#include <stdexcept>

#ifdef NDEBUG
#error Activer les assertions pour les tests.
#endif

int main() {
    NavMesh mesh=MakeDemoMesh();
    assert(SegmentInside(mesh,{1,1},{5,1}));
    assert(!SegmentInside(mesh,{1,1},{5,5}));
    assert(!SegmentInside(mesh,{1,1},{9,1}));

    Target player{{5,5},100};
    Enemy enemy;
    enemy.nav.position={1,1};
    constexpr double dt=1.0/60.0;
    for (int frame=0; frame<600; ++frame) {
        enemy.Update(mesh,player,dt);
        assert(mesh.Locate(enemy.nav.position)>=0);
    }
    assert(player.hp<100);
    assert(enemy.nav.requests<20);

    // Un seul impact pendant cette préparation.
    Target close{{1.3,1},100};
    Enemy attacker;
    attacker.nav.position={1,1};
    attacker.state=EnemyState::Windup;
    attacker.timer=0.1;
    for (int i=0; i<30; ++i) attacker.Update(mesh,close,dt);
    assert(close.hp==90);

    // La cible esquive avant l'impact.
    Target escaped{{5,5},100};
    Enemy miss;
    miss.nav.position={1,1};
    miss.state=EnemyState::Windup;
    miss.timer=0.1;
    for (int i=0; i<30; ++i) miss.Update(mesh,escaped,dt);
    assert(escaped.hp==100);

    // Une cible extérieure n'entraîne aucun déplacement à travers le vide.
    Target outside{{3,3},100};
    Enemy blocked;
    blocked.nav.position={1,1};
    for (int i=0; i<120; ++i) blocked.Update(mesh,outside,dt);
    assert(Distance(blocked.nav.position,{1,1})<1e-9);
    assert(blocked.nav.path.search.status==Status::OutsideNavMesh);
    blocked.hp=0;
    blocked.Update(mesh,outside,dt);
    assert(blocked.state==EnemyState::Dead);
    NavAgent follower;
    follower.position={1,1};
    follower.speed=2.0;
    follower.path=FindNavPath(mesh,{1,1},{5,5});
    follower.next=1;
    follower.Follow(1.0);
    assert(SamePoint(follower.position,Point{3,1}));
    follower.Follow(10.0);
    assert(SamePoint(follower.position,Point{5,5}));
    assert(follower.Finished());

    Target protectedTarget{{1.3,1},100};
    Enemy dying;
    dying.nav.position={1,1};
    dying.state=EnemyState::Windup;
    dying.timer=0.01;
    dying.hp=0;
    dying.Update(mesh,protectedTarget,dt);
    assert(dying.state==EnemyState::Dead);
    assert(protectedTarget.hp==100);

    Target deadTarget{{1.3,1},0};
    Enemy living;
    living.nav.position={1,1};
    living.state=EnemyState::Chase;
    living.Update(mesh,deadTarget,dt);
    assert(living.state==EnemyState::Idle);

    std::cout << "Tests Jour 4 OK\n";
}
