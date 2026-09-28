#include "phx_World.hpp"
#include "Collisions/phx_Manifolds.hpp"


namespace Phx{


void PhysicsWorld::step(float dt){

    for (auto& b : m_bodies) {
        if (b->is_static) continue;
        b->force = b->force + Vec2(0.f, -9.81f) * b->mass;
    }


    for(size_t i = 0; i < m_bodies.size(); i++){
        m_integrator.solver(*m_bodies[i], dt);
    }

    m_manifolds.clear();

    for(size_t i = 0; i < m_bodies.size() - 1; i++){
        for(size_t j = i + 1; j < m_bodies.size(); j++){
            
            Manifold manifold;
            
            if(m_narrow_phase.collision(*m_bodies[i], *m_bodies[j], manifold))
                m_manifolds.emplace_back(manifold);
        }
    }

    for(size_t i = 0; i < m_manifolds.size(); i++){
        m_solver.resolveCollision(m_manifolds[i]);
    }

    for(auto& b : m_bodies){
        b->force = 0.f;
        b->torque = 0.f;
    }

}

void PhysicsWorld::addBody(std::unique_ptr<Body> body){
    m_bodies.emplace_back(std::move(body));
}


}
