#ifndef PHYSICS_WOLRD_HPP
#define PHYSICS_WOLRD_HPP

#include "phx_Body.hpp"
#include "Dynamics/phx_Integrator.hpp"
#include "Collisions/phx_Solver.hpp"
#include "Collisions/phx_NarrowPhase.hpp"
#include "Collisions/phx_Manifolds.hpp"

#include <memory>
#include <vector>

namespace Phx{


class PhysicsWorld
{
    std::vector<std::unique_ptr<Body>> m_bodies;
    std::vector<Manifold> m_manifolds;

    Integrator  m_integrator;
    Solver      m_solver;
    NarrowPhase m_narrow_phase;

public:

    void step(float dt);

    void addBody(std::unique_ptr<Body> body);

    const std::vector<std::unique_ptr<Body>>& get_bodies() const {return m_bodies;}
};

}


#endif 