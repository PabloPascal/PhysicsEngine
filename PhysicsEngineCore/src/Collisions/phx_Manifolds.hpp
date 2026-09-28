#pragma once 
#include "Geometry/phx_vector.hpp"
#include "phx_Body.hpp"


namespace Phx{

struct Manifold{

    Body* bodyA = nullptr;
    Body* bodyB = nullptr;

    Vec2 normal = {0.f, 0.f};
    Vec2 contactPointA = {0.f, 0.f};
    Vec2 contactPointB = {0.f, 0.f};

    float penetration = 0.f;

};


};
