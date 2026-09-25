#pragma once 
#include "phx_vector.hpp"
#include "phx_Body.hpp"


namespace Phx{

struct Manifold{

    Body* bodyA;
    Body* bodyB;

    Vec2 normal;
    Vec2 contactPointA;
    Vec2 contactPointB;

    float penetration;

};


};
