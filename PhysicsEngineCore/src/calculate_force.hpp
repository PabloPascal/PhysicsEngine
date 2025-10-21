#ifndef FORCE_CALC_HPP
#define FORCE_CALC_HPP


#include <phx_circle.hpp>

namespace Phx{

void calcGravityForce(Circle& c1, Circle& c2);

void gravity(Circle &c, float gravity,  float dt);

}

#endif