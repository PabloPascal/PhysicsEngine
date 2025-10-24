#ifndef FORCE_CALC_HPP
#define FORCE_CALC_HPP


#include <phx_circle.hpp>

namespace Phx{

void ForceGravity(Circle& c1, Circle& c2);

void FallGravity(Circle &c, float gravity,  float dt);

}

#endif