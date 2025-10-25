#ifndef FORCE_CALC_HPP
#define FORCE_CALC_HPP


#include <phx_circle.hpp>

namespace Phx{

class ForceSolver{
public:

    static void ForceGravity(Circle& c1, Circle& c2);

    static void FallGravity(Circle &c, float gravity,  float dt);

    
};
}

#endif