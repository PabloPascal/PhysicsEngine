#ifndef BODY_HPP
#define BODY_HPP

#include "Geometry/phx_vector.hpp"
#include "Geometry/phx_matrix.hpp"
#include "Colliders/phx_Collider.hpp"

/*
This is physical body and it has physical parameters: mass, velocity etc.
*/


namespace Phx{


class Body{
public:
    float mass;
    float inv_mass;
    float angle;
    float angle_velocity;
    float inertia;
    float inv_inertia;
    float torque;

    Vec2 position;
    Vec2 velocity;
    Vec2 force;
    
    const Collider* collider;

};


}


#endif 