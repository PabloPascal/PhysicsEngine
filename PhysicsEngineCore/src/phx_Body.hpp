#ifndef BODY_HPP
#define BODY_HPP

#include "Geometry/phx_vector.hpp"
#include "Geometry/phx_matrix.hpp"
#include "Colliders/phx_Collider.hpp"
#include "phx_Material.hpp"
#include <stdexcept>

namespace Phx{

/*
This is physical body and it has physical parameters: mass, velocity etc.
*/



class Body{
public:
    float mass = 0.f;
    float inv_mass = 0.f;
    float angle = 0.f;
    float angle_velocity = 0.f;
    float inertia = 0.f;
    float inv_inertia = 0.f;
    float torque = 0.f;

    Vec2 position;
    Vec2 velocity;
    Vec2 force;
    
    const Collider* collider = nullptr;
    Material material;

    bool is_static = false;


    void set_mass(float m){
        if(m < 0){
            throw std::invalid_argument("mass must be a positive number");
        }
        mass = m;
        if(mass == 0)
            inv_mass = 0;
        else{
            inv_mass = 1/mass;
        }
    }

    void set_inertia(float I){
        if(I < 0){
            throw std::invalid_argument("Inertia must be a positive number");
        }

        inertia = I;
        if(I == 0)
            inv_inertia = 0;
        else{
            inv_inertia = 1/inertia;
        }
    }

    void set_static(bool s){
        is_static = s;

        if(s){
            inv_mass = 0;
            inv_inertia = 0;
        }else{
            inv_mass = 1/mass;
            inv_inertia = 1/inertia;
        }
    }


};


}


#endif 