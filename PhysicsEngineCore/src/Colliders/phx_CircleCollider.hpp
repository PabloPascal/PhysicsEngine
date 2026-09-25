#pragma once 
#include "phx_Collider.hpp"


namespace Phx{

class CircleCollider: public Collider{
    
    float m_radius;

    public:
        CircleCollider(float radius);

        AABB computeAABB(const Body& circle) const override;

        ColliderType type() const {
            return ColliderType::Circle;    
        }

        float get_radius() const {return m_radius;}

};



};