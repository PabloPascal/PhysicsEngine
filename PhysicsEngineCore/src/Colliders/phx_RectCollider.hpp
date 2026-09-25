#pragma once 

#include "phx_Collider.hpp"


namespace Phx{

    class RectCollider: public Collider{
        
        float m_width;
        float m_height;
        
        public:

            RectCollider(float width, float height);

            float get_width() const {return m_width;}
            float get_height() const {return m_height;}

            AABB computeAABB(const Body& rect) const override;

            ColliderType type() const override{
                return ColliderType::Rect;
            }
            
        };


};