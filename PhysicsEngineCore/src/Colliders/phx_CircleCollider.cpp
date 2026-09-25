#include "phx_CircleCollider.hpp"
#include "phx_Body.hpp"


namespace Phx{

    CircleCollider::CircleCollider(float radius){

        if(radius <= 0) {
            throw std::invalid_argument("radius must be a positive number");
        }

        m_radius = radius;
    }

    AABB CircleCollider::computeAABB(const Body& circle) const {
        AABB aabb;

        aabb.min = {circle.position.x - m_radius, circle.position.y - m_radius};
        aabb.max = {circle.position.x + m_radius, circle.position.y + m_radius};

        return aabb;
    }


};