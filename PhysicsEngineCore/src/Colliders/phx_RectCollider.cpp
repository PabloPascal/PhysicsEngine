#include "phx_RectCollider.hpp"
#include "phx_Body.hpp"
#include <string>

namespace Phx{

    RectCollider::RectCollider(float width, float height){
        if(width <= 0 || height <= 0) throw std::invalid_argument("width or height must be positive number");
        m_width = width;
        m_height = height;
    }

    AABB RectCollider::computeAABB(const Body& rect) const {
        AABB aabb;

        aabb.min = {rect.position.x - m_width/2.f, rect.position.y - m_height/2.f};
        aabb.max = {rect.position.x + m_width/2.f, rect.position.y + m_height/2.f};

        return aabb;
    }


};