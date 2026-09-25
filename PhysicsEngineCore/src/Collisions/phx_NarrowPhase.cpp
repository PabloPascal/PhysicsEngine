#include "phx_NarrowPhase.hpp"
#include "../phx_Body.hpp"
#include "../Colliders/phx_CircleCollider.hpp"
#include "../Colliders/phx_RectCollider.hpp"
#include "../Geometry/phx_vector.hpp"
#include "Geometry/phx_utils.hpp"
#include "phx_Manifolds.hpp"

#include <vector>


namespace Phx{


bool NarrowPhase::checkCirclesCollision(const Body& circleA, const Body& circleB){
    
    float rA = static_cast<const CircleCollider&>(*circleA.collider).get_radius();
    float rB = static_cast<const CircleCollider&>(*circleB.collider).get_radius();

    if(length(circleA.position - circleB.position) <= rA + rB){
        return true;
    }

    return false;

}


bool NarrowPhase::checkRectsCollision(const Body& rectA, const Body& rectB, Manifold& manifold){
    
    Vec2 allAxis[4] = {getRectAxis(rectA)[0], getRectAxis(rectA)[1],
                       getRectAxis(rectB)[0], getRectAxis(rectB)[1]};


    std::array<Vec2, 4> verticesA = getRectVertices(rectA);
    std::array<Vec2, 4> verticesB = getRectVertices(rectB);
    

    manifold.penetration = std::numeric_limits<float>::max();
    
    for(auto axis : allAxis)
    {
        float min1, max1, min2, max2;
        
        findProjection(axis, min1, max1, verticesA);
        findProjection(axis, min2, max2, verticesB);

        float overlap = std::min(max1, max2) - std::max(min1, min2);
        

        if(max1 < min2 || max2 < min1){
            manifold.penetration = 0;
            return false;
        }

        if(overlap < manifold.penetration)
        {
            manifold.penetration = overlap;
            manifold.normal = axis;
        }

    }

    Vec2 center_dir = (rectB.position - rectA.position);
    if (length(center_dir) > 0.0f) {
        center_dir.normalize();

        if(dot(center_dir, manifold.normal) < 0)
        {
            manifold.normal = -1 * manifold.normal;
        }

    }
    
    std::vector<Vec2> vertex_inside;

    for(auto vert : verticesA)
    {
        if(checkPointInsideRect(rectB, vert))
        {
            vertex_inside.push_back(vert);
        }
    }

    for(auto vert : verticesB)
    {
        if(checkPointInsideRect(rectA, vert))
        {
            vertex_inside.push_back(vert);
        }
    }

    Vec2 sum;

    for(auto vert : vertex_inside)
    {
        sum = sum + vert;
    }

    if(!vertex_inside.empty())
        sum = sum / vertex_inside.size();

    manifold.contactPointA = sum;
    manifold.contactPointB = sum;


    return true;

}


bool NarrowPhase::checkCircleRectCollision(const Body& circle, const Body& rect, Manifold& manifold){
    Vec2 center = circle.position;
    const CircleCollider& c_collider = static_cast<const CircleCollider&>(*circle.collider);
    const RectCollider& r_collider = static_cast<const RectCollider&>(*rect.collider);
    
    float r = c_collider.get_radius();

    float half_w = r_collider.get_width() / 2.f;
    float half_h = r_collider.get_height() / 2.f;

    const auto axis = getRectAxis(rect); 

    Vec2 dv = center - rect.position;
    Vec2 local = {dot(dv, axis[0]), dot(dv, axis[1])};
    Vec2 closestPoint;
   
    //clamping
    closestPoint.x = std::max(-half_w, std::min(dot(dv, axis[0]), half_w));
    closestPoint.y = std::max(-half_h, std::min(dot(dv, axis[1]), half_h));


    Vec2 dir = {local.x - closestPoint.x, local.y - closestPoint.y};

    float dist = dot(dir, dir); 

    Vec2 local_norm;

    if(dist < 0.000001f)
    {
        float dx = half_w - abs(local.x);
        float dy = half_h - abs(local.y);

    if(dx < dy)
    {
        local_norm = {
            local.x > 0 ? 1.f : -1.f,
            0
        };

        manifold.penetration = r + dx;
    }
    else
    {
        local_norm = {
            0,
            local.y > 0 ? 1.f : -1.f
        };

        manifold.penetration = r + dy;
    }
    }
    else
    {
        local_norm = dir.normalize();
    }


    //normal = transpose(rect.get_transform()) * local_norm; 

    manifold.penetration = r - std::sqrt(dist);  
    
    Matrix2 transform(axis[0], axis[1]);

    //inverse matrix for ortogonal matrix is transpone matrix
    manifold.normal = -1 * (transform * local_norm); 

    manifold.contactPointA = rect.position + transform * closestPoint;
    manifold.contactPointB = rect.position + transform * closestPoint;

    if(dist <= r * r)
        return true;
    else    
        return false;

}



bool NarrowPhase::collision(const Body& bodyA, const Body& bodyB, Manifold& manifold){

}



}
