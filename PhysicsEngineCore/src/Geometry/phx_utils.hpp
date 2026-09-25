#pragma once 
#include <array>
#include "phx_vector.hpp"
#include "phx_Body.hpp"
#include "Colliders/phx_RectCollider.hpp"
#include "phx_matrix.hpp"
#include <cmath>



namespace Phx{

    std::array<Vec2, 4> getRectVertices(const Body& rect){

        const RectCollider& collider = static_cast<const RectCollider&>(*rect.collider);


        float angle = rect.angle;

        Matrix2 R(std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle));

        Vec2 v1 = Vec2{-collider.get_width()/2, collider.get_height()/2};
        Vec2 v2 = Vec2{collider.get_width()/2, collider.get_height()/2};
        Vec2 v3 = Vec2{ collider.get_width()/2, -collider.get_height()/2};
        Vec2 v4 = Vec2{ -collider.get_width()/2, -collider.get_height()/2};

        Vec2 v_new1 = R * v1 + rect.position; 
        Vec2 v_new2 = R * v2 + rect.position;
        Vec2 v_new3 = R * v3 + rect.position;
        Vec2 v_new4 = R * v4 + rect.position;

        return {v_new1, v_new2, v_new3, v_new4};
    } 


    std::array<Vec2, 2> getRectAxis(const Body& rect){

        float angle = rect.angle;
        Vec2 axis1 = {1, 0};
        Vec2 axis2 = {0, 1};

        Matrix2 R(std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle));


        return {R*axis1, R*axis2};
    }



    void findProjection(Vec2 Axis, float& minProj, float& maxProj, std::array<Vec2, 4>& vertices){
        minProj = maxProj = dot(Axis, vertices[0]);

        for(auto v : vertices)
        {
            float projection = dot(Axis, v);
            if(projection > maxProj) maxProj = projection;
            else if(projection < minProj) minProj = projection;
        }
    }


    bool checkPointInsideRect(const Body& rect, Vec2 point){
        std::array<Vec2,2> axis = getRectAxis(rect);
        const RectCollider& collider = static_cast<const RectCollider&>(*rect.collider); 

        Vec2 e1 = axis[0];
        Vec2 e2 = axis[1];

        Matrix2 T(e1, e2);

        Vec2 p = point - rect.position;
        Vec2 p_local = transpose(T)*p; 

        float half_w = collider.get_width()/2.f;
        float half_h = collider.get_height()/2.f;  
        
        if(std::abs(p_local.x) < half_w && std::abs(p_local.y) < half_h) 
            return true; 


        return false;

    }
    


}
