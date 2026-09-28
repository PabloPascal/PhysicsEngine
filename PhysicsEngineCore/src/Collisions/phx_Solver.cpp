#include "phx_Solver.hpp"
#include "phx_Body.hpp"
#include "Colliders/phx_Collider.hpp"
#include "phx_Manifolds.hpp"

#include <cmath>
#include <algorithm>

namespace Phx{

constexpr float SLOP       = 0.01f;
constexpr float BAUMGARTE  = 0.4f;
constexpr float REST_THRESH = 0.5f;   // ниже этого vn упругость обнуляется
constexpr float TANGENT_EPS = 1e-4f;  // порог «нет касательной скорости»


void seperateBodies(Manifold& manifold){

    Body* bodyA = manifold.bodyA;
    Body* bodyB = manifold.bodyB;

    const float inv_mass_sum = bodyA->inv_mass + bodyB->inv_mass;  
    
    if(inv_mass_sum > 0.f){
        float depth = std::max(manifold.penetration - SLOP, 0.f);
        const Vec2 corr = (depth * BAUMGARTE / inv_mass_sum) * manifold.normal;

        if (!bodyA->is_static) bodyA->position = bodyA->position - corr * bodyA->inv_mass;
        if (!bodyB->is_static) bodyB->position = bodyB->position + corr * bodyB->inv_mass;
    }

}


void applyImpulse(Body& b, const Vec2& r,const Vec2& impulse){
    if(b.is_static) 
        return;
    
    b.velocity = b.velocity + b.inv_mass*impulse;
    b.angle_velocity = b.angle_velocity + b.inv_inertia * cross2d(r, impulse);
}


Vec2 pointVelocity(Body& b, const Vec2& r){
    return b.velocity + Vec2(-b.angle_velocity * r.y, 
                            b.angle_velocity * r.x);
}



void Solver::resolveCollision(Manifold& manifold) 
{
    //========== DEFINES AND CHECKS EXCEPTION ===================

    Body* bodyA = manifold.bodyA;
    Body* bodyB = manifold.bodyB;
 
    if(bodyA->inv_mass == 0 && bodyB->inv_mass == 0)
        return;

    
    //================== SEPERATE BODIES FROM EACH OTHER ============================
    
    seperateBodies(manifold);

    // ================ NORMAL IMPULSE ================
 

    const Vec2 normal = manifold.normal; //from A -> B

    Vec2 rA = manifold.contactPointA - bodyA->position;
    Vec2 rB = manifold.contactPointB - bodyB->position;


    Vec2 v_rel = pointVelocity(*bodyB, rB) - pointVelocity(*bodyA, rA);
    float v_n = dot(v_rel, normal);

    if(v_n > 0.f) return;

    const float invEffMassN = bodyA->inv_mass + bodyB->inv_mass 
    + cross2d(rA, normal) * cross2d(rA, normal) * bodyA->inv_inertia
    + cross2d(rB, normal) * cross2d(rB, normal) * bodyB->inv_inertia;
    
    if (invEffMassN <= 0.f) 
        return;


    float e = std::max(bodyA->material.restitution, bodyB->material.restitution);
    if(std::abs(v_n) < REST_THRESH) 
        e = 0.f;

    const float j_n = -(1 + e) * v_n / invEffMassN;
    Vec2 normal_impulse = j_n * normal;

    applyImpulse(*bodyA, rA, -1 * normal_impulse);
    applyImpulse(*bodyB, rB, normal_impulse);

    //==============TANGENT IMPULS(Friction) =================

    v_rel = pointVelocity(*bodyB, rB) - pointVelocity(*bodyA, rA);
    Vec2 tangent = v_rel - dot(v_rel, normal) * normal; 

    float t_len = length(tangent);

    if(t_len < TANGENT_EPS) 
        return;

    tangent = tangent/t_len;

    const float invEffMassT = bodyA->inv_mass + bodyB->inv_mass
            + cross2d(rA, tangent) * cross2d(rA, tangent) * bodyA->inv_inertia
            + cross2d(rB, tangent) * cross2d(rB, tangent) * bodyB->inv_inertia;


    if(invEffMassT <= 0.f) 
        return;
    
    float v_t = dot(v_rel, tangent);

    float j_t = -v_t / invEffMassT;
    
    // Конус Кулона: |jt| ≤ μ · |jn|
    const float mu = std::sqrt(bodyA->material.friction * bodyB->material.friction);
    const float maxFriction = mu * std::abs(j_n);

    j_t = std::clamp(j_t, -maxFriction, maxFriction);

    const Vec2 frictionImpulse = tangent * j_t;

    applyImpulse(*bodyA, rA, -1 * frictionImpulse);
    applyImpulse(*bodyB, rB,  frictionImpulse);

}


}