#include "calculate_collisions.hpp"
#include "phx_matrix.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <printf.h>

namespace Phx{


bool CollisionSolver::checkBallsCollision(const Circle& circleA, const Circle& circleB)
{
        float r1 = circleA.get_radius();
        float r2 = circleB.get_radius();


        if (length(circleA.get_position() - circleB.get_position()) < r1 + r2) {
            return true;
        }

        return false;
}

void CollisionSolver::resolveCircleCollision(Circle& circleA, Circle& circleB) 
{
        Vec2 xA = circleA.get_position();
        Vec2 xB = circleB.get_position();

        Vec2 normal = (xB - xA) / length(xB-xA); 

        Vec2 velocityA = circleA.get_velocity();
        Vec2 velocityB = circleB.get_velocity();

        float massA = circleA.get_mass();
        float massB = circleB.get_mass();

        if(massA == 0 || massB == 0) std::cout << "zero mass!!!" << std::endl;

        float Mass1 = 2 * massB / (massA + massB);
        float Mass2 = 2 * massA / (massA + massB);

        float j = -(1 + circleA.get_elasticity() * circleB.get_elasticity() ) * dot(velocityA - velocityB, normal) 
        / (1/massA + 1/massB);


        Vec2 vA_new = velocityA + normal * (j / massA);
        Vec2 vB_new = velocityB - normal * (j / massB);

        circleA.set_velocity( vA_new );
        circleB.set_velocity( vB_new );
}


void CollisionSolver::separateBalls(Circle& circleA, Circle& circleB) 
{

    float totalMass = circleA.get_mass() + circleB.get_mass();

    float ratioA = circleA.get_mass() / totalMass;
    float ratioB = circleB.get_mass() / totalMass;
    

    float angle = std::atan2(circleA.get_position().y - circleB.get_position().y,
                circleA.get_position().x - circleB.get_position().x);

    float radiusA = circleA.get_radius();
    float radiusB = circleA.get_radius();

    float slop = 0.01;
    float diffDist = radiusA + radiusB - length(circleA.get_position() - circleB.get_position()) - slop;

    diffDist *= 0.5;

    Vec2 dir = { std::cos(angle), std::sin(angle) };
        
    Vec2 dir1 = dir + circleA.get_velocity();
    Vec2 dir2 = dir*(-1) + circleB.get_velocity();

    Vec2 delta1 = circleA.get_position() + dir * diffDist * ratioA;
    Vec2 delta2 = circleB.get_position() + dir * -diffDist * ratioB;

    circleA.set_position(delta1);
    circleB.set_position(delta2);


}


void CollisionSolver::separateCircleWalls(Circle& c, Vec2 normal, float diff) {

    Vec2 delta = c.get_position() + normal * diff;

    c.set_position(delta);

}




void CollisionSolver::boundaryCollision(Circle& c, const Vec2 border)
{

        float r = c.get_radius();

        Vec2 position = c.get_position();
        Vec2 velocity = c.get_velocity();


        if (position.x + r > border.x) {
            float diff = position.x + r - border.x;

            separateCircleWalls(c, { -1, 0 }, diff);
            c.set_velocity({ -velocity.x, velocity.y });
        }

        if (position.x - r < 0) {
            float diff = -(position.x - r);

            separateCircleWalls(c, { 1, 0 }, diff);
            c.set_velocity({ -velocity.x, velocity.y });
        }

        if (position.y + r > border.y) {
            float diff = position.y + r - border.y;

            separateCircleWalls(c, { 0, -1 }, diff);
            c.set_velocity({ velocity.x, -velocity.y * c.get_elasticity() });

        }
        if (position.y - r < 0) {
            float diff = -(position.y - r);

            separateCircleWalls(c, { 0, 1 }, diff);
            c.set_velocity({ velocity.x, -velocity.y });

        }


}

bool CollisionSolver::AABBcheckCollision(Rect& rectA, Rect& rectB, Vec2& normal, float& penetrate)
{
    float Ax_min = rectA.get_position().x - rectA.get_size().x / 2;
    float Ax_max = rectA.get_position().x + rectA.get_size().x / 2;
 
    float Ay_min = rectA.get_position().y - rectA.get_size().y / 2;
    float Ay_max = rectA.get_position().y + rectA.get_size().y / 2;


    float Bx_min = rectB.get_position().x - rectB.get_size().x / 2;
    float Bx_max = rectB.get_position().x + rectB.get_size().x / 2;
 
    float By_min = rectB.get_position().y - rectB.get_size().y / 2;
    float By_max = rectB.get_position().y + rectB.get_size().y / 2;

    
    
    float overlap_right = Ax_max - Bx_min;  //n1
    float overlap_left = Bx_max - Ax_min;   //-n1
    float overlap_top = Ay_max - By_min;    //n2
    float overlap_bottom = By_max - Ay_min;  //-n2
    
    Vec2 n1 = rectA.get_normal1();
    Vec2 n2 = rectA.get_normal2();

    float min_overlap = std::min({overlap_left, overlap_right, overlap_bottom, overlap_top});

    penetrate = min_overlap;

    if(min_overlap == overlap_right) normal = -1*n1;
    else if(min_overlap == overlap_left) normal = n1;
    else if(min_overlap == overlap_top) normal = -1 * n2;
    else if(min_overlap == overlap_bottom) normal = n2;



    if(Ax_min < Bx_max && Bx_min < Ax_max){
        if(Ay_min < By_max && By_min < Ay_max)
            return true;
    }


    return false;

}


bool CollisionSolver::SATcheckCollision(Rect& rectA, Rect& rectB, Vec2& normal, Vec2& contact_point, float& penetrate)
{

    Vec2 allAxis[4] = {rectA.get_normal1(), rectA.get_normal2(), rectB.get_normal1(), rectB.get_normal2()};

    std::vector<Vec2> Avertices = {
        rectA.get_vertices()[0],
        rectA.get_vertices()[1],
        rectA.get_vertices()[2],
        rectA.get_vertices()[3]};


    std::vector<Vec2> Bvertices = {
        rectB.get_vertices()[0],
        rectB.get_vertices()[1],
        rectB.get_vertices()[2],
        rectB.get_vertices()[3]};



    penetrate = std::numeric_limits<float>::max();
    
    for(auto axis : allAxis)
    {
        float min1, max1, min2, max2;
        
        FindProjection(axis, min1, max1, Avertices);
        FindProjection(axis, min2, max2, Bvertices);

        float overlap = std::min(max1, max2) - std::max(min1, min2);
        

        if(max1 < min2 || max2 < min1){
            penetrate = 0;
            return false;
        }

        if(overlap < penetrate)
        {
            penetrate = overlap;
            normal = axis;
        }

    }

    Vec2 center_dir = (rectB.get_position() - rectA.get_position()).normalize();
    
   std::vector<Vec2> vertex_inside;

    for(auto vert : Avertices)
    {
        if(checkPointInsidePolygon(vert, rectB))
        {
            vertex_inside.push_back(vert);
        }
    }

    for(auto vert : Bvertices)
    {
        if(checkPointInsidePolygon(vert, rectA))
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

    contact_point = sum;

    if(dot(center_dir, normal) > 0)
    {
        normal = -1 * normal;
    }

    return true;

}



Vec2 CollisionSolver::FindContactPoint(Rect& rectA, Rect& rectB)
{

    std::vector<Vec2> vertex_inside;

    for(short i = 0; i < 4; i++)
    {
        if(checkPointInsidePolygon(rectA.get_vertices()[i], rectB));
        {
            vertex_inside.push_back(rectA.get_vertices()[i]);
        }
    }

    for(short i = 0; i < 4; i++)
    {
        if(checkPointInsidePolygon(rectB.get_vertices()[i], rectA));
        {
            vertex_inside.push_back(rectB.get_vertices()[i]);
        }
    }

    Vec2 sum;

    for(auto vert : vertex_inside)
    {
        sum = sum + vert;
    }

    if(!vertex_inside.empty())
        sum = sum / vertex_inside.size();



    return sum;

}


void CollisionSolver::FindProjection(Vec2 Axis, float& minProj, float& maxProj, std::vector<Vec2>& vertices)
{
    minProj = maxProj = dot(Axis, vertices[0]);

    for(auto v : vertices)
    {
        float projection = dot(Axis, v);
        if(projection > maxProj) maxProj = projection;
        else if(projection < minProj) minProj = projection;
    }

}




void CollisionSolver::resolveAABBCollision(Rect& rectA, Rect& rectB)
{
    Vec2 n;
    float diff = 0;
    if(AABBcheckCollision(rectA, rectB, n, diff)){

        Vec2 v1 = rectA.get_velocity();
        Vec2 v2 = rectB.get_velocity();

        rectA.set_position(rectA.get_position() + n * diff  );
        rectB.set_position(rectB.get_position() + n * -diff  );


    }

}


void CollisionSolver::resolveCircleRectCollsion(Circle& circle, Rect& rect)
{
    Vec2 n;
    float depth = 0;
    Vec2 contact_point;
    if(CircleRectCheckCollision(circle, rect, n, contact_point, depth))
    {
        Vec2 center_mass1 = rect.get_position();
        Vec2 center_mass2 = circle.get_position();
        Vec2 v1 = rect.get_velocity();
        Vec2 v2 = circle.get_velocity();
        float m1 = rect.get_mass();
        float m2 = circle.get_mass();
        float w1 = rect.get_angle_speed();
        float w2 = circle.get_angle_speed();
        float I1 = rect.get_inertia();
        float I2 = circle.get_inertia();

        Vec2 lever1 = contact_point - center_mass1;
        Vec2 lever2 = contact_point - center_mass2;  

        if(m1 == 0 || m2 == 0) std::cout << "zero mass!!!" << std::endl;

        float inv_m1 = 1/m1;
        float inv_m2 = 1/m2;

        float slop = 0.01;
        float percent = 0.2;

        float clamp = std::max(depth - slop, 0.f);
        Vec2 correction = (clamp * percent) / (inv_m1 + inv_m2) * n; 

        rect.set_position(rect.get_position() - correction * inv_m1);
        circle.set_position(circle.get_position() + correction * inv_m2);

        Vec2 crossVW1( -w1 * lever1.y, w1 * lever1.x);
        Vec2 crossVW2( -w2 * lever2.y, w2 * lever2.x); 

        Vec2 Vcontact1 = v1 + crossVW1;
        Vec2 Vcontact2 = v2 + crossVW2;

        Vec2 v_rel = Vcontact1 - Vcontact2;

        float V_n = dot( v_rel, n );

        float perp_l1 = cross2d(lever1, n);
        float perp_l2 = cross2d(lever2, n);

        float mass_eff =  1 / (inv_m1 + inv_m2 + (perp_l1 * perp_l1) / I1 +  (perp_l2 * perp_l2) / I2 );
        float elasticity = std::min(rect.get_elasiticy(), circle.get_elasticity());

        float normal_impulse = -(1 + elasticity) * V_n * mass_eff;
        Vec2 impulse = normal_impulse * n;

        v1 = v1 + impulse / m1;
        v2 = v2 - impulse / m2;

        w1 = w1 + cross2d(lever1, impulse) / I1;
        w2 = w2 - cross2d(lever2, impulse) / I2; 


        rect.set_velocity( v1 );
        circle.set_velocity( v2 );

        rect.set_angle_speed(w1);
        circle.set_angle_speed(w2);
        
    }



}



bool CollisionSolver::CircleRectCheckCollision(Circle& circle, Rect& rect, Vec2& normal, Vec2& contact_point, float& penetration)
{

    Vec2 center = circle.get_position();
    float r = circle.get_radius();

    float half_w = rect.get_size().x / 2.f;
    float half_h = rect.get_size().y / 2.f;

    Vec2 n1 = rect.get_normal1();
    Vec2 n2 = rect.get_normal2();

    Vec2 dv = center - rect.get_position();
    Vec2 local = {dot(dv, n1), dot(dv, n2)};
    Vec2 closestPoint;
   
    //clamping
    closestPoint.x = std::max(-half_w, std::min(dot(dv, n1), half_w));
    closestPoint.y = std::max(-half_h, std::min(dot(dv, n2), half_h));


    Vec2 dir = {local.x - closestPoint.x, local.y - closestPoint.y};

    float dist = dot(dir, dir); //square of dist
    Vec2 local_norm = dir.normalize();
    normal = transpose(rect.get_transform()) * local_norm; 

    penetration = r - std::sqrt(dist);  
    
    Matrix2 transform(n1, n2);

    //inverse matrix for ortogonal matrix is transpone matrix
    normal = transform * local_norm; 

    contact_point = rect.get_position() + transform * closestPoint;

    if(dist <= r * r)
        return true;
    else    
        return false;
}



void CollisionSolver::resolveRectsCollision(Rect& rectA, Rect& rectB)
{

    Vec2 n;
    float depth;
    Vec2 contact_point;

    if(SATcheckCollision(rectA, rectB, n, contact_point, depth))
    {   

        Vec2 center_mass1 = rectA.get_position();
        Vec2 center_mass2 = rectB.get_position();
        Vec2 v1 = rectA.get_velocity();
        Vec2 v2 = rectB.get_velocity();
        float m1 = rectA.get_mass();
        float m2 = rectB.get_mass();
        float w1 = rectA.get_angle_speed();
        float w2 = rectB.get_angle_speed();
        float I1 = rectA.get_inertia();
        float I2 = rectB.get_inertia();

        Vec2 lever1 = contact_point - center_mass1;
        Vec2 lever2 = contact_point - center_mass2;  

        if(m1 == 0 || m2 == 0) std::cout << "zero mass!!!" << std::endl;

        float inv_m1 = 1/m1;
        float inv_m2 = 1/m2;

        float slop = 0.01;
        float percent = 0.2;

        float clamp = std::max(depth - slop, 0.f);
        Vec2 correction = (clamp * percent) / (inv_m1 + inv_m2) * n; 

        rectA.set_position(rectA.get_position() + correction * inv_m1);
        rectB.set_position(rectB.get_position() - correction * inv_m2);

        Vec2 crossVW1( -w1 * lever1.y, w1 * lever1.x);
        Vec2 crossVW2( -w2 * lever2.y, w2 * lever2.x); 

        Vec2 Vcontact1 = v1 + crossVW1;
        Vec2 Vcontact2 = v2 + crossVW2;

        Vec2 v_rel = Vcontact1 - Vcontact2;

        float V_n = dot( v_rel, n );

        float perp_l1 = cross2d(lever1, n);
        float perp_l2 = cross2d(lever2, n);

        float mass_eff =  1 / (inv_m1 + inv_m2 + (perp_l1 * perp_l1) / I1 +  (perp_l2 * perp_l2) / I2 );
        float elasticity = std::min(rectA.get_elasiticy(), rectB.get_elasiticy());

        float normal_impulse = -(1 + elasticity) * V_n * mass_eff;
        Vec2 impulse = normal_impulse * n;

        v1 = v1 + impulse / m1;
        v2 = v2 - impulse / m2;

        w1 = w1 + cross2d(lever1, impulse) / I1;
        w2 = w2 - cross2d(lever2, impulse) / I2; 


        rectA.set_velocity( v1 );
        rectB.set_velocity( v2 );

        rectA.set_angle_speed(w1);
        rectB.set_angle_speed(w2);



    }


}


void CollisionSolver::boundaryCollision(Rect& r, const Vec2 border)
{
    Vec2 n1(1,0);
    Vec2 n2(0,1);

    Vec2 center = r.get_position();
    float w = r.get_size().x;
    float h = r.get_size().y;

    std::vector<Vec2> vertices = {
        r.get_vertices()[0],
        r.get_vertices()[1],
        r.get_vertices()[2],
        r.get_vertices()[3]
    };


    float minX, maxX, minY, maxY;
    FindProjection(n1, minX, maxX, vertices);
    FindProjection(n2, minY, maxY, vertices);


    float depth = 0;
    if(minX < 0)
    {
        r.set_velocity({0,0});
        depth = minX;
        r.set_position(center + Vec2(-depth, 0));
    }
    if(maxX > border.x)
    {
        r.set_velocity({0,0});
        depth = maxX - border.x;
        r.set_position(center + Vec2(-depth, 0));
    }
    if(minY < 0)
    {
        r.set_velocity({0,0});
        depth = minY;
        r.set_position(center + Vec2(0, -depth));
    }
    if(maxY > border.y)
    {
        r.set_velocity({ r.get_velocity().x, -r.get_velocity().y * r.get_elasiticy() });
        depth = maxY - border.y;
        r.set_position(center + Vec2(0, -depth));
    }

}



bool CollisionSolver::checkPointInsidePolygon(Vec2 point, Rect& rect)
{
    
    Vec2 n1 = {1, 0};
    Vec2 n2 = {0, 1};

    std::vector<Vec2> vertices = {
        rect.get_vertices()[0],
        rect.get_vertices()[1],
        rect.get_vertices()[2],
        rect.get_vertices()[3]
    };

    float minX, maxX, minY, maxY;
    FindProjection(n1, minX, maxX, vertices);
    FindProjection(n2, minY, maxY, vertices);

    if(minX < point.x && maxX > point.x){
        if(minY < point.y && maxY > point.y) return true;
    }


    return false;

}



} // namespace 


