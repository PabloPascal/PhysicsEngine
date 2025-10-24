#include "calculate_collisions.hpp"
#include "phx_matrix.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>

namespace Phx{


bool checkBallsCollision(const Circle& c1, const Circle& c2)
{
        float r1 = c1.get_radius();
        float r2 = c2.get_radius();


        if (length(c1.get_position() - c2.get_position()) < r1 + r2) {
            return true;
        }

        return false;
}

void hitBalls(Circle& circle1, Circle& circle2) 
{
        Vec2 x1 = circle1.get_position();
        Vec2 x2 = circle2.get_position();

        Vec2 normal = (x2 - x1) / length(x2-x1); 

        Vec2 v1 = circle1.get_velocity();
        Vec2 v2 = circle2.get_velocity();

        float m1 = circle1.get_mass();
        float m2 = circle2.get_mass();

        if(m1 == 0 || m2 == 0) std::cout << "zero mass!!!" << std::endl;

        float Mass1 = 2 * m2 / (m1 + m2);
        float Mass2 = 2 * m1 / (m1 + m2);

        float j = -(1 + circle1.get_elasticity() * circle2.get_elasticity() ) * dot(v1 - v2, normal) / (1/m1 + 1/m2);


        Vec2 v1_new = v1 + normal * (j / m1);
        Vec2 v2_new = v2 - normal * (j / m2);

        circle1.set_velocity( v1_new );
        circle2.set_velocity( v2_new );
}


void separateBalls(Circle& c1, Circle& c2) 
{

    float totalMass = c1.get_mass() + c2.get_mass();

    float ratioA = c1.get_mass() / totalMass;
    float ratioB = c2.get_mass() / totalMass;
    

    float angle = std::atan2(c1.get_position().y - c2.get_position().y,
                c1.get_position().x - c2.get_position().x);

    float r1 = c1.get_radius();
    float r2 = c2.get_radius();

    float slop = 0.01;
    float diffDist = r1 + r2 - length(c1.get_position() - c2.get_position()) - slop;

    diffDist *= 0.5;

    Vec2 dir = { std::cos(angle), std::sin(angle) };
        
    Vec2 dir1 = dir + c1.get_velocity();
    Vec2 dir2 = dir*(-1) + c2.get_velocity();

    Vec2 delta1 = c1.get_position() + dir * diffDist * ratioA;
    Vec2 delta2 = c2.get_position() + dir * -diffDist * ratioB;

    c1.set_position(delta1);
    c2.set_position(delta2);


}


void separateCircleWalls(Circle& c, Vec2 normal, float diff) {

        //diff *= 0.5;

        Vec2 delta = c.get_position() + normal * diff;

        c.set_position(delta);

}




void boundaryCollision(Circle& c, const Vec2 border)
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

bool AABBcheckCollision(Rect& r1, Rect& r2, Vec2& normal, float& penetrate)
{
    float r1_x_min = r1.get_position().x - r1.get_size().x / 2;
    float r1_x_max = r1.get_position().x + r1.get_size().x / 2;
 
    float r1_y_min = r1.get_position().y - r1.get_size().y / 2;
    float r1_y_max = r1.get_position().y + r1.get_size().y / 2;


    float r2_x_min = r2.get_position().x - r2.get_size().x / 2;
    float r2_x_max = r2.get_position().x + r2.get_size().x / 2;
 
    float r2_y_min = r2.get_position().y - r2.get_size().y / 2;
    float r2_y_max = r2.get_position().y + r2.get_size().y / 2;

    
    
    float overlap_right = r1_x_max - r2_x_min;  //n1
    float overlap_left = r2_x_max - r1_x_min;   //-n1
    float overlap_top = r1_y_max - r2_y_min;    //n2
    float overlap_bottom = r2_y_max - r1_y_min;  //-n2
    
    Vec2 n1 = r1.get_normal1();
    Vec2 n2 = r2.get_normal2();

    float min_overlap = std::min({overlap_left, overlap_right, overlap_bottom, overlap_top});

    penetrate = min_overlap;

    if(min_overlap == overlap_right) normal = -1*n1;
    else if(min_overlap == overlap_left) normal = n1;
    else if(min_overlap == overlap_top) normal = -1 * n2;
    else if(min_overlap == overlap_bottom) normal = n2;



    if(r1_x_min < r2_x_max && r2_x_min < r1_x_max){
        if(r1_y_min < r2_y_max && r2_y_min < r1_y_max)
            return true;
    }


    return false;

}


bool SATcheckCollision(Rect& r1, Rect& r2, Vec2& normal, Vec2& contact_point, float& penetrate)
{

    Vec2 allAxis[4] = {r1.get_normal1(), r1.get_normal2(), r2.get_normal1(), r2.get_normal2()};

    std::vector<Vec2> r1_vertices = {
        r1.get_vertices()[0],
        r1.get_vertices()[1],
        r1.get_vertices()[2],
        r1.get_vertices()[3]};


    std::vector<Vec2> r2_vertices = {
        r2.get_vertices()[0],
        r2.get_vertices()[1],
        r2.get_vertices()[2],
        r2.get_vertices()[3]};


    penetrate = std::numeric_limits<float>::max();
    
    for(auto axis : allAxis)
    {
        float min1, max1, min2, max2;
        
        FindProjection(axis, min1, max1, r1_vertices);
        FindProjection(axis, min2, max2, r2_vertices);

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

    Vec2 center_dir = (r2.get_position() - r1.get_position()).normalize();
    
    std::vector<Vec2> vertex_inside;

    for(auto vert : r1_vertices)
    {
        if(checkPointInsidePolygon(vert, r2))
        {
            vertex_inside.push_back(vert);
        }
    }

    for(auto vert : r2_vertices)
    {
        if(checkPointInsidePolygon(vert, r1))
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

    // std::cout << "sum_x = " <<  sum.x << std::endl;
    // std::cout << "sum_y = " << sum.y << std::endl;

    contact_point = sum;

    // std::cout << "cp_x = " << contact_point.x << std::endl;
    // std::cout << "cp_y = " << contact_point.y << std::endl;


    if(dot(center_dir, normal) > 0)
    {
        normal = -1 * normal;
    }

    return true;

}



void FindProjection(Vec2 Axis, float& minProj, float& maxProj, std::vector<Vec2>& vertices)
{
    minProj = maxProj = dot(Axis, vertices[0]);

    for(auto v : vertices)
    {
        float projection = dot(Axis, v);
        if(projection > maxProj) maxProj = projection;
        else if(projection < minProj) minProj = projection;
    }

}




void solveAABBCollision(Rect& r1, Rect& r2)
{
    Vec2 n;
    float diff = 0;
    if(AABBcheckCollision(r1, r2, n, diff)){

        Vec2 v1 = r1.get_velocity();
        Vec2 v2 = r2.get_velocity();

        r1.set_position(r1.get_position() + n * diff  );
        r2.set_position(r2.get_position() + n * -diff  );


    }

}


void CircleVsRectCollsion(Circle& circle, Rect& rect)
{
    Vec2 n;
    float penetration = 0;
    Vec2 contact_point;
    if(CircleRectCheckCollision(circle, rect, n, contact_point, penetration))
    {
        Vec2 v = circle.get_velocity();
    
        //seperate
        circle.set_position(circle.get_position() + n * penetration);

        //change velocity
        if(dot(n, v) < 0)
        {
            circle.set_velocity((v + n * dot(v*(-1), n) * 2) * circle.get_elasticity());
        }
        
    }



}



bool CircleRectCheckCollision(Circle& circle, Rect& rect, Vec2& normal, Vec2& contact_point, float& penetration)
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



void resolveRectsCollision(Rect& r1, Rect& r2)
{

    Vec2 n;
    float depth;
    Vec2 contact_point;

    if(SATcheckCollision(r1, r2, n, contact_point, depth))
    {   

        Vec2 center_mass1 = r1.get_position();
        Vec2 center_mass2 = r2.get_position();
        Vec2 v1 = r1.get_velocity();
        Vec2 v2 = r2.get_velocity();
        float m1 = r1.get_mass();
        float m2 = r2.get_mass();
        float w1 = r1.get_angle_speed();
        float w2 = r2.get_angle_speed();
        float I1 = r1.get_inertia();
        float I2 = r2.get_inertia();

        Vec2 lever1 = contact_point - center_mass1;
        Vec2 lever2 = contact_point - center_mass2;  

        if(m1 == 0 || m2 == 0) std::cout << "zero mass!!!" << std::endl;

        float inv_m1 = 1/m1;
        float inv_m2 = 1/m2;

        float slop = 0.01;
        float percent = 0.2;

        float clamp = std::max(depth - slop, 0.f);
        Vec2 correction = (clamp * percent) / (inv_m1 + inv_m2) * n; 

        r1.set_position(r1.get_position() + correction * inv_m1);
        r2.set_position(r2.get_position() - correction * inv_m2);

        Vec2 crossVW1( -w1 * lever1.y, w1 * lever1.x);
        Vec2 crossVW2( -w2 * lever2.y, w2 * lever2.x); 

        Vec2 Vcontact1 = v1 + crossVW1;
        Vec2 Vcontact2 = v2 + crossVW2;

        Vec2 v_rel = Vcontact1 - Vcontact2;

        float V_n = dot( v_rel, n );

        float perp_l1 = cross2d(lever1, n);
        float perp_l2 = cross2d(lever2, n);

        float mass_eff =  1 / (inv_m1 + inv_m2 + (perp_l1 * perp_l1) / I1 +  (perp_l2 * perp_l2) / I2 );
        float elasticity = std::min(r1.get_elasiticy(), r2.get_elasiticy());

        float normal_impulse = -(1 + elasticity) * V_n * mass_eff;
        Vec2 impulse = normal_impulse * n;

        v1 = v1 + impulse / m1;
        v2 = v2 - impulse / m2;

        w1 = w1 + cross2d(lever1, impulse) / I1;
        w2 = w2 - cross2d(lever2, impulse) / I2; 


        r1.set_velocity( v1 );
        r2.set_velocity( v2 );

        r1.set_angle_speed(w1);
        r2.set_angle_speed(w2);



    }


}


void boundaryCollision(Rect& r, const Vec2 border)
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



bool checkPointInsidePolygon(Vec2 point, Rect& rect)
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


