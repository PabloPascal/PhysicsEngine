#include "calculate_collisions.hpp"
#include "phx_matrix.hpp"
#include <iostream>
#include <algorithm>


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

bool AABBcheckCollision(Rect& r1, Rect& r2)
{
    float r1_x_min = r1.get_position().x - r1.get_size().x / 2;
    float r1_x_max = r1.get_position().x + r1.get_size().x / 2;
 
    float r1_y_min = r1.get_position().y - r1.get_size().y / 2;
    float r1_y_max = r1.get_position().y + r1.get_size().y / 2;


    float r2_x_min = r2.get_position().x - r2.get_size().x / 2;
    float r2_x_max = r2.get_position().x + r2.get_size().x / 2;
 
    float r2_y_min = r2.get_position().y - r2.get_size().y / 2;
    float r2_y_max = r2.get_position().y + r2.get_size().y / 2;


    if(r1_x_min < r2_x_max && r2_x_min < r1_x_max){
        if(r1_y_min < r2_y_max && r2_y_min < r1_y_max)
            return true;
    }


    return false;

}


bool SATcheckCollision(Rect& r1, Rect& r2, float& diff)
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


    for(auto axis : allAxis)
    {
        float min1, max1, min2, max2;
        
        FindProjection(axis, min1, max1, r1_vertices);
        FindProjection(axis, min2, max2, r2_vertices);

        if(max1 < min2 || max2 < min1){
            diff = 0;
            return false;
        }

        diff = std::abs(min1 - min2);

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



Vec2 FindNormal(Rect& rect1, Rect& rect2, float& diff)
{
    Vec2 C1 = rect1.get_position();
    Vec2 C2 = rect2.get_position();

    float w1 = rect1.get_size().x/2.f;
    float h1 = rect1.get_size().y/2.f;
    float w2 = rect2.get_size().x/2.f;
    float h2 = rect2.get_size().y/2.f;

    float left1 =   C1.x - w1;
    float right1 =  C1.x + w1;
    float top1 =    C1.y - h1;
    float bottom1 = C1.y + h1;
    
    float left2 =   C2.x - w2;
    float right2 =  C2.x + w2;
    float top2 =    C2.y - h2;
    float bottom2 = C2.y + h2;
    

    if(top1 < bottom2 && (left1 < left2 || right1 > right2))
    {
        diff = std::abs(top1 - bottom2);
        return {0,1};
    }
    if(bottom1 > top2 && (left1 < left2 || right1 > right2))
    {
        diff = std::abs(top1 - bottom2);
        return {0,-1};
    }
    if(right1 > left2 && (bottom1 > top2 || bottom2 > top1))
    {
        return {1,0};
    }
    if(left1 < right2 && (bottom1 > top2 || bottom2 > top1))
    {
        return {-1,0};
    }




    return {0,0};
}



Vec2 FindNormal(Rect& rect, Vec2 point)
{
        float x_b = rect.get_position().x;
        float y_b = rect.get_position().y;

        float w = rect.get_size().x;
        float h = rect.get_size().y;

        if (point.x > x_b - w / 2 && point.x < x_b + w / 2) {

            if (point.y - y_b >= 0) return { 0, 1 };
            else return { 0, -1 };
        }

        if (point.y > y_b - h && point.y < y_b + h) {
            if (point.x - x_b > 0) return { 1.f,0.f };
            else                   return {-1.f,0.f };
        }

        return {0,0};

}


void solveAABBCollision(Rect& r1, Rect& r2)
{
    if(AABBcheckCollision(r1, r2)){
        float diff;
        Vec2 n = FindNormal(r1, r2, diff);

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



bool CircleRectCheckCollision(Circle& circle, Rect& rect, Vec2& normal, Vec2& contactPoint, float& penetration)
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

    contactPoint = rect.get_position() + transpose(rect.get_transform()) * closestPoint;

    if(dist <= r * r)
        return true;
    else    
        return false;
}




} // namespace 


