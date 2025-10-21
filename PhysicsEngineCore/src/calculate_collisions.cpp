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


bool SATcheckCollision(Rect& r1, Rect& r2)
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
            return false;
        }

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


void solveRectCollision(Rect& r1, Rect& r2)
{

    if(SATcheckCollision(r1, r2)){

 

    }

}



}


