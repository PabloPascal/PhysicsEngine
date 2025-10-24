#include "phx_world.hpp"
#include <calculate_collisions.hpp>
#include <calculate_force.hpp>
#include <iostream>
#include <random>
#include <time.h>

namespace Phx
{

PhysicsWorld::PhysicsWorld(Vec2 border){

    m_Gravity = 1000;
    m_border = border;

}


void PhysicsWorld::update(float dt){

    for(auto circle : circles){
        circle->update(dt);
        boundaryCollision(*circle, m_border);
    }


    for(auto circle : circles){
        if(circle->get_gravity_indicate()){
            gravity(*circle, m_Gravity, dt);
            Vec2 new_pos = circle->get_position() + circle->get_velocity() * dt;
            circle->set_position(new_pos);
            boundaryCollision(*circle, m_border);
        }
    }

    for(size_t i = 0; i < circles.size(); i++){

        if(!circles[i]->get_collision_indicate())
            continue;

        for(size_t j = i+1; j < circles.size(); j++){
            if(checkBallsCollision(*circles[i], *circles[j])){
                separateBalls(*circles[i], *circles[j]);
                hitBalls(*circles[i], *circles[j]);
                
            }
        }

    }

    for(auto c : circles)
    {
        for(auto r : rectangles)
        {
            if(c->get_collision_indicate() && r->get_collision_indicate())
                CircleVsRectCollsion(*c, *r);
        }
    }

    for(auto r : rectangles)
    {
        r->update(dt);
        boundaryCollision(*r, m_border);
    }


    for(int i = 0; i < rectangles.size(); i++)
    {
        for(int j = i + 1; j < rectangles.size(); j++)
        {
            if(rectangles[i]->get_collision_indicate() && rectangles[j]->get_collision_indicate())
                resolveRectsCollision(*rectangles[i], *rectangles[j]);
        }
    }


}

void PhysicsWorld::add_circle(Circle* circle)
{
    if(circle->get_gravity_indicate())
        circle->set_acceleration({0, m_Gravity});
    circles.emplace_back(circle);
}


void PhysicsWorld::add_circle(std::shared_ptr<Circle> circle)
{
    circles.emplace_back(circle);
}

void PhysicsWorld::add_rect(std::shared_ptr<Rect> rect)
{
    rectangles.emplace_back(rect);
}



void PhysicsWorld::generate_circles(size_t count){
    
    std::srand(time(0));

    for(size_t i = 0; i < count; i++){
        //Vec2 pos = {(float)(rand() % (int)m_border.x),(float)(rand() % (int)m_border.y)};
        Vec2 pos = {50,50};
        float r = 3 + rand() % 5;
        std::shared_ptr<Circle> circle = std::make_shared<Circle>(pos, r);
        circle->set_mass(10 + rand() % 60);
        circle->set_velocity({150, 0});
        circle->set_collision_on(true);
        circle->set_gravity_on(true);
        circle->set_elasticity(0.8);
        circles.push_back(circle);
    }
}

}