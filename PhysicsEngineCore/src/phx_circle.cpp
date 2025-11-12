#include "phx_circle.hpp"
#include <iostream>

namespace Phx{

Circle::Circle() : 
 m_center({0, 0}),
 m_mass(0), 
 m_elasticity(0), 
 m_radius(0), 
 m_velocity({0,0}),
 b_gravity(false),
 b_collision(false),
 b_move(false),
 b_force_gravity(false),
 b_bound_collision(false),
 b_static(false)
{}

Circle::Circle(float pos_x, float pos_y, float radius) :
 m_center{pos_x, pos_y},
 m_radius(radius), 
 m_velocity{0, 0},
 m_mass(0),
 m_elasticity(0),
 b_gravity(false),
 b_collision(false),
 b_move(false),
 b_force_gravity(false),
 b_bound_collision(false),
 b_static(false)

{}

Circle::Circle(Phx::Vec2 position, float radius) :
 m_center(position),
 m_radius(radius), 
 m_velocity{0, 0},
 m_mass(0),
 m_elasticity(0),
 m_angle_speed(0),
 b_gravity(false),
 b_collision(false),
 b_move(false),
 b_force_gravity(false),
 b_bound_collision(false),
  b_static(false)

{}


Circle::Circle(Vec2 position, float radius, float mass) :
 m_center(position),
 m_radius(radius), 
 m_velocity{0, 0},
 m_mass(mass),
 m_elasticity(0),
 m_angle_speed(0),
 b_gravity(false),
 b_collision(false),
 b_move(false),
 b_force_gravity(false),
 b_bound_collision(false),
 b_static(false)

{
    m_inertia = 1/2.f * m_mass * (m_radius * m_radius);
}



void Circle::update(float dt)
{  
    m_angle = m_angle + m_angle_speed * dt;
    m_velocity = m_velocity + m_acceleration * dt;
    m_center = m_center + m_velocity * dt;
}


}