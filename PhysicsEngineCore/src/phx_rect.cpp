#include "phx_rect.hpp"
#include <phx_matrix.hpp>
#include <iostream>

namespace Phx
{


Rect::Rect() : 
m_width(0), 
m_height(0), 
m_mass(0), 
b_collsion(false),
b_gravity(false),
m_angle(0),
m_acceleration(0),
m_angle_speed(0),
m_elasticity(0),
m_friction(0)
{
    for(short i = 0; i < 4; i++)
        vertices[i] = Vec2(0,0);
}



Rect::Rect(Vec2 position, Vec2 size, float mass) : 
m_position(position), 
m_width(size.x), 
m_height(size.y),
m_mass(mass), 
b_collsion(false),
b_gravity(false),
m_angle(0),
m_acceleration(0),
m_angle_speed(0),
m_elasticity(0),
m_friction(0)
{
    m_inertia = 1/12.f * m_mass * (m_width * m_width + m_height * m_height);

    Vec2 dir1(m_width/2, m_height/2);
    Vec2 dir2(m_width/2, -m_height/2);

    vertices[0] = Vec2(m_position - dir2);
    vertices[1] = Vec2(m_position + dir1);
    vertices[2] = Vec2(m_position + dir2);
    vertices[3] = Vec2(m_position - dir1);
}


Rect::Rect(Vec2 position, Vec2 size) : 
m_width(size.x), 
m_height(size.y), 
m_mass(0), 
m_position(position), 
b_collsion(false),
b_gravity(false),
m_angle(0),
m_acceleration(0),
m_angle_speed(0),
m_elasticity(0),
m_friction(0)
{
    
}


void Rect::set_rotate(float rad){

    m_angle = rad;

    transform.a11 =  std::cos(m_angle);
    transform.a12 = -std::sin(m_angle);
    transform.a21 =  std::sin(m_angle);
    transform.a22 =  std::cos(m_angle);

    n1 = transform * n1;
    n2 = transform * n2;

}




void Rect::update(float dt){


    m_angle = m_angle + m_angle_speed * dt;
    set_rotate(m_angle_speed * dt);
    m_angle_speed *= std::exp(-2.0f * dt);  // Экспоненциальное затухание

    m_velocity = m_velocity + m_acceleration * dt;
    m_position = m_position + m_velocity * dt;


}




const Vec2* const Rect::get_vertices()
{
    
    Vec2 dir1 = n1 * m_width / 2 + n2 * m_height / 2; 
    Vec2 dir2 = n1 * m_width / 2 - n2 * m_height / 2; 

    vertices[0] = m_position - dir2;
    vertices[1] = m_position + dir1;
    vertices[2] = m_position + dir2;
    vertices[3] = m_position - dir1;

    return vertices;
}


} //namespace