#include "phx_rect.hpp"
#include <phx_matrix.hpp>
#include <iostream>

namespace Phx
{


Rect::Rect() : 
m_width(0), 
m_height(0), 
m_mass(0), 
m_position{0,0}, 
m_velocity{0,0}
{
    for(short i = 0; i < 4; i++)
        vertices[i] = Vec2(0,0);
}



Rect::Rect(Vec2 position, Vec2 size, float mass) : 
m_position(position), m_width(size.x), m_height(size.y),
m_mass(mass), m_velocity{0,0}
{
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
m_velocity{0,0}
{

}


void Rect::set_rotate(float rad){

    angle = rad;

    transform.a11 =  std::cos(angle);
    transform.a12 = -std::sin(angle);
    transform.a21 =  std::sin(angle);
    transform.a22 =  std::cos(angle);

    n1 = transform * n1;
    n2 = transform * n2;

}




void Rect::update(float dt){

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