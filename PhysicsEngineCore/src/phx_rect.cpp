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


void Rect::set_rotate(float rad){

    Matrix2 rotor(std::cos(rad), -std::sin(rad), std::sin(rad), std::cos(rad));
    
    n1 = rotor * n1;
    n2 = rotor * n2;


    for(short i = 0; i < 4; i++)
        vertices[i] = rotor * (vertices[i] - m_position) + m_position;

}




void Rect::update(float dt){

    m_position = m_position + m_velocity * dt;
    for(short i = 0; i < 4; i++){
        vertices[i] = vertices[i] + m_velocity * dt;
    }


}


} //namespace