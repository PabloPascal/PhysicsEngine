#include "phx_circle.hpp"


namespace Phx{

Circle::Circle() : 
 m_center({0, 0}),
 m_mass(0), 
 m_elasticity(0), 
 m_radius(0), 
 m_velocity({0,0}),
 b_gravity(false),
 b_collision(false),
 b_move(false)
{}

Circle::Circle(float pos_x, float pos_y, float radius) :
 m_center{pos_x, pos_y},
 m_radius(radius), 
 m_velocity{0, 0},
 m_mass(0),
 m_elasticity(0),
 b_gravity(false),
 b_collision(false),
b_move(false)

{}

Circle::Circle(Phx::Vec2 position, float radius) :
 m_center(position),
 m_radius(radius), 
 m_velocity{0, 0},
 m_mass(0),
 m_elasticity(0),
 b_gravity(false),
 b_collision(false),
b_move(false)

{}


}