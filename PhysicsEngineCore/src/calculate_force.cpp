
#include "calculate_force.hpp"
#include <iostream>

namespace Phx{


void calcGravityForce(Circle& c1, Circle& c2) {

        constexpr float Gravity = 1000;

        float m1 = c1.get_mass();
        float m2 = c2.get_mass();

        float d = length(c1.get_position() - c2.get_position());

        Vec2 dir_12 = c1.get_position() - c2.get_position();

        Vec2 F12 = dir_12 * (Gravity * m1 * m2 / (d * d)) / length(dir_12);

        Vec2 u1_prev = c1.get_velocity();
        Vec2 u2_prev = c2.get_velocity();

        Vec2 u1_next = (F12 * (-1.f / 100.f) + u1_prev * m1) / m1;
        Vec2 u2_next = (F12 * (1.f / 100.f) + u2_prev * m2) / m2;

        c1.set_velocity(u1_next);
        c2.set_velocity(u2_next);

}

void gravity(Circle &c, float gravity,  float dt)
{
        float vx = c.get_velocity().x;
        float vy = c.get_velocity().y;

        vy += gravity * dt;
        c.set_velocity({ vx, vy });
}



}