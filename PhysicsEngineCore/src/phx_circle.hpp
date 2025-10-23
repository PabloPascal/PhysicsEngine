
#ifndef CIRCLE_HPP
#define CIRCLE_HPP
#include <phx_vector.hpp>

namespace Phx{

class Circle{
public:

    Circle();
    Circle(float pos_x, float pos_y, float radius);
    Circle(Vec2 position, float radius);

    
    inline Vec2 get_position() const {return m_center;}
    inline Vec2 get_velocity() const {return m_velocity;}
    inline Vec2 get_acceleration() const {return m_acceleration;}

    inline float get_radius() const {return m_radius;}
    inline float get_mass() const {return m_mass;}
    inline float get_elasticity() const {return m_elasticity;}
    inline bool get_collision_indicate() const { return b_collision;}
    inline bool get_gravity_indicate() const { return b_gravity;}
    inline bool get_move_indicate() const {return b_move;}


    inline void set_position(const Vec2 position) {m_center = position; }
    inline void set_velocity(const Vec2 velocity) {m_velocity = velocity;}
    inline void set_mass(const float mass) {m_mass = mass;}
    inline void set_elasticity(const float elasticity) {m_elasticity = elasticity;}
    inline void set_gravity_on(bool gravity) {b_gravity = gravity;}
    inline void set_collision_on(bool collision) {b_collision = collision;}
    inline void set_move_on(bool move) {b_move = move;}
    inline void set_acceleration(Vec2 acceleration) {m_acceleration = acceleration;}

    void update(float dt);
private:
    Vec2 m_center;
    float m_radius;
    float m_mass;
    float m_elasticity;
    Vec2 m_velocity;
    Vec2 m_acceleration;

    bool b_gravity;
    bool b_move;
    bool b_collision;

};

}


#endif 