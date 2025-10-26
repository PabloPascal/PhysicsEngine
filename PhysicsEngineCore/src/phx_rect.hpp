#ifndef RECTANGLE_HPP
#define RECTANGLE_HPP


#include <phx_vector.hpp>
#include <phx_matrix.hpp>


namespace Phx{

class Rect{

public:
    Rect();
    Rect(Vec2 position, Vec2 size);
    Rect(Vec2 position, Vec2 size, float mass);

    inline Vec2 get_position() const {return m_position;}
    inline Vec2 get_velocity() const {return m_velocity;}
    inline float get_mass() const {return m_mass;}
    inline Vec2 get_size() const {return {m_width, m_height};}
    inline Vec2 get_normal1() const {return n1;}
    inline Vec2 get_normal2() const {return n2;}
    const Vec2* const get_vertices();
    inline Matrix2 get_transform() {return transform;}
    inline float get_elasiticy() const {return m_elasticity;}
    inline bool get_collision_indicate() const {return b_collsion;}
    inline bool get_gravity_indicate() const {return b_gravity;}
    inline Vec2 get_acceleration() const {return m_acceleration;}
    inline float get_angle() const {return m_angle;}
    inline float get_angle_speed() const {return m_angle_speed;}
    inline float get_angle_acceleration() const {return m_angle_acceleration;}
    inline float get_inertia() const {return m_inertia;}
    inline float get_friction() {return m_friction;}

    inline void set_velocity(Vec2 velocity) {m_velocity = velocity;}
    inline void set_position(Vec2 position) {m_position = position; }
    inline void set_mass(float mass) {
        m_mass = mass;
        m_inertia = 1/12 * m_mass * (m_width * m_width + m_height * m_height);
    }
    inline void set_elasticity(float elasticity) {m_elasticity = elasticity;} 
    inline void set_friction(float friction) {m_friction = friction;} 

    void set_rotate(float angle);
    inline void set_collision_indicate(bool turn) {b_collsion = turn;}
    inline void set_gravity_indicate(bool turn) {b_gravity = turn;}
    inline void set_acceleration(Vec2 acceleration) {m_acceleration = acceleration;} 
    inline void set_angle(float angle) {m_angle = angle;}
    inline void set_angle_speed(float angle_speed) {m_angle_speed = angle_speed;}
    inline void set_angle_acceleration(float angle_acceleration) {m_angle_acceleration = angle_acceleration;}



    void update(float dt);
private:

    float m_width;
    float m_height;
    float m_mass;
    float m_inertia;

    Vec2 m_position;
    Vec2 m_velocity;
    Vec2 m_acceleration = {0, 500.f};
    Vec2 force;

    float m_angle;
    float m_angle_speed;
    float m_angle_acceleration;

    float m_elasticity = 1;
    float m_friction;

    Vec2 n1 = {1, 0};
    Vec2 n2 = {0, 1};
    Matrix2 transform;
    
    bool b_collsion;
    bool b_gravity;

    Vec2 vertices[4];


};

}

#endif
