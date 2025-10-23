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

    inline void set_velocity(Vec2 velocity) {m_velocity = velocity;}
    inline void set_position(Vec2 position) {m_position = position; }
    inline void set_mass(float mass) {m_mass = mass;}
    void set_rotate(float angle);

    void update(float dt);
private:

    float m_width;
    float m_height;
    float m_mass;
    Vec2 m_position;
    Vec2 m_velocity;
    float angle;


    Vec2 n1 = {1, 0};
    Vec2 n2 = {0, 1};
    Matrix2 transform;
    

    Vec2 vertices[4];


};

}

#endif
