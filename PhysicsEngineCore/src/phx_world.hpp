#ifndef PHYSIC_WORLD_HPP
#define PHYSIC_WORLD_HPP

#include <phx_circle.hpp>
#include <phx_rect.hpp>

#include <vector>
#include <memory>
#include <iostream>
#include <functional>


namespace Phx
{


class PhysicsWorld
{

    float m_Gravity;
    Vec2 m_border;

    std::vector<std::shared_ptr<Circle>> circles;
    std::vector<std::shared_ptr<Rect>> rectangles;

public:
    PhysicsWorld(Vec2 border);

    void add_circle(Circle* circle);
    void add_circle(std::shared_ptr<Circle> circle);
    void add_rect(std::shared_ptr<Rect> rect);
    void delete_object();

    void generate_circles(size_t count);
    void update(float dt);

    std::vector<std::shared_ptr<Circle>>& get_circles()
    {
        return circles;
    }

    std::vector<std::shared_ptr<Rect>>& get_rects()
    {
        return rectangles;
    }

};


}

#endif
