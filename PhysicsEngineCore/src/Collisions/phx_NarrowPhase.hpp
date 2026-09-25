#pragma once 



namespace Phx{

class Body;
class Manifold;

class NarrowPhase{

public:

    bool collision(const Body& bodyA, const Body& bodyB, Manifold& manifold);

private:
    bool checkCirclesCollision(const Body& circleA, const Body& circleB);

    bool checkRectsCollision(const Body& rectA, const Body& rectB, Manifold& manifold);

    bool checkCircleRectCollision(const Body& circle, const Body& rect, Manifold& manifold);

};


}