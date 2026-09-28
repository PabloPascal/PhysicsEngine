#pragma once 



namespace Phx{

class Body;
class Manifold;

class NarrowPhase{

public:

    bool collision(Body& bodyA, Body& bodyB, Manifold& manifold);

};


}