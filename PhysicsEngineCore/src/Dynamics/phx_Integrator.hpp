#pragma once 

namespace Phx{

class Body;

class Integrator{

public:

    void solver(Body& body, float dt);

};


}


