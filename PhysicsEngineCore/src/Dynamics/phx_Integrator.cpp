#include "phx_Integrator.hpp"
#include "phx_Body.hpp"


namespace Phx{

    void Integrator::solver(Body& body, float dt){

        body.velocity = body.velocity + body.force * body.inv_mass * dt;
        body.position = body.position + body.velocity*dt;
        
        body.angle_velocity += body.torque * body.inv_inertia * dt; 
        body.angle += body.angle_velocity * dt;

    }


}
