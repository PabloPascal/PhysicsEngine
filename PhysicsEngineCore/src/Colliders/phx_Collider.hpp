#ifndef PHX_COLLIDER 
#define PHX_COLLIDER 
#include "Geometry/phx_aabb.hpp"

namespace Phx{

class Body;

enum class ColliderType{
    Circle, 
    Rect
};


class Collider{

public:

    virtual AABB computeAABB(const Body& body) const = 0; 
    
    virtual ColliderType type() const = 0;

    virtual ~Collider() = default;

};


};

#endif 