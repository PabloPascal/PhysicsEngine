#ifndef CALC_COLLISION_H
#define CALC_COLLISION_H

#include <phx_circle.hpp>
#include <phx_rect.hpp>
#include <vector>

namespace Phx{

class CollisionSolver{
public:
    
    static bool checkBallsCollision(const Circle& cricleA, const Circle& circleB);

    static void resolveCircleCollision(Circle& circleA, Circle& circleB);

    static void separateBalls(Circle& circleA, Circle& circleB);

    static void boundaryCollision(Circle& c, const Vec2 border);
    static void boundaryCollision(Rect& r, const Vec2 border);

    static void separateCircleWalls(Circle& c1, Vec2 normal, float diff);

    static bool AABBcheckCollision(Rect& rectA, Rect& rectB, Vec2& normal, float& penetrate);

    /*
        check collision between rectangles, find normal, find contact_point, find penetration koef
    */
    static bool SATcheckCollision(Rect& rectA, Rect& rectB, Vec2& normal, Vec2& contact_point, float& penetration);
    static Vec2 FindContactPoint(Rect& rectA, Rect& rectB);

    static void resolveRectsCollision(Rect& rectA, Rect& rectB);
    /*
        find mininmum and maximum projection on axis
    */
    static void FindProjection(Vec2 Axis, float& minProj, float& maxProj, std::vector<Vec2>& vertices);
    /*
        resolve collision with no rotation rectangles
    */
    static void resolveAABBCollision(Rect& rectA, Rect& rectB);

    static void resolveCircleRectCollsion(Circle& circle, Rect& rect);

    /*
        check collision with SAT algorithm
    */
    static bool CircleRectCheckCollision(Circle& circle, Rect& rect, Vec2& normal, Vec2& contactPoint, float& penetration);

    static bool checkPointInsidePolygon(Vec2 point, Rect& rect);



};


}

#endif //CALC_COLLISION_H