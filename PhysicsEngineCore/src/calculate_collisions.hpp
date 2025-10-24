#ifndef CALC_COLLISION_H
#define CALC_COLLISION_H

#include <phx_circle.hpp>
#include <phx_rect.hpp>
#include <vector>

namespace Phx{


bool checkBallsCollision(const Circle& c1, const Circle& c2);

void resolveCircleCollision(Circle& circle1, Circle& circle2);

void separateBalls(Circle& c1, Circle& c2);

void boundaryCollision(Circle& c, const Vec2 border);
void boundaryCollision(Rect& r, const Vec2 border);


void separateCircleWalls(Circle& c1, Vec2 normal, float diff);

bool AABBcheckCollision(Rect& r1, Rect& r2, Vec2& normal, float& penetrate);

bool SATcheckCollision(Rect& r1, Rect& r2, Vec2& normal, Vec2& contact_point, float& penetration);

void resolveRectsCollision(Rect& r1, Rect& r2);

void FindProjection(Vec2 Axis, float& minProj, float& maxProj, std::vector<Vec2>& vertices);

void solveAABBCollision(Rect& r1, Rect& r2);

void resolveCircleRectCollsion(Circle& circle, Rect& rect);

/*
    check collision with SAT algorithm
*/
bool CircleRectCheckCollision(Circle& circle, Rect& rect, Vec2& normal, Vec2& contactPoint, float& penetration);

bool checkPointInsidePolygon(Vec2 point, Rect& rect);
}

#endif //CALC_COLLISION_H