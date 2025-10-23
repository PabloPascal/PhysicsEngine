#ifndef CALC_COLLISION_H
#define CALC_COLLISION_H

#include <phx_circle.hpp>
#include <phx_rect.hpp>
#include <vector>

namespace Phx{


bool checkBallsCollision(const Circle& c1, const Circle& c2);

void hitBalls(Circle& circle1, Circle& circle2);

void separateBalls(Circle& c1, Circle& c2);

void boundaryCollision(Circle& c, const Vec2 border);

void separateCircleWalls(Circle& c1, Vec2 normal, float diff);

bool AABBcheckCollision(Rect& r1, Rect& r2);

bool SATcheckCollision(Rect& r1, Rect& r2, float& diff);

void FindProjection(Vec2 Axis, float& minProj, float& maxProj, std::vector<Vec2>& vertices);

void solveAABBCollision(Rect& r1, Rect& r2);

void CircleVsRectCollsion(Circle& circle, Rect& rect);

/*
    check collision with SAT algorithm
*/
bool CircleRectCheckCollision(Circle& circle, Rect& rect, Vec2& normal, Vec2& contactPoint, float& penetration);

Vec2 FindNormal(Rect& rect, Vec2 point);

Vec2 FindNormal(Rect& rect1, Rect& rect2, float& diff);



}

#endif //CALC_COLLISION_H