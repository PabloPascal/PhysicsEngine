#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <cmath>


namespace Phx{

    class Vec2{
    public:

        float x, y;
        
    Vec2() : x(0), y(0)
    {}

    Vec2(float vx, float vy) : x(vx), y(vy)
    {}

    Vec2(const Vec2& vec){
        x = vec.x;
        y = vec.y;  
    }

    inline float getX(){return x;}
    inline float getY(){return y;}

    inline float getX() const{return x;}
    inline float getY() const{return y;}

    inline void setX(float vx) {x = vx;}
    inline void setY(float vy) {y = vy;}

    Vec2 operator=(const Vec2& vec){
        x = vec.x;
        y = vec.y;
        return *this;
    }

    Vec2 operator+(const Vec2& vec){
        return Vec2(x + vec.x, y + vec.y);
    }

    Vec2 operator-(const Vec2& vec){
        return Vec2(x - vec.x, y - vec.y);
    }

    Vec2 operator*(const Vec2& vec){
        return Vec2(x*vec.x, y*vec.y);
    }

    Vec2 operator*(const float scalar){
        return Vec2(scalar * x, scalar * y);
    }

    Vec2 operator/(const float scalar){
        return Vec2(x / scalar, y / scalar);
    }

    Vec2 normalize(){
        float len = length();
        x = x / len;
        y = y / len;
        return *this;
    }

    float length(){
        return std::sqrt(x*x + y*y);
    }

    };


    inline float dot(const Vec2& vec1, const Vec2& vec2){
        return vec1.x * vec2.x + vec1.y * vec2.y;
    }

    inline float length(const Vec2& vec){
        return std::sqrt(vec.x*vec.x + vec.y*vec.y);
    }


}


#endif