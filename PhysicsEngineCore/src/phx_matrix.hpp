#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <phx_vector.hpp>
#include <iostream>

namespace Phx{

class Matrix2{

public:

    float a11, a12, a21, a22;



    Matrix2(float _a11, float _a12, float _a21, float _a22) : 
    a11(_a11), a12(_a12), a21(_a21), a22(_a22) 
    {}

    Matrix2(Vec2 v1, Vec2 v2) : 
    a11(v1.x), a21(v1.y), a12(v2.x), a22(v2.y)
    {}

    Matrix2(const Matrix2& mat){
        a11 = mat.a11;
        a12 = mat.a12;
        a21 = mat.a21;
        a22 = mat.a22;
    }


    Matrix2 operator*(float scalar){
        
        return Matrix2(a11 * scalar, a12 * scalar,
                      a21 * scalar, a22 * scalar);

    }


    Matrix2 operator/(float scalar){
        
        return Matrix2(a11 / scalar, a12 / scalar,
                      a21 / scalar, a22 / scalar);

    }


    Matrix2 operator*(const Matrix2& mat){
        return Matrix2(
            a11 * mat.a11 + a12 * mat.a21, 
            a11 * mat.a12 + a12 * mat.a22, 
            a21 * mat.a11 + a22 * mat.a21,
            a21 * mat.a12 + a22 * mat.a22
        );
    }


    Matrix2 operator+(const Matrix2& mat){
        return Matrix2( 
            a11 + mat.a11, 
            a12 + mat.a12, 
            a21 + mat.a21,
            a22 + mat.a22
        );
    }


    Matrix2 operator-(const Matrix2& mat){
        return Matrix2( 
            a11 - mat.a11, 
            a12 - mat.a12, 
            a21 - mat.a21,
            a22 - mat.a22
        );
    }

    Matrix2& operator=(const Matrix2& mat){
        a11 = mat.a11;
        a12 = mat.a12;
        a21 = mat.a21;
        a22 = mat.a22;
        return *this;
    }

    Vec2 operator*(const Vec2& v){
        return Vec2(
            a11 * v.x + a12 * v.y, 
            a21 * v.x + a22 * v.y
        );
    }




};


static Matrix2 transpose(const Matrix2& mat){

    return Matrix2(
        mat.a11,  mat.a21,
        mat.a12, mat.a22
    );

}


static Matrix2 inverse(const Matrix2& mat){

    float det = mat.a11 * mat.a22 - mat.a12*mat.a21;

    if(det == 0){
        std::cerr << "invalid matrix with det = 0\n";
        return Matrix2(0,0,0,0);
    }

    float inv_det = 1/det;
    return Matrix2(mat.a22, -mat.a12,
                  -mat.a21, mat.a11) * inv_det;


}


}//namespace

#endif