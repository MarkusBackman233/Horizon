#pragma once
#include <stdexcept>
#include "Vec3.h"
namespace Horizon
{
    class Mat33
    {
    public:
        float m[3][3]; // Row-major: m[row][col]
    
        Mat33() 
        {
            m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f;
            m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f;
            m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f;
        }
    
        Mat33(float m00, float m01, float m02,
              float m10, float m11, float m12,
              float m20, float m21, float m22) {
            m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
            m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
            m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
        }
    
        static Mat33 Identity() {
            return Mat33();
        }

        static Mat33 Zero() {
            Mat33 m;
            m.m[0][0] = 0.0f;
            m.m[1][1] = 0.0f;
            m.m[2][2] = 0.0f;
            return m;
        }
    
        static Mat33 Diagonal(float x, float y, float z) {
            return Mat33(
                x, 0, 0,
                0, y, 0,
                0, 0, z
            );
        }
    
        Mat33 Transpose() const {
            return Mat33(
                m[0][0], m[1][0], m[2][0],
                m[0][1], m[1][1], m[2][1],
                m[0][2], m[1][2], m[2][2]
            );
        }
    
        // Matrix multiplication
        Mat33 operator*(const Mat33& other) const {
            Mat33 result;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    result.m[r][c] = m[r][0] * other.m[0][c] +
                    m[r][1] * other.m[1][c] +
                    m[r][2] * other.m[2][c];
            return result;
        }
        // In-place addition
        inline Mat33& operator+=(const Mat33& other) {
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    m[r][c] += other.m[r][c];
            return *this;
        }

        // Matrix + Matrix
        inline Mat33 operator+(const Mat33& other) const {
            Mat33 result;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    result.m[r][c] = m[r][c] + other.m[r][c];
            return result;
        }

        inline Mat33 operator*(float scalar) {
            Mat33 result;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    result.m[r][c] = scalar * m[r][c];
            return result;
        }

        inline Mat33 operator-(const Mat33& other) const {
            Mat33 result;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    result.m[r][c] = m[r][c] - other.m[r][c];
            return result;
        }

        float Determinant() const {
            return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        }

        static Mat33 OuterProduct(const Vec3& a, const Vec3& b)
        {
            return Mat33(
                a.x * b.x, a.x * b.y, a.x * b.z,
                a.y * b.x, a.y * b.y, a.y * b.z,
                a.z * b.x, a.z * b.y, a.z * b.z
            );
        }

        Mat33 Inverse() const {
            float det = Determinant();

            if (det == 0.0f) {
                throw std::runtime_error("Matrix is singular and cannot be inverted.");
            }

            Mat33 adjugate;
            adjugate.m[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
            adjugate.m[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
            adjugate.m[0][2] = m[0][1] * m[1][2] - m[0][2] * m[1][1];

            adjugate.m[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
            adjugate.m[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
            adjugate.m[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];

            adjugate.m[2][0] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
            adjugate.m[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
            adjugate.m[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];

            Mat33 inverse;
            float invDet = 1.0f / det;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    inverse.m[i][j] = adjugate.m[i][j] * invDet;

            return inverse;
        }

        Vec3 operator*(const Vec3& v) const
        {
            return Vec3(
                m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, 
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z, 
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z  
            );
        }

        // Print matrix (for debugging)
        //void Print() const {
        //    for (int r = 0; r < 3; ++r) {
        //        std::cout << "[ ";
        //        for (int c = 0; c < 3; ++c)
        //            std::cout << m[r][c] << " ";
        //        std::cout << "]\n";
        //    }
        //}

    };
}
