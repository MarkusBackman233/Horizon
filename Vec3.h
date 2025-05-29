#pragma once
#include "pch.h"

#include <cmath>
namespace Horizon
{
	class Vec3
	{
	public:
		Vec3() : x(0.0f), y(0.0f), z(0.0f){}

		Vec3(float scalar) : x(scalar), y(scalar), z(scalar){}
		Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z){}

		float x, y, z;


		inline Vec3 operator +=(const Vec3& other) {
			x += other.x; y += other.y; z += other.z; return *this;
		} inline Vec3 operator +=(const float other) {
			x += other; y += other; z += other; return *this;
		} inline Vec3 operator + (const float other) const {
			return Vec3(x + other, y + other, z + other);
		} friend Vec3 operator +(float other, const Vec3& vec) {
			return Vec3(vec.x + other, vec.y + other, vec.z + other);
		} inline Vec3 operator +(const Vec3& other) {
			return Vec3(x + other.x, y + other.y, z + other.z);
		} friend Vec3 operator +(const Vec3& other, const Vec3& vec) {
			return Vec3(vec.x + other.x, vec.y + other.y, vec.z + other.z);
		};
		inline Vec3 operator -=(const Vec3& other) {
			x -= other.x; y -= other.y; z -= other.z; return *this;
		} inline Vec3 operator -=(const float other) {
			x -= other; y -= other; z -= other; return *this;
		} inline Vec3 operator - (const float other) const {
			return Vec3(x - other, y - other, z - other);
		} friend Vec3 operator -(float other, const Vec3& vec) {
			return Vec3(vec.x - other, vec.y - other, vec.z - other);
		} inline Vec3 operator -(const Vec3& other) {
			return Vec3(x - other.x, y - other.y, z - other.z);
		} friend Vec3 operator -(const Vec3& other, const Vec3& vec) {
			return Vec3(vec.x - other.x, vec.y - other.y, vec.z - other.z);
		};
		inline Vec3 operator *=(const Vec3& other) {
			x *= other.x; y *= other.y; z *= other.z; return *this;
		}

		inline Vec3 operator *=(const float other) {
			x *= other; y *= other; z *= other; return *this;
		}

		inline Vec3 operator * (const float other) const {
			return Vec3(x * other, y * other, z * other);
		}

		friend Vec3 operator *(float other, const Vec3& vec) {
			return Vec3(vec.x * other, vec.y * other, vec.z * other);
		}

		inline Vec3 operator *(const Vec3& other) const {
			return Vec3(x * other.x, y * other.y, z * other.z);
		}

		friend Vec3 operator *(const Vec3& lhs, const Vec3& rhs) {
			return Vec3(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
		}


		inline Vec3 operator /=(const Vec3& other) {
			x /= other.x; y /= other.y; z /= other.z; return *this;
		}

		inline Vec3 operator /=(const float other) {
			x /= other; y /= other; z /= other; return *this;
		}

		inline Vec3 operator / (const float other) const {
			return Vec3(x / other, y / other, z / other);
		}

		friend Vec3 operator /(float other, const Vec3& vec) {
			return Vec3(vec.x / other, vec.y / other, vec.z / other);
		}

		inline Vec3 operator /(const Vec3& other) const {
			return Vec3(x / other.x, y / other.y, z / other.z);
		}

		friend Vec3 operator /(const Vec3& other, const Vec3& vec) {
			return Vec3(vec.x / other.x, vec.y / other.y, vec.z / other.z);
		}
		inline Vec3 operator -()
		{
			x = -x;
			y = -y;
			z = -z;
			return *this;
		}
		inline bool operator == (const Vec3& A) const
		{
			return x == A.x && y == A.y && z == A.z;
		}
		inline bool operator != (const Vec3& A) const
		{
			return x != A.x || y != A.y || z != A.z;
		}

		float Length() const
		{
			return std::sqrt(LengthSquared());
		}

		float LengthSquared() const
		{
			return x * x + y * y + z * z;
		}

		Vec3 Cross(const Vec3& b)
		{
			return Vec3(
				y * b.z - z * b.y,
				z * b.x - x * b.z,
				x * b.y - y * b.x
			);
		}
		float Dot(const Vec3& b) const
		{
			return x * b.x + y * b.y + z * b.z;
		}

		//Vec3 operator*(const Mat33& mat) const {
		//	return Vec3(
		//		x * mat.m[0][0] + y * mat.m[1][0] + z * mat.m[2][0],
		//		x * mat.m[0][1] + y * mat.m[1][1] + z * mat.m[2][1],
		//		x * mat.m[0][2] + y * mat.m[1][2] + z * mat.m[2][2]
		//	);
		//}
	};

}