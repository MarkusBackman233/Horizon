#pragma once
#include "Vec3.h"
#include "Mat33.h"
#include <cmath>

namespace Horizon
{
	class Quat
	{
	public:
		Quat() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
		Quat(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}


		Quat(float angleRadians, const Vec3& unitAxis)
		{
			const float angle = angleRadians * 0.5f;
			float sin = std::sin(angle);

			x = unitAxis.x * sin;
			y = unitAxis.y * sin;
			z = unitAxis.z * sin;
		}


		Quat operator*(const Quat& other) const
		{
			return Quat(
				w * other.x + x * other.w + y * other.z - z * other.y,
				w * other.y - x * other.z + y * other.w + z * other.x,
				w * other.z + x * other.y - y * other.x + z * other.w,
				w * other.w - x * other.x - y * other.y - z * other.z
			);
		}

		Quat& operator*=(const Quat& other)
		{
			*this = *this * other;
			return *this;
		}

		Quat operator*(float scalar) const
		{
			return Quat(x * scalar, y * scalar, z * scalar, w * scalar);
		}

		friend Quat operator*(float scalar, const Quat& q)
		{
			return q * scalar;
		}

		Quat operator+(const Quat& other) const
		{
			return Quat(x + other.x, y + other.y, z + other.z, w + other.w);
		}		
		
		void operator+=(const Quat& other)
		{
			x += other.x;
			y += other.y; 
			z += other.z;
			w += other.w;
		}

		Vec3 RotateVector(const Vec3& v) const
		{
			Quat vQuat(v.x, v.y, v.z, 0);
			Quat qConj(-x, -y, -z, w);

			Quat rotated = *this * vQuat * qConj;
			return Vec3(rotated.x, rotated.y, rotated.z);
		}


		void Normalize()
		{
			float len = Length();
			if (len > 0.000001f)
			{
				float invLen = 1.0f / len;
				x *= invLen;
				y *= invLen;
				z *= invLen;
				w *= invLen;
			}
		}

		float Length() const
		{
			return std::sqrt(x * x + y * y + z * z + w * w);
		}

		Vec3 Axis() const
		{
			float sinAngle = std::sqrt(x * x + y * y + z * z);

			if (sinAngle < 0.0001f)
				return Vec3(1.0f, 0.0f, 0.0f); 

			return Vec3(x, y, z) / sinAngle;
		}

		Horizon::Mat33 ToMat33() const {
			float xx = x * x;
			float yy = y * y;
			float zz = z * z;
			float xy = x * y;
			float xz = x * z;
			float yz = y * z;
			float wx = w * x;
			float wy = w * y;
			float wz = w * z;

			Horizon::Mat33 m;
			m.m[0][0] = 1.0f - 2.0f * (yy + zz);
			m.m[0][1] = 2.0f * (xy - wz);
			m.m[0][2] = 2.0f * (xz + wy);

			m.m[1][0] = 2.0f * (xy + wz);
			m.m[1][1] = 1.0f - 2.0f * (xx + zz);
			m.m[1][2] = 2.0f * (yz - wx);

			m.m[2][0] = 2.0f * (xz - wy);
			m.m[2][1] = 2.0f * (yz + wx);
			m.m[2][2] = 1.0f - 2.0f * (xx + yy);

			return m;
		}

		float x, y, z, w;
	};

}