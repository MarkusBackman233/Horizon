#pragma once
#include "Vec3.h"
#include "Quat.h"

namespace Horizon
{
	class Transform
	{
	public:
		Transform(){}
		Transform(const Vec3& _pos) : pos(_pos){}
		Transform(const Quat& _quat) : quat(_quat){}
		Transform(const Vec3& _pos, const Quat& _quat) : pos(_pos), quat(_quat){}

		Transform operator*(const Transform& other) const
		{
			Vec3 worldPos = pos + quat.RotateVector(other.pos);
			Quat worldRot = quat * other.quat;
			return Transform(worldPos, worldRot);
		}

		Vec3 pos;
		Quat quat;
	};
}