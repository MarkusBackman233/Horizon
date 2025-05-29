#pragma once
#include "Actor.h"
#include "Transform.h"

namespace Horizon
{
	class DynamicActor : public virtual Actor
	{
	public:
		virtual	Vec3 GetLinearVelocity() const = 0;
		virtual	Vec3 GetAngularVelocity() const = 0;

		virtual	float GetMass() const = 0;
		virtual	void SetMass(float mass) = 0;
	};
}

