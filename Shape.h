#pragma once
#include "Transform.h"

namespace Horizon
{
	class Shape
	{
	public:
		virtual	Transform GetLocalPose() const = 0;
		virtual	void SetLocalPose(const Transform& pose) = 0;
	};
}

