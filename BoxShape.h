#pragma once
#include "Shape.h"

namespace Horizon
{
	class BoxShape : public virtual Shape
	{
	public:
		virtual	Vec3 GetHalfExtents() const = 0;
		virtual	void SetHalfExtents(const Vec3& pose) = 0;
	};
}

