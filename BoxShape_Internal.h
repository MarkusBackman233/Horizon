#pragma once
#include "Mat33.h"
#include "Shape_Internal.h"
#include "BoxShape.h"
namespace Horizon
{
	class BoxShape_Internal : public virtual BoxShape, public virtual Shape_Internal
	{
	public:
		BoxShape_Internal() {};

		Vec3 GetHalfExtents() const override;
		void SetHalfExtents(const Vec3& extents) override;

		Mat33 CalculateInertiaDiagonal(float mass) const override;
		float CalculateVolume() const override;

	private:
		Vec3 m_halfExtents;
	};
}