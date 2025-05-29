#pragma once

#include "Shape.h"
namespace Horizon
{
	class Shape_Internal : public virtual Shape
	{
	public:
	
		Transform GetLocalPose() const override;
		void SetLocalPose(const Transform& pose) override;

		virtual Mat33 CalculateInertiaDiagonal(float mass) const = 0;
		virtual float CalculateVolume() const = 0;

		bool IsChanged() const;
		void ClearChanged();

	private:
		Transform m_localPose;
		bool m_isChanged;
	};
}