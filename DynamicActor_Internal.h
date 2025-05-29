#pragma once
#include "DynamicActor.h"
#include "Actor_Internal.h"
namespace Horizon
{
	class DynamicActor_Internal : public virtual DynamicActor, public virtual Actor_Internal
	{
	public:
		DynamicActor_Internal() {};

		void Reset() override;

		Vec3 GetLinearVelocity() const override;
		Vec3 GetAngularVelocity() const override;

		void SetLinearVelocity(const Vec3& velocity);
		void SetAngularVelocity(const Vec3& velocity);


		float GetMassInv() const;
		float GetMass() const override;
		void SetMass(float mass) override;

		void Integrate(float deltaTime);

		Shape* AddShape(ShapeType shapeType) override;
		Mat33 GetInertiaTensor() const;

		void CalculateInertiaTensor();

	private:
		Vec3 m_linearVelocity;
		Vec3 m_angularVelocity;

		Mat33 m_inertiaTensor;

		float m_mass;
		float m_massInv;


	};
}



