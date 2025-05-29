#include "pch.h"
#include "DynamicActor_Internal.h"
#include "Shape_Internal.h"
#undef max

using namespace Horizon;


void DynamicActor_Internal::Reset()
{
	m_linearVelocity = Vec3(0.0f);
	m_angularVelocity = Vec3(0.0f);
	m_mass = 1.0f;
	m_massInv = 1.0f;
}

Vec3 DynamicActor_Internal::GetLinearVelocity() const
{
	return m_linearVelocity;
}

Vec3 DynamicActor_Internal::GetAngularVelocity() const
{
	return m_angularVelocity;
}

void DynamicActor_Internal::SetLinearVelocity(const Vec3& velocity)
{
	m_linearVelocity = velocity;
}

void DynamicActor_Internal::SetAngularVelocity(const Vec3& velocity)
{
	m_angularVelocity = velocity;
}

float DynamicActor_Internal::GetMassInv() const
{
	return m_massInv;
}

float DynamicActor_Internal::GetMass() const
{
	return m_mass;
}

void DynamicActor_Internal::SetMass(float mass)
{
	m_mass = mass;
	m_massInv = 1.0f / mass;
}

void DynamicActor_Internal::Integrate(float deltaTime)
{
	const float linearDamping = 0.02f;
	const float angularDamping = 0.05f;

	const float linearDampingMultiplier = std::max(1.0f - linearDamping * deltaTime, 0.0f);
	const float angularDampingMultiplier = std::max(1.0f - angularDamping * deltaTime, 0.0f);

	m_linearVelocity.y -= 9.81f * deltaTime;

	m_linearVelocity *= linearDampingMultiplier;
	m_angularVelocity *= angularDampingMultiplier;
	
	m_pose.pos += m_linearVelocity * deltaTime;

	float w = m_angularVelocity.Length();
	if (w != 0.0f)
	{
		w = sqrt(w);
		const float maxW = 1e+7f;
		if (w > maxW)
		{
			m_angularVelocity = m_angularVelocity * maxW;
			w = maxW;
		}
		const float v = deltaTime * w * 0.5f;

		float s = sin(v);
		float q = cos(v);

		s /= w;

		const Vec3 pqr = m_angularVelocity * s;
		const Quat quatVel(pqr.x, pqr.y, pqr.z, 0);
		Quat result = quatVel * m_pose.quat;

		result += m_pose.quat * q;
		result.Normalize();
		m_pose.quat = result;
	}
}

Shape* DynamicActor_Internal::AddShape(ShapeType shapeType)
{
	auto shape = Actor_Internal::AddShape(shapeType);
	SetMass(GetMass()); // update inertia tensor
	return shape;
}

Mat33 DynamicActor_Internal::GetInertiaTensor() const
{
	return m_inertiaTensor;
}

void DynamicActor_Internal::CalculateInertiaTensor()
{

	m_inertiaTensor = Horizon::Mat33::Zero();

	std::vector<float> volumes;

	float totalVolume = 0.0f;

	for (auto& shape : m_shapes)
	{
		float volume = dynamic_cast<Shape_Internal*>(shape.get())->CalculateVolume();
		volumes.push_back(volume);
		totalVolume += volume;
	}

	for (size_t i = 0; i < m_shapes.size(); i++)
	{

		float shapeMass = (volumes[i] / totalVolume) * m_mass;
		Transform shapePose = m_shapes[i]->GetLocalPose();
		//Mat33 inertiaDiag = dynamic_cast<Shape_Internal*>(m_shapes[i].get())->CalculateInertiaDiagonal(shapeMass);

		//// Rotate Inertia
		//Mat33 R = (/*GetPose().quat * */shapePose.quat).ToMat33();
		//Mat33 worldInertia = R * inertiaDiag * R.Transpose();
		//
		//Horizon::Vec3 r = shapePose.pos /* - centerOfMass*/;
		//Horizon::Mat33 shift;
		//{ // parallelAxis
		//	float x2 = r.x * r.x;
		//	float y2 = r.y * r.y;
		//	float z2 = r.z * r.z;
		//
		//	shift = Mat33::Diagonal(
		//		shapeMass * (y2 + z2),
		//		shapeMass * (x2 + z2),
		//		shapeMass * (x2 + y2)
		//	) - Mat33(
		//		0, r.x * r.y, r.x * r.z,
		//		r.y * r.x, 0, r.y * r.z,
		//		r.z * r.x, r.z * r.y, 0
		//	) * shapeMass;
		//}
		//m_inertiaTensor += worldInertia + shift;




		// Get local inertia
		Mat33 localInertia = dynamic_cast<Shape_Internal*>(m_shapes[i].get())
			->CalculateInertiaDiagonal(shapeMass);

		// Rotate local shape inertia into actor space
		shapePose.quat.w = 0.0f;
		Mat33 R = shapePose.quat.ToMat33();
		Mat33 rotatedInertia = R * localInertia * R.Transpose();

		// Parallel axis theorem
		Vec3 offset = shapePose.pos;
		float d2 = offset.Dot(offset);
		Mat33 parallelAxis = Mat33::Identity() * d2 - Mat33::OuterProduct(offset, offset);

		rotatedInertia += parallelAxis * shapeMass;

		// Accumulate
		m_inertiaTensor += rotatedInertia;
	}
}
