#include "pch.h"
#include "Horizon.h"
#include <iostream>
#include "Context.h"
#include "BoxShape_Internal.h"
#undef max
using namespace Horizon;

void Horizon::InitializePhysics()
{
}

Actor* Horizon::CreateDynamicActor(const Transform& pose)
{
	return  Context::Instance().CreateDynamicActor(pose);
}

Actor* Horizon::CreateStaticActor(const Transform& pose)
{
	return  Context::Instance().CreateStaticActor(pose);
}


void Horizon::Simulate(float deltaTime)
{
	auto& actors = Context::Instance().GetDynamicActors();
	for (auto& actor : actors)
	{

		for (Shape* shape : actor.GetShapes())
		{
			if (dynamic_cast<Shape_Internal*>(shape)->IsChanged())
			{
				actor.CalculateInertiaTensor();
				dynamic_cast<Shape_Internal*>(shape)->ClearChanged();
			}
		}

		Transform pose = actor.GetPose();
		Vec3 linear = actor.GetLinearVelocity();
		Vec3 angular = actor.GetAngularVelocity();

		float biggestInpactVelocity = 1.0f;
		Vec3 biggestInpactPos(0,1,0);
		const Vec3 inpactNormal(0.0f, 1.0f, 0.0f);



		for (Shape* shape : actor.GetShapes())
		{
			if (auto boxShape = dynamic_cast<BoxShape_Internal*>(shape))
			{
				for (size_t i = 0; i < 8; i++)
				{
					float signX = (i & 1) ? 1.0f : -1.0f;
					float signY = (i & 2) ? 1.0f : -1.0f;
					float signZ = (i & 4) ? 1.0f : -1.0f;

					Vec3 localCorner = Vec3(
						signX * boxShape->GetHalfExtents().x,
						signY * boxShape->GetHalfExtents().y,
						signZ * boxShape->GetHalfExtents().z
					);

					Vec3 corner = (pose * shape->GetLocalPose() * Transform(localCorner)).pos;

					if (corner.y < 0.0f)
					{

						Vec3 r = corner - pose.pos; // should be cmass instead


						Vec3 velocity0 = linear + angular.Cross(r);
						Vec3 velocity1(0.0f);

						const Vec3 relativeVelocity = velocity0 - velocity1;
						const float normalRelativeVelocity = inpactNormal.Dot(relativeVelocity) ;
						if (normalRelativeVelocity < biggestInpactVelocity && normalRelativeVelocity < -1e-6f)
						{
							biggestInpactPos = corner;
							biggestInpactVelocity = normalRelativeVelocity;
						}
					}
				}
			}
		}


		if (biggestInpactPos.y < 0.0f)
		{
			pose.pos.y += -biggestInpactPos.y;
			actor.SetPose(pose);
		}


		//if (biggestInpactPos.y < 0.01f)
		//{
		//	float friction = 0.1f;  // Small friction coefficient
		//	Vec3 frictionForce = -linear * friction;
		//	frictionForce.y = 0.0f;
		//	actor.SetLinearVelocity(actor.GetLinearVelocity() + frictionForce);
		//}

		if (biggestInpactVelocity < 0.0f)
		{
			float sumRecipMass = actor.GetMassInv() /*+ invMass1*/; // add actor1

			const float jLin = -biggestInpactVelocity;
			const float normalResponse = (1.0f + 0.7f) * jLin;
			// inertiaInv = actor.GetInertiaTensor().Inverse();

			Mat33 Ibody = actor.GetInertiaTensor(); // local
			Mat33 R = actor.GetPose().quat.ToMat33(); // world rotation
			Mat33 inertiaInv = R * Ibody * R.Transpose(); // then inverse
			inertiaInv = inertiaInv.Inverse();

			Vec3 r = biggestInpactPos - pose.pos; // should be cmass instead

			Vec3 cross = r.Cross(inpactNormal);
			Vec3 inertiaCross = inertiaInv * cross;
			float angularTerm = cross.Dot(inertiaCross);


			const float jImp = normalResponse / (sumRecipMass + angularTerm);

			Vec3 force = inpactNormal * jImp;
			Vec3 torque = r.Cross(force);

			Vec3 angularAcceleration = inertiaInv * torque;


			actor.SetLinearVelocity(actor.GetLinearVelocity() + force * actor.GetMassInv());
			actor.SetAngularVelocity(angular + angularAcceleration);
		}

		actor.SetPose(pose);
		actor.Integrate(deltaTime);

	}
}