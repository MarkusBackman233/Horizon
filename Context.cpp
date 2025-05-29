#include "pch.h"
#include "Context.h"
#include "DynamicActor_Internal.h"
#include "StaticActor_Internal.h"
using namespace Horizon;

Actor* Horizon::Context::CreateDynamicActor(const Transform& pose)
{
	m_dynamicActors.push_back(DynamicActor_Internal());
	DynamicActor_Internal* actor = &m_dynamicActors.back();
	actor->SetPose(pose);
	//actor->SetAngularVelocity(Vec3(1.0f, 1.0f, 0.0f));
	actor->SetMass(10.0f);
	return actor;
}

Actor* Horizon::Context::CreateStaticActor(const Transform& pose)
{
	m_staticActors.push_back(StaticActor_Internal());
	StaticActor_Internal* actor = &m_staticActors.back();
	actor->SetPose(pose);
	return &m_staticActors.back();
}

Context::Context()
{
	m_dynamicActors.reserve(1000);
	m_staticActors.reserve(1000);
}
