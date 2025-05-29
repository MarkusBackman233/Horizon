#pragma once
#include "Actor.h"
#ifdef HORIZON_EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif


namespace Horizon
{
    extern "C" {
        PHYSICS_API void InitializePhysics();
        PHYSICS_API Actor* CreateDynamicActor(const Transform& pose);
        PHYSICS_API Actor* CreateStaticActor(const Transform& pose);
        PHYSICS_API void Simulate(float deltaTime);
    }
}

