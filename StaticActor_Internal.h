#pragma once
#include "Actor.h"
#include "Actor_Internal.h"
#include "Transform.h"
#include "StaticActor.h"


namespace Horizon
{
	class StaticActor_Internal : public Actor_Internal, public StaticActor
	{
	public:
		StaticActor_Internal() {};

		void Reset() override;

	private:


	};
}

