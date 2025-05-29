#pragma once
#include "Transform.h"
#include "Shape.h"
#include <vector>

namespace Horizon
{
	enum class ShapeType : uint8_t
	{
		Box
	};

	class Actor
	{
	public:
		virtual	Transform GetPose() const = 0;
		virtual	void SetPose(const Transform& pose) = 0;


		virtual	Shape* AddShape(ShapeType shapeType) = 0;
		virtual	const std::vector<Shape*> GetShapes() = 0;


		// Will remove current forces and velocity
		virtual	void Reset() = 0;
	};
}
