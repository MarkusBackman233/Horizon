#pragma once
#include "DynamicActor.h"
#include <memory>
namespace Horizon
{
	class Actor_Internal : public virtual Actor
	{
	public:
		Actor_Internal();


		Transform GetPose() const override;
		void SetPose(const Transform& pose) override;
		void Reset() override;


		Shape* AddShape(ShapeType shapeType) override;
		const std::vector<Shape*> GetShapes() override;

	protected:
		Transform m_pose;

		std::vector<std::unique_ptr<Shape>> m_shapes;
	};
}
