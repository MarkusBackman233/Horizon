#include "pch.h"
#include "Actor_Internal.h"
#include "BoxShape_Internal.h"
using namespace Horizon;

Actor_Internal::Actor_Internal()
{
}

void Actor_Internal::SetPose(const Transform& pose)
{
	m_pose = pose;
}

void Actor_Internal::Reset()
{
}

Shape* Actor_Internal::AddShape(ShapeType shapeType)
{
	Shape* shape = nullptr;

	switch (shapeType)
	{
	case ShapeType::Box:
		m_shapes.emplace_back(std::make_unique<BoxShape_Internal>());
		shape = m_shapes.back().get();
		dynamic_cast<BoxShape*>(shape)->SetHalfExtents(Vec3(0.5f,0.5f,0.5f));
		break;
	default:
		break;
	}

	return shape;
}

const std::vector<Shape*> Actor_Internal::GetShapes()
{
	std::vector<Shape*> shapes;

	for (auto& shape : m_shapes)
	{
		shapes.push_back(shape.get());
	}
	return shapes;
}



Transform Actor_Internal::GetPose() const
{
	return m_pose;
}

