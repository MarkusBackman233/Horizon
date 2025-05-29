#include "pch.h"
#include "Shape_Internal.h"

using namespace Horizon;

Transform Shape_Internal::GetLocalPose() const
{
    return m_localPose;
}

void Shape_Internal::SetLocalPose(const Transform& pose)
{
    m_localPose = pose;
    m_isChanged = true;
}

bool Shape_Internal::IsChanged() const
{
    return m_isChanged;
}

void Shape_Internal::ClearChanged()
{
    m_isChanged = false;
}
