#include "pch.h"
#include "BoxShape_Internal.h"

using namespace Horizon;



Vec3 BoxShape_Internal::GetHalfExtents() const
{
    return m_halfExtents;
}

void BoxShape_Internal::SetHalfExtents(const Vec3& extents)
{
    m_halfExtents = extents;
}

Mat33 BoxShape_Internal::CalculateInertiaDiagonal(float mass) const
{
    float w = m_halfExtents.x;
    float h = m_halfExtents.y;
    float d = m_halfExtents.z;

    float mult = mass / 12.0f;

    float ix = mult * (h * h + d * d);
    float iy = mult * (w * w + d * d);
    float iz = mult * (w * w + h * h);

    return Mat33::Diagonal(ix, iy, iz);
}

float BoxShape_Internal::CalculateVolume() const
{
    return 8.0f * m_halfExtents.x * m_halfExtents.y * m_halfExtents.z;
}
