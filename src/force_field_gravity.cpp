#include "force_field_gravity.h"

void ForceFieldGravity::apply(RigidBody &r) const
{
    r.force_ += GRAVITY_ * r.mass_;
}
