#include "force_field_drag.h"

ForceFieldDrag::ForceFieldDrag(float drag)
{
    drag_ = drag;
}

void ForceFieldDrag::apply(RigidBody &r) const
{
    r.force_ += -drag_ * r.vel_;
}
