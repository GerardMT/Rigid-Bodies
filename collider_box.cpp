#include "collider_box.h"

ColliderBox::ColliderBox(float a, float b, float c)
{
    a_ = a;
    b_ = b;
    c_ = c;
}

void ColliderBox::intertialTensorInverted(glm::mat3 &t, float mass) const
{
    float prod = 3.0f * 1.0f / mass;

    t[0][0] = prod * 1.0f / (b_ * b_ + c_ * c_);
    t[0][1] = 0.0f;
    t[0][2] = 0.0f;
    t[1][0] = prod * 1.0f / (a_ * a_ + c_ * c_);
    t[1][1] = 0.0f;
    t[1][2] = 0.0f;
    t[2][0] = prod * 1.0f / (a_ * a_ + b_ * b_);
    t[2][1] = 0.0f;
    t[2][2] = 0.0f;
}
