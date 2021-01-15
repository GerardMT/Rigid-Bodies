#include "collider_sphere.h"


ColliderSphere::ColliderSphere(float radius)
{
    radius_ = radius;
}

void ColliderSphere::pos(const glm::vec3 &p)
{
    pos_ += p;
}

void ColliderSphere::intertialTensorInverted(glm::mat3 &t, float mass) const
{
    float v = 0.5 * radius_ * radius_ * 5.0f * 1.0f / mass;

    t[0][0] = v;
    t[0][1] = 0.0f;
    t[0][2] = 0.0f;
    t[1][0] = v;
    t[1][1] = 0.0f;
    t[1][2] = 0.0f;
    t[2][0] = v;
    t[2][1] = 0.0f;
    t[2][2] = 0.0f;
}
