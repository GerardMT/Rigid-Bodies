#include "collider_sphere.h"


ColliderSphere::ColliderSphere(float radius)
{
    radius_ = radius;

    pos_ = glm::vec3(0.0f);
}

ColliderSphere::~ColliderSphere()
{
}

void ColliderSphere::translate(const glm::vec3 &t)
{
    pos_ += t;
}

void ColliderSphere::rotate(const glm::quat &r)
{
}

void ColliderSphere::intertialTensorInverted(glm::mat3 &t, float mass) const
{
    float v = 0.5 * radius_ * radius_ * 5.0f * 1.0f / mass;

    t[0][0] = v;
    t[0][1] = 0.0f;
    t[0][2] = 0.0f;
    t[1][0] = 0.0f;
    t[1][1] = v;
    t[1][2] = 0.0f;
    t[2][0] = 0.0f;
    t[2][1] = 0.0f;
    t[2][2] = v;
}
