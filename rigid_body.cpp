#include "rigid_body.h"

#include "object.h"

#include <glm/gtx/quaternion.hpp>

RigidBody::~RigidBody()
{
    delete collider_;
}

RigidBody::RigidBody(float mass, Collider &collider)
{
    collider_ = &collider;

    lin_mom_ = glm::vec3(0.0f, 0.0f, 0.0f);
    ang_mom_ = glm::vec3(0.0f, 0.0f, 0.0f);

    mass_ = mass;

    force_ = glm::vec3(0);
    torque_ = glm::vec3(0);

    collider.intertialTensorInverted(inertia_tensor_inv_, mass);

    fixed_ = false;
}

void RigidBody::pos(const glm::vec3 &v)
{
    *pos_ = v;
    collider_->pos(v);
}

void RigidBody::rot(const glm::quat &q)
{
    *rot_ = q;
    collider_->rot(q);
}

const glm::vec3 &RigidBody::pos() const
{
    return *pos_;
}

const glm::quat &RigidBody::rot() const
{
    return *rot_;
}
