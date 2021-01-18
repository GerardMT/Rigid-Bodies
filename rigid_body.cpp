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
    collider_->rigid_body_ = this;

    lin_mom_ = glm::vec3(0.0f, 0.0f, 0.0f);
    ang_mom_ = glm::vec3(0.0f, 0.0f, 0.0f);

    mass_ = mass;

    force_ = glm::vec3(0.0f);
    torque_ = glm::vec3(0.0f);

    collider.intertialTensorInverted(inertia_tensor_inv_, mass_);

    fixed_ = false;
}

void RigidBody::translate(const glm::vec3 &t)
{
    *pos_ += t;
    collider_->translate(t);
}

void RigidBody::rotate(const glm::quat &q)
{
    *rot_ = glm::normalize(*rot_ * q);
    collider_->rotate(q);
}

const glm::vec3 &RigidBody::pos() const
{
    return *pos_;
}

const glm::quat &RigidBody::rot() const
{
    return *rot_;
}
