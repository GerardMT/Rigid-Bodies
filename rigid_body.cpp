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

    fixed_ = false;
    reference_ = false;
}

RigidBody::RigidBody(const RigidBody &r)
{
    vel_ = r.vel_;
    lin_mom_ = r.ang_mom_;
    ang_mom_ = r.ang_mom_;
    force_ = r.force_;
    torque_ = r.torque_;
    mass_ = r.mass_;
    intertia_tensor_inv_ = r.intertia_tensor_inv_;
    collider_ = r.collider_->clone();
    fixed_ = r.fixed_;

    pos_ = new glm::vec3;
    rot_ = new glm::quat;
    reference_ = true;
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
