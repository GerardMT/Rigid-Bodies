#include "rigid_body.h"
#include "object.h"

RigidBody::~RigidBody()
{
    delete collider_;
}

RigidBody::RigidBody(Object &obj, float mass) : pos_(obj.pos_), rot_(obj.rot_)
{
    lin_mom_ = glm::vec3(0.0f, 0.0f, 0.0f);
    ang_mom_ = glm::vec3(0.0f, 0.0f, 0.0f);

    mass_ = mass;

    force_ = glm::vec3(0);
    torque_ = glm::vec3(0);

    fixed_ = false;
}

void RigidBody::setCollider(Collider &c)
{
    collider_ = &c;
    c.intertialTensorInverted(intertia_tensor_inv_, mass_);
}
