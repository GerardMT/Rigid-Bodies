#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "collider.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class Object;

class RigidBody
{
public:
    RigidBody(Object &obj, float mass);

    ~RigidBody();

    void setCollider(Collider &c);

    glm::vec3 &pos_;
    glm::quat &rot_;
    glm::vec3 vel_;
    glm::vec3 lin_mom_;
    glm::vec3 ang_mom_;

    // Non-properties
    glm::vec3 force_;
    glm::vec3 torque_;

    // Constants
    float mass_;
    glm::mat3 intertia_tensor_inv_;

    Collider *collider_;

    bool fixed_;
};

#endif // RIGIDBODY_H
