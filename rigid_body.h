#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "collider.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class Object;

class RigidBody
{
public:
    friend class Object;

    RigidBody(float mass, Collider &collider);

    RigidBody(const RigidBody &r);

    ~RigidBody();

    void pos(const glm::vec3 &v);

    void rot(const glm::quat &q);

    const glm::vec3 &pos() const;

    const glm::quat &rot() const;

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

private:
    glm::vec3 *pos_;
    glm::quat *rot_;

    bool reference_;
};

#endif // RIGIDBODY_H
