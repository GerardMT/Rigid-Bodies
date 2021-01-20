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

    ~RigidBody();

    void translate(const glm::vec3 &t);

    void rotate(const glm::quat &q);

    const glm::vec3 &pos() const;

    const glm::quat &rot() const;

    glm::vec3 vel_;
    glm::vec3 lin_mom_;
    glm::vec3 ang_mom_;

    // Non-properties
    glm::vec3 force_;
    glm::vec3 torque_;

    glm::mat3 i_inv_;
    glm::vec3 w_;

    // Constants
    float mass_;
    glm::mat3 inertia_tensor_inv_;

    Collider *collider_;

    bool fixed_;

    glm::quat *rot_;

private:
    glm::vec3 *pos_;
};

#endif // RIGIDBODY_H
