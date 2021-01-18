#ifndef COLLIDER_H
#define COLLIDER_H

#include <glm/glm.hpp>

class RigidBody;

class Collider
{
public:
    virtual ~Collider() {};

    virtual void pos(const glm::vec3 &p) = 0;

    virtual void rot(const glm::quat &r) = 0;

    virtual void intertialTensorInverted(glm::mat3 &t, float mass) const = 0;

    RigidBody *rigid_body_;
};

#endif // COLLIDER_H
