#ifndef COLLIDER_H
#define COLLIDER_H

#include <glm/glm.hpp>

class Collider
{
public:
    ~Collider() {};

    virtual Collider *clone() const = 0;

    virtual void pos(const glm::vec3 &p);

    virtual void rot(const glm::quat &r);

    virtual void intertialTensorInverted(glm::mat3 &t, float mass) const = 0;
};

#endif // COLLIDER_H
