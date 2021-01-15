#ifndef COLLIDER_H
#define COLLIDER_H

#include <glm/glm.hpp>

class Collider
{
public:
    virtual ~Collider() {};

    virtual void intertialTensorInverted(glm::mat3 &t, float mass) const = 0;
};

#endif // COLLIDER_H
