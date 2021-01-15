#ifndef COLLIDERSPHERE_H
#define COLLIDERSPHERE_H

#include "collider.h"

class ColliderSphere : public Collider
{
public:
    ColliderSphere(float radius);

    void intertialTensorInverted(glm::mat3 &t, float mass) const override;

    float radius_;
};

#endif // COLLIDERSPHERE_H
