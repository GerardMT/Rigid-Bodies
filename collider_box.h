#ifndef COLLIDERBOX_H
#define COLLIDERBOX_H

#include "collider.h"

class ColliderBox : public Collider
{
public:
    ColliderBox(float a, float b, float c);

    void intertialTensorInverted(glm::mat3 &t, float mass) const override;

private:
    float a_;
    float b_;
    float c_;
};

#endif // COLLIDERBOX_H
