#ifndef COLLIDERSPHERE_H
#define COLLIDERSPHERE_H

#include "collider.h"

class ColliderSphere : public Collider
{
public:
    ColliderSphere(float radius);

    ~ColliderSphere();

    void pos(const glm::vec3 &p) override;

    void rot(const glm::quat &r) override;

    void intertialTensorInverted(glm::mat3 &t, float mass) const override;

   glm::vec3 pos_;

   float radius_;
};

#endif // COLLIDERSPHERE_H
