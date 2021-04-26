#ifndef COLLIDERSPHERE_H
#define COLLIDERSPHERE_H

#include "collider.h"

class ColliderSphere : public Collider
{
public:
    ColliderSphere(float radius);

    ~ColliderSphere();

    void translate(const glm::vec3 &t) override;

    void rotate(const glm::quat &r) override;

    void intertialTensorInverted(glm::mat3 &t, float mass) const override;

   glm::vec3 pos_;

   float radius_;
};

#endif // COLLIDERSPHERE_H
