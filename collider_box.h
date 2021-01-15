#ifndef COLLIDERBOX_H
#define COLLIDERBOX_H

#include "collider.h"

#include <glm/gtx/quaternion.hpp>

class ColliderBox : public Collider
{
public:    
    ColliderBox(float a, float b, float c);

    Collider *clone() const override;

    void pos(const glm::vec3 &p) override;

    void rot(const glm::quat &r) override;

    void intertialTensorInverted(glm::mat3 &t, float mass) const override;

    static const int NUM_VERTICES = 8;

    glm::vec3 pos_;
    glm::quat rot_;

    glm::vec3 min_;
    glm::vec3 max_;

private:
    ColliderBox();

    float a_;
    float b_;
    float c_;
};

#endif // COLLIDERBOX_H
