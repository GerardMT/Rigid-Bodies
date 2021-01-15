#include "collider_box.h"

#include <glm/gtx/quaternion.hpp>


ColliderBox::ColliderBox()
{
}

ColliderBox::ColliderBox(float a, float b, float c)
{
    a_ = a;
    b_ = b;
    c_ = c;

    min_.x = +a / 2.0f;
    min_.y = +b / 2.0f;
    min_.z = +c / 2.0f;
    max_.x = -a / 2.0f;
    max_.y = -b / 2.0f;
    max_.z = -c / 2.0f;
}

Collider *ColliderBox::clone() const
{
    ColliderBox *box = new ColliderBox();

    box->pos_ = pos_;
    box->rot_ = rot_;
    box->min_ = min_;
    box->max_ = max_;
    box->a_ = a_;
    box->b_ = b_;
    box->c_ = c_;

    return box;
}

void ColliderBox::pos(const glm::vec3 &p)
{
    pos_ = p;
}

void ColliderBox::rot(const glm::quat &r)
{
    rot_ = r;
}

void ColliderBox::intertialTensorInverted(glm::mat3 &t, float mass) const
{
    float prod = 3.0f * 1.0f / mass;

    t[0][0] = prod * 1.0f / (b_ * b_ + c_ * c_);
    t[0][1] = 0.0f;
    t[0][2] = 0.0f;
    t[1][0] = prod * 1.0f / (a_ * a_ + c_ * c_);
    t[1][1] = 0.0f;
    t[1][2] = 0.0f;
    t[2][0] = prod * 1.0f / (a_ * a_ + b_ * b_);
    t[2][1] = 0.0f;
    t[2][2] = 0.0f;
}
