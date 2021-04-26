#include "collider_box.h"

#include <glm/gtx/quaternion.hpp>

#include <iostream>
using namespace std;

ColliderBox::ColliderBox(float a, float b, float c)
{
    a_ = a;
    b_ = b;
    c_ = c;

    max_.x = +a / 2.0f;
    max_.y = +b / 2.0f;
    max_.z = +c / 2.0f;
    min_.x = -a / 2.0f;
    min_.y = -b / 2.0f;
    min_.z = -c / 2.0f;

    pos_ = glm::vec3(0.0f);
    rot_ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    vertices_[0] = max_;
    vertices_[1] = glm::vec3(max_.x, max_.y, min_.z);
    vertices_[2] = glm::vec3(max_.x, min_.y, max_.z);
    vertices_[3] = glm::vec3(max_.x, min_.y, min_.z);
    vertices_[4] = glm::vec3(min_.x, max_.y, max_.z);
    vertices_[5] = glm::vec3(min_.x, max_.y, min_.z);
    vertices_[6] = glm::vec3(min_.x, min_.y, max_.z);
    vertices_[7] = min_;
}

ColliderBox::~ColliderBox()
{
}

void ColliderBox::translate(const glm::vec3 &t)
{
    pos_ += t;

    vertices_[0] += t;
    vertices_[1] += t;
    vertices_[2] += t;
    vertices_[3] += t;
    vertices_[4] += t;
    vertices_[5] += t;
    vertices_[6] += t;
    vertices_[7] += t;
}

void ColliderBox::rotate(const glm::quat &r)
{
    rot_ = r * rot_;

    glm::mat3 mat = glm::toMat3(r);
    vertices_[0] = mat * (vertices_[0] - pos_) + pos_;
    vertices_[1] = mat * (vertices_[1] - pos_) + pos_;
    vertices_[2] = mat * (vertices_[2] - pos_) + pos_;
    vertices_[3] = mat * (vertices_[3] - pos_) + pos_;
    vertices_[4] = mat * (vertices_[4] - pos_) + pos_;
    vertices_[5] = mat * (vertices_[5] - pos_) + pos_;
    vertices_[6] = mat * (vertices_[6] - pos_) + pos_;
    vertices_[7] = mat * (vertices_[7] - pos_) + pos_;
}

void ColliderBox::intertialTensorInverted(glm::mat3 &t, float mass) const
{
    float prod = 12.0f / mass;

    t[0][0] = prod * 1.0f / (b_ * b_ + c_ * c_);
    t[0][1] = 0.0f;
    t[0][2] = 0.0f;
    t[1][0] = 0.0f;
    t[1][1] = prod * 1.0f / (a_ * a_ + c_ * c_);
    t[1][2] = 0.0f;
    t[2][0] = 0.0f;
    t[2][1] = 0.0f;
    t[2][2] = prod * 1.0f / (a_ * a_ + b_ * b_);
}
