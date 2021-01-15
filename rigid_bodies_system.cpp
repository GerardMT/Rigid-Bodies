#include "collider_box.h"
#include "collider_sphere.h"
#include "rigid_bodies_system.h"

#include <glm/gtx/quaternion.hpp>
#include "object.h"

RigidBodiesSystem::RigidBodiesSystem()
{

}

RigidBodiesSystem::~RigidBodiesSystem()
{
    for (auto f : force_fields_) {
        delete f;
    }
}

void RigidBodiesSystem::addRigidBody(RigidBody &r)
{
    rigid_bodies_.push_back(&r);
}

void RigidBodiesSystem::addForceField(ForceField &f)
{
    force_fields_.push_back(&f);
}


struct RigidBodyPhysics {
    RigidBodyPhysics(RigidBody &r) {
        pos_ = r.pos_;
        vel_ = r.vel_;
        rot_ = r.rot_;
        lin_mom_ = r.lin_mom_;
        ang_mom_ = r.ang_mom_;
        r_ = &r;
    };

    glm::vec3 pos_;
    glm::vec3 vel_;
    glm::quat rot_;
    glm::vec3 lin_mom_;
    glm::vec3 ang_mom_;

    RigidBody *r_;

    void apply(RigidBody &r)
    {
        r.pos_ = pos_;
        r.vel_ = vel_;
        r.rot_ = rot_;
        r.lin_mom_ = lin_mom_;
        r.ang_mom_ = ang_mom_;
    }
};

/**
 * Semi-implicit Euler
 */
void solve(RigidBodyPhysics &p, float dt)
{
    p.lin_mom_ = p.lin_mom_ + dt * p.r_->force_;
    p.vel_ = p.lin_mom_ / p.r_->mass_;
    p.pos_ = p.pos_ + dt * p.vel_;

    p.ang_mom_= p.ang_mom_+ dt * p.r_->torque_;
    glm::mat3 rot_mat = glm::toMat3(p.rot_);
    glm::mat3 i_inv = rot_mat * p.r_->intertia_tensor_inv_ * glm::transpose(rot_mat);
    glm::vec3 w = i_inv * p.ang_mom_;
    glm::quat rot_der_ = 0.5f * w * p.rot_;
    p.rot_ = glm::mix(p.rot_, rot_der_, dt);
}

/**
 * Semi-implicit Euler
 */
void solve(RigidBody &r, float dt)
{
    r.lin_mom_ = r.lin_mom_ + dt * r.force_;
    r.vel_ = r.lin_mom_ / r.mass_;
    r.pos_ = r.pos_ + dt * r.vel_;

    r.ang_mom_= r.ang_mom_+ dt * r.torque_;
    glm::mat3 rot_mat = glm::toMat3(r.rot_);
    glm::mat3 i_inv = rot_mat * r.intertia_tensor_inv_ * glm::transpose(rot_mat);
    glm::vec3 w = i_inv * r.ang_mom_;
    glm::quat rot_der_ = 0.5f * w * r.rot_;
    r.rot_ = glm::mix(r.rot_, rot_der_, dt);
}

void RigidBodiesSystem::update(float dt)
{
    for (auto &r_a : rigid_bodies_) {
        if (r_a->fixed_) {
            continue;
        }

        for (auto &f : force_fields_) {
            f->apply(*r_a);
        }

        solve(*r_a, dt);

        bool collision;
        bool collision_free = true;
        do {
            collision = false;
            for (auto &r_b : rigid_bodies_) {
                if (r_a != r_b) {
                    collision = collide(*r_a, *r_b, dt);
                    collision_free = false;
                    break;
                }
            }
        } while (collision);
    }
}

inline void colliderCollideSphereBox(RigidBodyPhysics p_sphere, RigidBodyPhysics p_box, glm::vec3 &contact_point, float &contact_dist)
{
    // TODO
}

void collideContactPoint(RigidBodyPhysics &p_a, RigidBodyPhysics &p_b, glm::vec3 &contact_point, float &contact_dist)
{
    if (dynamic_cast<ColliderSphere *>(p_a.r_->collider_) != nullptr
            && dynamic_cast<ColliderSphere *>(p_b.r_->collider_) != nullptr)
    {
        glm::vec3 ab = p_b.pos_ - p_a.pos_;

        contact_point = p_a.pos_ + 0.5f * ab;
        contact_dist = glm::length(ab) - (static_cast<ColliderSphere *>(p_a.r_->collider_)->radius_ + static_cast<ColliderSphere *>(p_b.r_->collider_)->radius_);
    }
    else if (dynamic_cast<ColliderBox *>(p_a.r_->collider_) != nullptr
             && dynamic_cast<ColliderBox *>(p_b.r_->collider_) != nullptr)
    {
        // TODO
    }
    else if (dynamic_cast<ColliderSphere *>(p_a.r_->collider_) != nullptr
             && dynamic_cast<ColliderBox *>(p_b.r_->collider_) != nullptr)
    {
        colliderCollideSphereBox(p_a, p_b, contact_point, contact_dist);
    }
    else if (dynamic_cast<ColliderBox *>(p_a.r_->collider_) != nullptr
             && dynamic_cast<ColliderSphere *>(p_b.r_->collider_) != nullptr)
    {
        colliderCollideSphereBox(p_a, p_b, contact_point, contact_dist);
    }
}

bool RigidBodiesSystem::collide(RigidBody &r_a, RigidBody &r_b, float dt)
{
    RigidBodyPhysics p_a(r_a);
    RigidBodyPhysics p_b(r_b);

    glm::vec3 contact_point;
    float contact_dist;
    collideContactPoint(p_a, p_b, contact_point, contact_dist);
    if (contact_dist > 0) {
        return false;
    }

    while (abs(contact_dist) > contact_point_dist_) {
        float dt_step = (contact_dist < 0) ? -dt : +dt;
        dt *= 0.5;

        solve(p_a, dt_step);
        solve(p_b, dt_step);

        collideContactPoint(p_a, p_b, contact_point, contact_dist);
    }

    // TODO Generate impulse

    return true;
}

