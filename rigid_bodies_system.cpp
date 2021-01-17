#include "rigid_bodies_system.h"

#include "collider_box.h"
#include "collider_sphere.h"
#include "object.h"

#include <glm/gtx/quaternion.hpp>

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

/**
 * Semi-implicit Euler
 */
void solve(RigidBody &r, float dt)
{
    r.lin_mom_ = r.lin_mom_ + dt * r.force_;
    r.vel_ = r.lin_mom_ / r.mass_;
    r.pos(r.pos() + dt * r.vel_);

    r.ang_mom_= r.ang_mom_+ dt * r.torque_;
    glm::mat3 rot_mat = glm::toMat3(r.rot());
    r.i_inv_ = rot_mat * r.inertia_tensor_inv_ * glm::transpose(rot_mat);
    r.w_ = r.i_inv_ * r.ang_mom_;
    glm::quat rot_der_ = 0.5f * r.w_ * r.rot();
    r.rot(glm::mix(r.rot(), rot_der_, dt));
}

void solve_impulse(RigidBody &r, float dt)
{
    r.vel_ = r.lin_mom_ / r.mass_;
    r.pos(r.pos() + dt * r.vel_);

    glm::mat3 rot_mat = glm::toMat3(r.rot());
    r.i_inv_ = rot_mat * r.inertia_tensor_inv_ * glm::transpose(rot_mat);
    r.w_ = r.i_inv_ * r.ang_mom_;
    glm::quat rot_der_ = 0.5f * r.w_ * r.rot();
    r.rot(glm::mix(r.rot(), rot_der_, dt));
}

void RigidBodiesSystem::update(float dt)
{
    for (auto &r_a : rigid_bodies_) {
        if (r_a->fixed_) {
            continue;
        }

        r_a->force_ = glm::vec3(0.0);
        for (auto &f : force_fields_) {
            f->apply(*r_a);
        }

        solve(*r_a, dt);

        bool collision;
        do {
            collision = false;
            for (auto &r_b : rigid_bodies_) {
                if (r_a != r_b) {
                    collision = collide(*r_a, *r_b, dt);
                    break;
                }
            }
        } while (collision);
    }
}

inline void contactPointSphereSphere(const ColliderSphere &sphere_a, const ColliderSphere &sphere_b, float &contact_dist, glm::vec3 &contact_point, glm::vec3 &contact_norm)
{
    glm::vec3 ba = sphere_a.pos_ - sphere_b.pos_;

    contact_dist = glm::length(ba) - (sphere_a.radius_ + sphere_b.radius_);
    contact_point = sphere_b.pos_ + 0.5f * ba;
    contact_norm = glm::normalize(ba);
}

inline void contactPointBoxBox(const ColliderBox &box_a, const ColliderBox &box_b, float &contact_dist, glm::vec3 &contact_point, glm::vec3 &contact_norm)
{
    // TODO
}

inline void contactPointSphereBox(const ColliderSphere &sphere, const ColliderBox &box, float &contact_dist, glm::vec3 &contact_point, glm::vec3 &contact_norm)
{
    glm::vec3 sphere_pos = glm::toMat3(glm::inverse(box.rot_)) * (sphere.pos_ - box.pos_);

    contact_point.x = max(box.min_.x, min(sphere_pos.x, box.max_.x));
    contact_point.y = max(box.min_.y, min(sphere_pos.y, box.max_.y));
    contact_point.z = max(box.min_.z, min(sphere_pos.z, box.max_.z));

    contact_dist = glm::length(contact_point - sphere_pos) - sphere.radius_;

    contact_point = glm::toMat3(box.rot_) * contact_point + box.pos_;
    contact_norm = glm::normalize(sphere.pos_ - contact_point);
}

void contactPoint(RigidBody &r_a, RigidBody &r_b, float &contact_dist, glm::vec3 &contact_point, glm::vec3 &contact_norm)
{
    if (dynamic_cast<ColliderSphere *>(r_a.collider_) != nullptr
            && dynamic_cast<ColliderSphere *>(r_b.collider_) != nullptr)
    {
        contactPointSphereSphere(*static_cast<ColliderSphere *>(r_a.collider_), *static_cast<ColliderSphere *>(r_b.collider_), contact_dist, contact_point, contact_norm);
    }
    else if (dynamic_cast<ColliderBox *>(r_a.collider_) != nullptr
             && dynamic_cast<ColliderBox *>(r_b.collider_) != nullptr)
    {
        contactPointBoxBox(*static_cast<ColliderBox *>(r_a.collider_), *static_cast<ColliderBox *>(r_a.collider_), contact_dist, contact_point, contact_norm);
    }
    else if (dynamic_cast<ColliderSphere *>(r_a.collider_) != nullptr
             && dynamic_cast<ColliderBox *>(r_b.collider_) != nullptr)
    {
        contactPointSphereBox(*static_cast<ColliderSphere *>(r_a.collider_), *static_cast<ColliderBox *>(r_b.collider_), contact_dist, contact_point, contact_norm);
    }
    else if (dynamic_cast<ColliderBox *>(r_a.collider_) != nullptr
             && dynamic_cast<ColliderSphere *>(r_b.collider_) != nullptr)
    {
        contactPointSphereBox(*static_cast<ColliderSphere *>(r_b.collider_), *static_cast<ColliderBox *>(r_a.collider_), contact_dist, contact_point, contact_norm);
        contact_norm = -contact_norm;
    }
}

bool RigidBodiesSystem::collide(RigidBody &r_a, RigidBody &r_b, float dt)
{
    // Find contact
    float contact_dist;
    glm::vec3 contact_point;
    glm::vec3 contact_norm;

    contactPoint(r_a, r_b, contact_dist, contact_point, contact_norm);
    if (contact_dist > 0) {
        return false;
    }

    float dt_contact = dt;
    unsigned int it = 0;
    while (abs(contact_dist) > contact_point_dist_ && it < 10) {
        float dt_step = (contact_dist < 0) ? -dt_contact : +dt_contact ;
        dt_contact *= 0.5;

        solve(r_a, dt_step);
        if (!r_b.fixed_) {
            solve(r_b, dt_step);
        }

        contactPoint(r_a, r_b, contact_dist, contact_point, contact_norm);
        ++it;
    }

    // Impulse
    const float coef_restitution = 0.5f;

    glm::vec3 rr_a = contact_point - r_a.pos();
    glm::vec3 rr_b = contact_point - r_b.pos();

    glm::vec3 p_v_a = r_a.vel_ + glm::cross(r_a.w_, rr_a);
    glm::vec3 p_v_b = r_b.vel_ + glm::cross(r_b.w_, rr_b);

    float v_r = glm::dot(contact_norm, (p_v_a - p_v_b));

    if (v_r > relative_velocity_threshold_) {
        return false;
    } else if (v_r > -relative_velocity_threshold_)  {
        return false; // Resting
    }

    float den_mass_a = r_a.fixed_ ? 0.0f : 1.0f / r_a.mass_;
    float den_mass_b = r_b.fixed_ ? 0.0f : 1.0f / r_b.mass_;
    float den_a = r_a.fixed_ ? 0.0f : glm::dot(contact_norm, glm::cross(r_a.i_inv_ * glm::cross(rr_a, contact_norm), rr_a));
    float den_b = r_b.fixed_ ? 0.0f : glm::dot(contact_norm, glm::cross(r_b.i_inv_ * glm::cross(rr_b, contact_norm), rr_b));

    float j = (-(1 + coef_restitution) * v_r) / (den_mass_a + den_mass_b + den_a + den_b);

    glm::vec3 J = j * contact_norm;

    r_a.lin_mom_ += J;
    r_b.lin_mom_ -= J;

    r_a.ang_mom_ += glm::cross(rr_a, J);
    r_b.ang_mom_ -= glm::cross(rr_b, J);

    float dt_impulse = dt - dt_contact;
    if (!r_a.fixed_) {
        solve(r_a, dt_impulse);
    }
    if (!r_b.fixed_) {
        solve(r_b, dt_impulse);
    }

    return true; // TODO DEBUG
}

