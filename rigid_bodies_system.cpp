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
    r.translate(dt * r.vel_);

    r.ang_mom_= r.ang_mom_+ dt * r.torque_;
    glm::mat3 rot_mat = glm::toMat3(r.rot());
    r.i_inv_ = rot_mat * r.inertia_tensor_inv_ * glm::transpose(rot_mat);
    r.w_ = r.i_inv_ * r.ang_mom_;
    glm::quat rot_der_ = 0.5f * (glm::quat(0.0, (r.w_)) * r.rot());

    glm::quat rot_diff = glm::normalize(r.rot() + dt * rot_der_) * glm::inverse(r.rot());
    r.rotate(rot_diff);
}

void RigidBodiesSystem::update(float dt)
{
    // Forces
    for (auto &r_a : rigid_bodies_) {
        if (!r_a->fixed_) {
            r_a->force_ = glm::vec3(0.0f);
            r_a->torque_ = glm::vec3(0.0f);
            for (auto &f : force_fields_) {
                f->apply(*r_a);
            }
        }
    }

    float time_start = 0;
    float time_end = dt;
    float dt_frac = dt;

    vector<Contact> contacts;

    bool has_contacts;
    bool interpenetration;
    do {
        // Solve
        for (auto r : rigid_bodies_) {
            if (!r->fixed_) {
                solve(*r, dt);
            }
        }

        interpenetration = false;
        contacts.clear();
        for (unsigned int i = 0; i < rigid_bodies_.size() && !interpenetration; ++i) {
            for (unsigned int j = 0; j < i && !interpenetration; ++j) {
                interpenetration = contactPoints(*rigid_bodies_[i], *rigid_bodies_[j], contacts);
            }
        }

        if (interpenetration) {
            // Find state without interpenetrations
            bool always_interpenetration = true;
            unsigned int it = 0;
            do {
                dt_frac = 0.5 * dt_frac;
                dt = interpenetration ? -dt_frac : +dt_frac;
                for (auto r : rigid_bodies_) {
                    if (!r->fixed_) {
                        solve(*r, dt);
                    }
                }

                interpenetration = false;
                contacts.clear();
                for (unsigned int i = 0; i < rigid_bodies_.size() && !interpenetration; ++i) {
                    for (unsigned int j = 0; j < i && !interpenetration; ++j) {
                        interpenetration = contactPoints(*rigid_bodies_[i], *rigid_bodies_[j], contacts);
                    }
                }

                always_interpenetration &= interpenetration;
                ++it;
            } while ((contacts.size() == 0 || interpenetration) && it < contact_steps_);

            if (always_interpenetration) { // The no interpenetation state is the intial one
                for (auto r : rigid_bodies_) { // Solve
                    if (!r->fixed_) {
                        solve(*r, -dt_frac + time_end);
                    }
                }

                interpenetration = false;
                contacts.clear();
                for (unsigned int i = 0; i < rigid_bodies_.size() && !interpenetration; ++i) {
                    for (unsigned int j = 0; j < i && !interpenetration; ++j) {
                        interpenetration = contactPoints(*rigid_bodies_[i], *rigid_bodies_[j], contacts);
                    }
                }

                if (contacts.size() == 0) { // Unable to get contact point because of not contempling resting contact
                    // Fix them
                    if (fix_on_resting_) {
                        for (unsigned int i = 0; i < rigid_bodies_.size(); ++i) {
                            for (unsigned int j = 0; j < i ; ++j) {
                                if(contactPoints(*rigid_bodies_[i], *rigid_bodies_[j], contacts)) {
                                    rigid_bodies_[i]->fixed_ = true;
                                    rigid_bodies_[j]->fixed_ = true;
                                }
                            }
                        }
                    }
                    interpenetration = false; // Allow exit
                }
            }

            time_start = dt;
            dt = time_end - time_start;
            dt_frac = dt;
        }

        has_contacts = contacts.size() > 0;
        if(has_contacts) {
            collision(contacts); // Impulse
        }
    } while(has_contacts || interpenetration);
}

void RigidBodiesSystem::clear()
{
    rigid_bodies_.clear();
}

bool RigidBodiesSystem::colliding(Contact &c)
{
    glm::vec3 rr_a = c.p_ - c.r_a_->pos();
    glm::vec3 rr_b = c.p_ - c.r_b_->pos();

    glm::vec3 p_v_a = c.r_a_->vel_ + glm::cross(c.r_a_->w_, rr_a);
    glm::vec3 p_v_b = c.r_b_->vel_ + glm::cross(c.r_b_->w_, rr_b);

    float v_r = glm::dot(c.n_, (p_v_a - p_v_b));

    if(v_r > relative_velocity_threshold_) { // Moving away
        return false;
    } else if (v_r > -relative_velocity_threshold_) { // Resting contact
        if (fix_on_resting_) {
            c.r_a_->fixed_ = true;
            c.r_b_->fixed_ = true;
        }
        return false;
    } else {
        return true;
    }
}

inline bool RigidBodiesSystem::contactPointSphereSphere(const ColliderSphere &sphere_a, const ColliderSphere &sphere_b, vector<Contact> &contacts)
{
    Contact c;
    glm::vec3 ba = sphere_a.pos_ - sphere_b.pos_;

    c.dist_ = glm::length(ba) - (sphere_a.radius_ + sphere_b.radius_);

    c.p_ = sphere_b.pos_ + 0.5f * ba;
    c.n_ = glm::normalize(ba);

    c.r_a_ = sphere_a.rigid_body_;
    c.r_b_ = sphere_b.rigid_body_;

    if (c.dist_ < -contact_point_dist_) { // Interpenetrating
        return true;
    }

    if (abs(c.dist_) < contact_point_dist_ && colliding(c)) {
        contacts.push_back(c);
    }

    return false;
}

inline bool RigidBodiesSystem::contactPointBoxBoxImmersive(const ColliderBox &box_a, const ColliderBox &box_b, vector<Contact> &contacts)
{
    for (unsigned int i = 0; i < ColliderBox::NUM_VERTICES; ++i) {
        glm::vec3 vert_pos = glm::toMat3(glm::inverse(box_a.rot_)) * (box_b.vertices_[i] - box_a.pos_);

        if (vert_pos.x > box_a.min_.x + contact_point_dist_ && vert_pos.x < box_a.max_.x - contact_point_dist_
                && vert_pos.y > box_a.min_.y + contact_point_dist_ && vert_pos.y < box_a.max_.y - contact_point_dist_
                && vert_pos.z > box_a.min_.z + contact_point_dist_&& vert_pos.z < box_a.max_.z - contact_point_dist_) {
            return true;
        }

        Contact c;
        c.p_ = glm::toMat3(box_a.rot_) * vert_pos + box_a.pos_;

        glm::vec3 proj;
        proj.x = max(box_a.min_.x + contact_point_dist_, min(vert_pos.x, box_a.max_.x - contact_point_dist_)); // Not exact
        proj.y = max(box_a.min_.y + contact_point_dist_, min(vert_pos.y, box_a.max_.y - contact_point_dist_));
        proj.z = max(box_a.min_.z + contact_point_dist_, min(vert_pos.z, box_a.max_.z - contact_point_dist_));
        c.n_ = glm::toMat3(box_a.rot_) * glm::normalize(vert_pos - proj) + box_a.pos_;

        c.dist_ = glm::length(vert_pos - proj);

        c.r_a_ = box_b.rigid_body_;
        c.r_b_ = box_a.rigid_body_;

        bool contact = vert_pos.x > box_a.min_.x - contact_point_dist_ && vert_pos.x < box_a.max_.x + contact_point_dist_
                && vert_pos.y > box_a.min_.y - contact_point_dist_ && vert_pos.y < box_a.max_.y + contact_point_dist_
                && vert_pos.z > box_a.min_.z - contact_point_dist_&& vert_pos.z < box_a.max_.z + contact_point_dist_;

        if (contact && colliding(c)) { // TODO Missing negative distance
            contacts.push_back(c);
        }
    }

    return false;
}


inline bool RigidBodiesSystem::contactPointBoxBox(const ColliderBox &box_a, const ColliderBox &box_b, vector<Contact> &contacts)
{
    bool res = contactPointBoxBoxImmersive(box_a, box_b, contacts);
    if (res) {
        return true;
    } else {
        return contactPointBoxBoxImmersive(box_b, box_a, contacts);
    }
}

inline bool RigidBodiesSystem::contactPointSphereBox(const ColliderSphere &sphere, const ColliderBox &box, vector<Contact> &contacts)
{
    glm::vec3 sphere_pos = glm::toMat3(glm::inverse(box.rot_)) * (sphere.pos_ - box.pos_);

    Contact c;
    c.p_.x = max(box.min_.x, min(sphere_pos.x, box.max_.x));
    c.p_.y = max(box.min_.y, min(sphere_pos.y, box.max_.y));
    c.p_.z = max(box.min_.z, min(sphere_pos.z, box.max_.z));

    c.dist_= glm::length(c.p_ - sphere_pos) - sphere.radius_;

    c.p_ = glm::toMat3(box.rot_) * c.p_ + box.pos_;
    c.n_ = glm::normalize(sphere.pos_ - c.p_);

    c.r_a_ = sphere.rigid_body_;
    c.r_b_ = box.rigid_body_;

    if (c.dist_ < -contact_point_dist_) { // Interpenetrating
        return true;
    }

    if (abs(c.dist_) < contact_point_dist_ && colliding(c)) {
        contacts.push_back(c);
    }

    return false;
}

bool RigidBodiesSystem::contactPoints(RigidBody &r_a, RigidBody &r_b, vector<Contact> &contacts)
{
    if (r_a.fixed_ && r_b.fixed_) {
        return false;
    }

    if (dynamic_cast<ColliderSphere *>(r_a.collider_) != nullptr
            && dynamic_cast<ColliderSphere *>(r_b.collider_) != nullptr)
    {
        return contactPointSphereSphere(*static_cast<ColliderSphere *>(r_a.collider_), *static_cast<ColliderSphere *>(r_b.collider_), contacts);
    }
    else if (dynamic_cast<ColliderBox *>(r_a.collider_) != nullptr
             && dynamic_cast<ColliderBox *>(r_b.collider_) != nullptr)
    {
        return contactPointBoxBox(*static_cast<ColliderBox *>(r_a.collider_), *static_cast<ColliderBox *>(r_b.collider_), contacts);
    }
    else if (dynamic_cast<ColliderSphere *>(r_a.collider_) != nullptr
             && dynamic_cast<ColliderBox *>(r_b.collider_) != nullptr)
    {
        return contactPointSphereBox(*static_cast<ColliderSphere *>(r_a.collider_), *static_cast<ColliderBox *>(r_b.collider_), contacts);
    }
    else if (dynamic_cast<ColliderBox *>(r_a.collider_) != nullptr
             && dynamic_cast<ColliderSphere *>(r_b.collider_) != nullptr)
    {
        return contactPointSphereBox(*static_cast<ColliderSphere *>(r_b.collider_), *static_cast<ColliderBox *>(r_a.collider_), contacts);
    } else
    {
        throw 1;
    }
}

void RigidBodiesSystem::collision(const vector<Contact> &contacts)
{
    for (auto &c : contacts) {
        glm::vec3 rr_a = c.p_ - c.r_a_->pos();
        glm::vec3 rr_b = c.p_ - c.r_b_->pos();

        glm::vec3 p_v_a = c.r_a_->fixed_ ? glm::vec3(0.0f) : c.r_a_->vel_ + glm::cross(c.r_a_->w_, rr_a);
        glm::vec3 p_v_b = c.r_b_->fixed_ ? glm::vec3(0.0f) : c.r_b_->vel_ + glm::cross(c.r_b_->w_, rr_b);

        float v_r = glm::dot(c.n_, (p_v_a - p_v_b));

        float den_mass_a = c.r_a_->fixed_ ? 0.0f : 1.0f / c.r_a_->mass_;
        float den_mass_b = c.r_b_->fixed_ ? 0.0f : 1.0f / c.r_b_->mass_;
        float den_a = c.r_a_->fixed_ ? 0.0f : glm::dot(c.n_, glm::cross(c.r_a_->i_inv_ * glm::cross(rr_a, c.n_), rr_a));
        float den_b = c.r_b_->fixed_ ? 0.0f : glm::dot(c.n_, glm::cross(c.r_b_->i_inv_ * glm::cross(rr_b, c.n_), rr_b));

        float j = (-(1 + coefficient_of_restitution_) * v_r) / (den_mass_a + den_mass_b + den_a + den_b);

        glm::vec3 J = j * c.n_;

        c.r_a_->lin_mom_ += J;
        c.r_b_->lin_mom_ -= J;

        c.r_a_->ang_mom_ += glm::cross(rr_a, J);
        c.r_b_->ang_mom_ -= glm::cross(rr_b, J);
    }
}

