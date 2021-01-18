#ifndef RIGIDBODIESSYSTEM_H
#define RIGIDBODIESSYSTEM_H

#include "collider.h"
#include "collider_box.h"
#include "collider_sphere.h"
#include "force_field.h"
#include "paint_gl.h"

#include <vector>

using namespace std;

class RigidBodiesSystem
{
public:
    ~RigidBodiesSystem();

    void addRigidBody(RigidBody &r);

    void addForceField(ForceField &f);

    void update(float dt);

    void clear();

    float coefficient_of_restitution_ = 0.5f;

private:
    struct Contact {
        glm::vec3 p_;
        glm::vec3 n_;
        float dist_;
        RigidBody *r_a_;
        RigidBody *r_b_;
    };

    const float contact_point_dist_ = 0.01f;
    const float relative_velocity_threshold_ = 0.071;

    const unsigned int contact_steps_ = 10;

    vector<RigidBody *> rigid_bodies_;

    vector<ForceField *> force_fields_;

    bool contactPointSphereSphere(const ColliderSphere &sphere_a, const ColliderSphere &sphere_b, vector<Contact> &contacts);

    bool contactPointBoxBox(const ColliderBox &box_a, const ColliderBox &box_b, vector<Contact> &contacts);

    bool contactPointBoxBoxImmersive(const ColliderBox &box_a, const ColliderBox &box_b, vector<Contact> &contacts);

    bool contactPointSphereBox(const ColliderSphere &sphere, const ColliderBox &box, vector<Contact> &contacts);

    void collision(const vector<Contact> &contacts);

    bool contactPoints(RigidBody &r_a, RigidBody &r_b, vector<Contact> &contacts);

    bool colliding(Contact &c);
};

#endif // RIGIDBODIESSYSTEM_H
