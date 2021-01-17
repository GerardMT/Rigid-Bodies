#ifndef RIGIDBODIESSYSTEM_H
#define RIGIDBODIESSYSTEM_H

#include "collider.h"
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

private:
    const float contact_point_dist_ = 0.05f;
    const float relative_velocity_threshold_ = 0.05;

    vector<RigidBody *> rigid_bodies_;

    vector<ForceField *> force_fields_;

    bool collide(RigidBody &a, RigidBody &b, float dt);
};

#endif // RIGIDBODIESSYSTEM_H
