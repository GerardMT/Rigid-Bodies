#ifndef FORCEFIELD_H
#define FORCEFIELD_H

#include "rigid_body.h"

class ForceField
{
public:
    virtual ~ForceField() {};

    virtual void apply(RigidBody &r) const = 0;
};

#endif // FORCEFIELD_H
