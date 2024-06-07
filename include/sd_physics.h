#pragma once

#include <sd_math.h>

struct SDParticle {
    SDVec3 position;
    SDVec3 velocity;
    SDVec3 acceleration;
    f32 damping;
    f32 inv_mass;
};

void sd_particle_set_mass(SDParticle *p, f32 mass) {
    SD_ASSERT(mass != 0);
    p->inv_mass = 1.0f/mass;
}

f32 sd_particle_get_mass(SDParticle *p) {
    if(p->inv_mass == 0) {
        return FLT_MAX;
    } else {
        return 1.0f / p->inv_mass;
    }
}

