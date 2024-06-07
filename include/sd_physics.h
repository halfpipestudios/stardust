#pragma once

#include <sd_math.h>

struct SDArena;

struct SDParticle {
    SDVec3 position;
    SDVec3 velocity;
    SDVec3 acceleration;
    SDVec3 force_accum;
    f32 damping;
    f32 inv_mass;
};

void sd_particle_set_mass(SDParticle *p, f32 mass);
f32 sd_particle_get_mass(SDParticle *p);

void sd_particle_intergrate(SDParticle *p, f32 dt);
void sd_particle_add_force(SDParticle *p, SDVec3 &v);

enum SDParticleForceGeneratorType {
    SD_PFG_GRAVITY,
    SD_PFG_DRAG
};

struct SDParticleGravity {
    SDVec3 gravity;
};


struct SDParticleDrag {
    f32 k1, k2;
};

struct SDParticleForceGenerator {
    SDParticleForceGeneratorType type;
    union {
        SDParticleGravity gravity;
        SDParticleDrag    drag;
    };
};

void sd_particle_gravity_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt);
void sd_particle_drag_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt);

void sd_particle_force_generator_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt);


struct SDParticleForceRegistration {
    SDParticle *particle;
    SDParticleForceGenerator *fg;
};

struct SDParticleForceRegistry {
    SDParticleForceRegistration *registrations;
    i32 registrations_count;
    i32 registrations_max;
};

SDParticleForceRegistry *sd_particle_force_registry_create(SDArena *arena, i32 particle_count);
void sd_particle_force_registry_add(SDParticleForceRegistry *registry, SDParticle *p, SDParticleForceGenerator *fg);
void sd_particle_force_registry_update(SDParticleForceRegistry *registry, f32 dt);