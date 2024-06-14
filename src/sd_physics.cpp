#include <sd_physics.h>
#include <sd_memory.h>

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

void sd_particle_intergrate(SDParticle *p, f32 dt) {
    if(p->inv_mass <= 0.0f) return;

    p->position += p->velocity * dt;

    SDVec3 resulting_acc = p->acceleration;
    resulting_acc += p->force_accum * p->inv_mass;
    p->velocity += resulting_acc * dt;

    p->velocity *= std::powf(p->damping, dt);

    p->force_accum = SDVec3();
}

void sd_particle_add_force(SDParticle *p, SDVec3 &force) {
    p->force_accum += force;
}

void sd_particle_gravity_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt) {
    if(p->inv_mass <= 0.0f) return;
    sd_particle_add_force(p, fg->gravity.gravity * sd_particle_get_mass(p));
}

void sd_particle_drag_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt) {
    SDVec3 force = p->velocity;
    // callculate the total drag coefficient
    f32 drag_coeff = sd_vec3_len(force);
    drag_coeff = fg->drag.k1 * drag_coeff + fg->drag.k2 * drag_coeff * drag_coeff;
    sd_vec3_normalize(force);
    force *= -drag_coeff;
    sd_particle_add_force(p, force);
}

void sd_particle_spring_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt) {
    SDVec3 force = p->position - fg->spring.other->position;
    // calculate the magnitude of the force
    f32 magnitude = sd_vec3_len(force);
    magnitude = std::fabs(magnitude - fg->spring.rest_length);
    magnitude *= fg->spring.spring_constant;

    sd_vec3_normalize(force);
    force *= -magnitude;
    sd_particle_add_force(p, force);
}

void sd_particle_anchored_spring_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt) {
    SDVec3 force = p->position - *fg->anchored_spring.anchor;
    // calculate the magnitude of the force
    f32 magnitude = sd_vec3_len(force);
    magnitude = std::fabs(magnitude - fg->spring.rest_length);
    magnitude *= fg->spring.spring_constant;

    sd_vec3_normalize(force);
    force *= -magnitude;
    sd_particle_add_force(p, force);
}

void sd_particle_force_generator_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt) {
    switch(fg->type) {
        case SD_PFG_GRAVITY: sd_particle_gravity_update(fg, p, dt); break;
        case SD_PFG_DRAG: sd_particle_drag_update(fg, p, dt); break;
        case SD_PFG_SPRING: sd_particle_spring_update(fg, p, dt); break;
        case SD_PFG_ANCHORED_SPRING: sd_particle_anchored_spring_update(fg, p, dt); break;
    }
}

SDParticleForceRegistry *sd_particle_force_registry_create(SDArena *arena, i32 particle_count) {
    SDParticleForceRegistry *registry = sd_arena_push_struct(arena, SDParticleForceRegistry);
    registry->registrations = sd_arena_push_array(arena, particle_count, SDParticleForceRegistration);
    registry->registrations_count = 0;
    registry->registrations_max = particle_count;
    return registry;
}

void sd_particle_force_registry_add(SDParticleForceRegistry *registry, SDParticle *p, SDParticleForceGenerator *fg) {
    SD_ASSERT(registry->registrations_count + 1 < registry->registrations_max);
    registry->registrations[registry->registrations_count++] = { p, fg };
}

void sd_particle_force_registry_update(SDParticleForceRegistry *registry, f32 dt) {
    for(i32 i = 0; i < registry->registrations_count; i++) {
        SDParticleForceRegistration *r = registry->registrations + i;
        sd_particle_force_generator_update(r->fg, r->particle, dt);
    }
}



static f32 calculate_separating_velocity(SDParticleContact *pc) {
    SDVec3 relative_velocity = pc->particle[0]->velocity;
    if(pc->particle[1]) relative_velocity -= pc->particle[1]->velocity;
    return sd_vec3_dot(relative_velocity, pc->contact_normal);
}


static void resolve_velocity(SDParticleContact *pc, f32 dt) {
    f32 separating_velocity = calculate_separating_velocity(pc);
    // chack if it needs to be resolve
    if(separating_velocity > 0) {
        return;
    }

    // calculate the new separating velocity
    f32 new_sep_velocity = -separating_velocity * pc->restitution;

    f32 delta_velocity = new_sep_velocity - separating_velocity;

    // we apply the change in velocity in proportion to 
    // their inverse mass
    f32 total_inv_mass = pc->particle[0]->inv_mass;
    if(pc->particle[1]) total_inv_mass += pc->particle[1]->inv_mass;

    // if all particles have infinite mass the impulse have no effect
    if(total_inv_mass <= 0) {
        return;
    }

    // calculate the impulse to apply
    f32 impulse = delta_velocity / total_inv_mass;

    // find the amount of impulse per unit of inverse mass
    SDVec3 impulse_per_inv_mass = pc->contact_normal * impulse;

    // apply the impulses: they are applied in the direction of the contact,
    // and are proportional to the inverse mass
    pc->particle[0]->velocity += impulse_per_inv_mass * pc->particle[0]->inv_mass;
    if(pc->particle[1]) {
        pc->particle[1]->velocity += impulse_per_inv_mass * -pc->particle[1]->inv_mass;
    }


}

static void resolve_interpenetration(SDParticleContact *pc, f32 dt) {

}

void sd_particle_contact_resolve(SDParticleContact *pc, f32 dt) {
    resolve_velocity(pc, dt);
    resolve_interpenetration(pc, dt);
}

