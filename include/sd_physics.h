#pragma once

#include <sd_math.h>

struct SDArena;
struct SDBlockAllocator;
//===================================================================
// Particle Physics
//===================================================================
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
    SD_PFG_DRAG,
    SD_PFG_SPRING,
    SD_PFG_ANCHORED_SPRING,
};

struct SDParticleGravity {
    SDVec3 gravity;
};


struct SDParticleDrag {
    f32 k1, k2;
};

struct SDParticleSpring {
    SDParticle *other;
    f32 spring_constant;
    f32 rest_length;
};

struct SDParticleAnchoredSpring {
    SDVec3 *anchor;
    f32 spring_constant;
    f32 rest_length;
};

struct SDParticleForceGenerator {
    SDParticleForceGeneratorType type;
    union {
        SDParticleGravity gravity;
        SDParticleDrag    drag;
        SDParticleSpring spring;
        SDParticleAnchoredSpring anchored_spring;
    };
};

void sd_particle_gravity_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt);
void sd_particle_drag_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt);
void sd_particle_spring_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt);
void sd_particle_anchored_spring_update(SDParticleForceGenerator *fg, SDParticle *p, f32 dt);

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

SDParticleForceRegistry sd_particle_force_registry_create(SDArena *arena, i32 particle_count);
void sd_particle_force_registry_add(SDParticleForceRegistry *registry, SDParticle *p, SDParticleForceGenerator *fg);
void sd_particle_force_registry_update(SDParticleForceRegistry *registry, f32 dt);


struct SDParticleContact {
    SDParticle *particle[2];
    SDVec3 contact_normal;
    SDVec3 particle_movement[2];
    f32 restitution;
    f32 penetration;
};

void sd_particle_contact_resolve(SDParticleContact *pc, f32 dt);

struct SDParticleContactResolver {
    u32 iterations;
    u32 iterations_used;
};

void sd_particle_contact_resolver_resolve_contacts(SDParticleContactResolver *pcr,
                                                   SDParticleContact *contact_array, u32 num_contacts, f32 dt);

enum SDParticleContactGeneratorType {
    SD_PCG_CABLE,
    SD_PCG_ROD,
    SD_PCG_PLANE_Y_ZERO
};

struct SDParticleCable {
    SDParticle *particle[2];
    f32 max_length;
    f32 restitution;
};

struct SDParticleRod {
    SDParticle *particle[2];
    f32 length;
};

struct SDParticlePlaneYZero {
    SDParticle *particle;
};

struct SDParticleContactGenerator {
    SDParticleContactGeneratorType type;
    union {
        SDParticleCable cable;
        SDParticleRod   rod;
        SDParticlePlaneYZero plane_y_zero;
    };
};

u32 sd_particle_contact_generator_add_contact(SDParticleContactGenerator *pcg,
                                              SDParticleContact *pc, u32 limit);
//===================================================================
//===================================================================




//===================================================================
// RigidBody Physics
//===================================================================

struct SDRigidBody {
    f32 inv_mass;
    f32 linear_damping;
    f32 angular_damping;
    
    SDVec3 position;
    SDQuat orientation;

    SDVec3 velocity;
    SDVec3 rotation;

    SDVec3 force_accum;
    SDVec3 torque_accum;
    SDVec3 acceleration;
    SDVec3 last_frame_acceleration;

    // derived data
    SDMat4 transform_matrix;
    SDMat3 inverse_inertia_tensor;
    SDMat3 inverse_inertia_tensor_world;

    bool is_awake;
};

void sd_body_set_mass(SDRigidBody *body, f32 mass);
f32 sd_body_get_mass(SDRigidBody *body);

void sd_body_integrate(SDRigidBody *body, f32 dt);
void sd_body_add_force(SDRigidBody *body, SDVec3 force);
void sd_body_add_force_at_point(SDRigidBody *body, SDVec3 force, SDVec3 point);
void sd_body_add_force_at_body_point(SDRigidBody *body, SDVec3 force, SDVec3 point);

SDVec3 sd_body_get_point_in_world_space(SDRigidBody *body, SDVec3 point);
void sd_body_calculate_derived_data(SDRigidBody *body);

struct SDSpringForceGenerator {
    SDVec3 connection_point;
    SDVec3 other_conection_point;
    SDRigidBody *other;
    f32 spring_constant;
    f32 rest_length;
};

void sd_spring_force_generator_update(SDSpringForceGenerator *fg, SDRigidBody *body, f32 dt);

//===================================================================
//===================================================================


//===================================================================
// Collision Detection
//===================================================================

struct SDBoundingSphere {
    SDVec3 center;
    f32 radius;
};

SDBoundingSphere sd_bounding_sphere_create(SDVec3 center, f32 radius);
SDBoundingSphere sd_bounding_sphere_create(SDBoundingSphere *a, SDBoundingSphere *b);

bool sd_bounding_sphere_overlaps(SDBoundingSphere *a, SDBoundingSphere *b);
f32 sd_bounding_sphere_get_growth(SDBoundingSphere *a, SDBoundingSphere *b);
f32 sd_bounding_sphere_get_volume(SDBoundingSphere *bs);

struct SDPotentialContact {
    SDRigidBody* body[2];
};

struct SDBVHNode {
    SDBVHNode *children[2];
    SDBoundingSphere volume; // use template for this
    SDRigidBody *body;
    SDBVHNode *parent;
};

SDBVHNode *sd_bvh_node_create(SDBlockAllocator *allocator, SDBVHNode *parent, SDBoundingSphere bs, SDRigidBody *body = 0);
bool sd_bvh_node_is_leaf(SDBVHNode *node);
u32 sd_bvh_node_get_potetial_contacts(SDBVHNode *node, SDPotentialContact *contacts, u32 limit);
void sd_bvh_node_insert(SDBlockAllocator *allocator, SDBVHNode *node, SDRigidBody *body, SDBoundingSphere *volume);
void sd_bvh_node_remove(SDBlockAllocator *allocator, SDBVHNode *node);
bool sd_bvh_node_overlaps(SDBVHNode *a, SDBVHNode *b);
//===================================================================
//===================================================================