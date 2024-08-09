#include <sd_physics.h>
#include <sd_memory.h>


//===================================================================
// Particle Physics
//===================================================================

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
    SDVec3 force = fg->gravity.gravity * sd_particle_get_mass(p);
    sd_particle_add_force(p, force);
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

SDParticleForceRegistry sd_particle_force_registry_create(SDArena *arena, i32 particle_count) {
    SDParticleForceRegistry registry;
    registry.registrations = sd_arena_push_array(arena, particle_count, SDParticleForceRegistration);
    registry.registrations_count = 0;
    registry.registrations_max = particle_count;
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

    //=====================================================================================
    // Resting Contacts
    //=====================================================================================
    // check the velocity build up due to acceleration only
    SDVec3 acc_caused_velocity = pc->particle[0]->acceleration;
    if(pc->particle[1]) acc_caused_velocity -= pc->particle[1]->acceleration;
    f32 acc_caused_sep_velocity = sd_vec3_dot(acc_caused_velocity, pc->contact_normal) * dt;
    
    // if we got a closing velocity due to acceleration build up
    // remove it from the new separating velocity
    if(acc_caused_sep_velocity < 0) {
        new_sep_velocity += pc->restitution * acc_caused_sep_velocity;
        // make sure we havent remove more than was
        // there to remove
        if(new_sep_velocity < 0) {
            new_sep_velocity = 0;
        }
    }
    //=====================================================================================
    //=====================================================================================

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
    // if we dont have any penetration, skip this step
    if(pc->penetration <= 0) {
        return;
    }

    // the movement of each object is based on their inverse mass, so total that
    f32 total_inv_mass = pc->particle[0]->inv_mass;
    if(pc->particle[1]) total_inv_mass += pc->particle[1]->inv_mass;

    // if all particles have infinite mass then do nothing
    if(total_inv_mass <= 0) {
        return;
    }

    // find the amount of penetration per unit of inverse mass
    SDVec3 move_per_inv_mass = pc->contact_normal * (pc->penetration / total_inv_mass);

    // Calculate the movement amount
    pc->particle_movement[0] = move_per_inv_mass * pc->particle[0]->inv_mass;
    if(pc->particle[1]) {
        pc->particle_movement[1] = move_per_inv_mass * -pc->particle[1]->inv_mass;
    }
    else {
        pc->particle_movement[1] = SDVec3();
    }

    // Apply the penetration resolution
    pc->particle[0]->position += pc->particle_movement[0];
    if(pc->particle[1]) {
        pc->particle[1]->position += pc->particle_movement[1];
    }

}

void sd_particle_contact_resolve(SDParticleContact *pc, f32 dt) {
    resolve_velocity(pc, dt);
    resolve_interpenetration(pc, dt);
}


void sd_particle_contact_resolver_resolve_contacts(SDParticleContactResolver *pcr,
                                                   SDParticleContact *contact_array, u32 num_contacts, f32 dt) {
    pcr->iterations_used = 0; 
    while(pcr->iterations_used < pcr->iterations) {
        // find the contact with the largest closing velocity
        f32 max = FLT_MAX;
        u32 max_index = num_contacts;
        for(u32 i = 0; i < num_contacts; i++) {
            f32 sep_vel = calculate_separating_velocity(contact_array + i);
            if(sep_vel < max && sep_vel < 0 || contact_array[i].penetration > 0) {
                max = sep_vel;
                max_index = i;
            }
        }
        // do we have anything worth resolving?
        if(max_index == num_contacts) break;

        // resolve this contact
        sd_particle_contact_resolve(contact_array + max_index, dt);

        // update interpenetration for all particles
        SDVec3 *move = contact_array[max_index].particle_movement; 
        for(int i = 0; i < num_contacts; i++) {
            if(contact_array[i].particle[0] == contact_array[max_index].particle[0]) {
                contact_array[i].penetration -= sd_vec3_dot(move[0], contact_array[i].contact_normal); 
            }
            else if(contact_array[i].particle[0] == contact_array[max_index].particle[1]) {
                contact_array[i].penetration -= sd_vec3_dot(move[1], contact_array[i].contact_normal); 
            }

            if(contact_array[i].particle[1]) {
                if(contact_array[i].particle[1] == contact_array[max_index].particle[0]) {
                    contact_array[i].penetration += sd_vec3_dot(move[0], contact_array[i].contact_normal); 
                }
                else if(contact_array[i].particle[1] == contact_array[max_index].particle[1]) {
                    contact_array[i].penetration += sd_vec3_dot(move[1], contact_array[i].contact_normal); 
                }
            }
        }
         
        pcr->iterations_used++;
    }
}


static f32 get_particle_distance(SDParticle **p) {
    SDVec3 relative_pos = p[0]->position - p[1]->position;
    return sd_vec3_len(relative_pos);
}

static u32 particle_cable_add_contact(SDParticleContactGenerator *pcg,
                               SDParticleContact *pc, u32 limit) {
    SDParticleCable *cable = &pcg->cable;
    f32 length = get_particle_distance(cable->particle);

    // check if  we're over extended
    if(length < cable->max_length) {
        return 0;
    }

    // otherwise return the contact
    pc->particle[0] = cable->particle[0];
    pc->particle[1] = cable->particle[1];

    // calculate the normal
    SDVec3 normal = cable->particle[1]->position - cable->particle[0]->position;
    sd_vec3_normalize(normal);
    pc->contact_normal = normal;
    pc->penetration = length - cable->max_length;
    pc->restitution = cable->restitution;

    return 1;
} 

static u32 particle_rod_add_contact(SDParticleContactGenerator *pcg,
                             SDParticleContact *pc, u32 limit) {
    SDParticleRod *rod = &pcg->rod;
    
    // find the length of the rod
    f32 current_length = get_particle_distance(rod->particle);
 
    // check if  we're over extended
    if(current_length == rod->length) {
        return 0;
    }

    // otherwise return the contact
    pc->particle[0] = rod->particle[0];
    pc->particle[1] = rod->particle[1];

    // calculate the normal
    SDVec3 normal = rod->particle[1]->position - rod->particle[0]->position;
    sd_vec3_normalize(normal);

    // the contact normal depends on whether we are extending or compressing
    if(current_length > rod->length) {
        pc->contact_normal = normal;
        pc->penetration = current_length - rod->length;
    } else {
        pc->contact_normal = normal * -1.0f;
        pc->penetration = rod->length - current_length;
    }

    // always use 0 restitution
    pc->restitution = 0;

    return 1;
}

static u32 particle_plane_y_zero_add_contact(SDParticleContactGenerator *pcg,
                                             SDParticleContact *pc, u32 limit) {
    SDParticlePlaneYZero *plane_y_zero = &pcg->plane_y_zero;
    
    f32 proj_on_y = sd_vec3_dot(plane_y_zero->particle->position, SDVec3(0, 1, 0));

    if(proj_on_y > 0) {
        return 0;
    }

    // otherwise return the contact
    pc->particle[0] = plane_y_zero->particle;
    pc->particle[1] = nullptr;

    pc->contact_normal = SDVec3(0, 1, 0);
    pc->penetration = std::fabs(proj_on_y);
    pc->restitution = 0;

    return 1;
}


u32 sd_particle_contact_generator_add_contact(SDParticleContactGenerator *pcg,
                                              SDParticleContact *pc, u32 limit) {
    switch(pcg->type) {
        case SD_PCG_CABLE: return particle_cable_add_contact(pcg, pc, limit);
        case SD_PCG_ROD: return particle_rod_add_contact(pcg, pc, limit);
        case SD_PCG_PLANE_Y_ZERO: return particle_plane_y_zero_add_contact(pcg, pc, limit);
    }
    return 0;
}
//===================================================================
//===================================================================


//===================================================================
// RigidBody Physics
//===================================================================

static void sd_body_calculate_transform_matrix(SDRigidBody *body) {
    SDMat4 trans = sd_mat4_translation(body->position);
    SDMat4 rot   = sd_quat_to_mat4(body->orientation);
    body->transform_matrix = trans * rot;
}

static void sd_body_transform_inertia_tensor(SDRigidBody *body) {
    SDMat3 world = sd_mat4_to_mat3(body->transform_matrix);
    SDMat3 inv_world = sd_mat3_inverse(world);
    body->inverse_inertia_tensor_world = world * body->inverse_inertia_tensor * inv_world;
}

static void sd_body_clear_accumulators(SDRigidBody *body) {
    body->force_accum = SDVec3();
    body->torque_accum = SDVec3();
}

void sd_body_calculate_derived_data(SDRigidBody *body) {
    sd_quat_normalize(body->orientation);
    sd_body_calculate_transform_matrix(body);
    sd_body_transform_inertia_tensor(body);
}

void sd_body_set_mass(SDRigidBody *body, f32 mass) {
    SD_ASSERT(mass != 0);
    body->inv_mass = 1.0f/mass;
}

f32 sd_body_get_mass(SDRigidBody *body) {
    if(body->inv_mass == 0) {
        return FLT_MAX;
    } else {
        return 1.0f / body->inv_mass;
    }
}

void sd_body_integrate(SDRigidBody *body, f32 dt) {

    if(!body->is_awake) return;

    // calculate linear acceleration from froce inputs
    body->last_frame_acceleration = body->acceleration;
    body->last_frame_acceleration += body->force_accum * body->inv_mass;

    // calculate angular acceleration from torque inputs
    SDVec3 angular_acceleration = body->inverse_inertia_tensor_world * body->torque_accum;

    // adjust velocities
    // update linear velocity from both acceleration and impulses
    body->velocity += body->last_frame_acceleration * dt;
    // update angular velocity from both acceleration and impulses
    body->rotation += angular_acceleration * dt;

    // impose drag
    body->velocity *= powf(body->linear_damping, dt);
    body->rotation *= powf(body->angular_damping, dt);

    // adjust positions
    // update linear position
    body->position += body->velocity * dt;
    sd_quat_add_scale_vec3(body->orientation, body->rotation, dt);


    // normalize the orientation, and update the matrices
    // with the new position and orientation
    sd_body_calculate_derived_data(body);
    // clear accumulators
    sd_body_clear_accumulators(body);
}

void sd_body_add_force(SDRigidBody *body, SDVec3 force) {
    body->force_accum += force;
    body->is_awake = true;
}

void sd_body_add_force_at_point(SDRigidBody *body, SDVec3 force, SDVec3 point) {
    // converto to coords relative to the center of mass
    //SDVec3 pt = point - body->position;
    //pt = body->orientation * pt;
    body->force_accum += force;
    SDMat4 world = sd_mat4_translation(body->position) * sd_quat_to_mat4(body->orientation);
    SDMat4 inv_world = sd_mat4_inverse(world);
    SDVec3 local_point = sd_mat4_transform_point(inv_world, point);
    SDVec3 local_force = sd_mat4_transform_vector(inv_world, force);
    body->torque_accum += sd_vec3_cross(local_point, local_force);
    body->is_awake = true;
}

void sd_body_add_force_at_body_point(SDRigidBody *body, SDVec3 force, SDVec3 point) {
    // get the point in world space
    SDMat4 world = sd_mat4_translation(body->position) * sd_quat_to_mat4(body->orientation);
    SDVec3 pt = sd_body_get_point_in_world_space(body, point);
    sd_body_add_force_at_point(body, force, pt);
    body->is_awake = true;
}


SDVec3 sd_body_get_point_in_world_space(SDRigidBody *body, SDVec3 point) {
    SDMat4 world = sd_mat4_translation(body->position) * sd_quat_to_mat4(body->orientation);
    return sd_mat4_transform_point(world, point);
}

SDVec3 sd_body_get_point_in_local_space(SDRigidBody *body, SDVec3 point) {
    SDMat4 world = sd_mat4_translation(body->position) * sd_quat_to_mat4(body->orientation);
    SDMat4 inv_world = sd_mat4_inverse(world);
    return sd_mat4_transform_point(inv_world, point);
}

void sd_spring_force_generator_update(SDSpringForceGenerator *fg, SDRigidBody *body, f32 dt) {
    SDVec3 lws = sd_body_get_point_in_world_space(body, fg->connection_point);
    SDVec3 ows = sd_body_get_point_in_world_space(fg->other, fg->other_conection_point);
    // calculate the vector of the spring
    SDVec3 force = lws - ows;

    // calculate the magnitude of the force
    f32 magnitude = sd_vec3_len(force);
    magnitude = std::fabs(magnitude - fg->rest_length);
    magnitude *= fg->spring_constant;

    sd_vec3_normalize(force);
    force *= -magnitude;
    sd_body_add_force_at_point(body, force, lws);
}

//===================================================================
//===================================================================



//===================================================================
// Collision Detection
//===================================================================
SDBoundingSphere sd_bounding_sphere_create(SDVec3 center, f32 radius) {
    SDBoundingSphere bs;
    bs.center = center;
    bs.radius = radius;
    return bs;
}

SDBoundingSphere sd_bounding_sphere_create(SDBoundingSphere *a, SDBoundingSphere *b) {
    SDBoundingSphere bs;

    SDVec3 center_offset = b->center - a->center;
    f32 distance = sd_vec3_len_sq(center_offset);
    f32 radius_diff = b->radius - a->radius;

    // Check if the larger sphere encloses the small one
    if(radius_diff*radius_diff >= distance) {
        if(a->radius > b->radius) {
            bs.center = a->center;
            bs.radius = a->radius;
        }
        else {
            bs.center = b->center;
            bs.radius = b->radius;
        }
    }
    // otherwise we need to work with partialy overlapping spheres
    else {
        distance = std::sqrtf(distance);
        bs.radius = (distance + a->radius + b->radius) * 0.5f;
        // the new center is base on a's center moved towards b's
        // center by an amount proportional to the spheres radii
        bs.center = a->center;
        if(distance > 0) {
            bs.center += center_offset * ((bs.radius - a->radius)/distance);
        }
    }
    return bs;
}

bool sd_bounding_sphere_overlaps(SDBoundingSphere *a, SDBoundingSphere *b) {
    f32 distance_sq = sd_vec3_len_sq(a->center - b->center);
    return distance_sq < (a->radius+b->radius)*(a->radius+b->radius);
}

f32 sd_bounding_sphere_get_growth(SDBoundingSphere *a, SDBoundingSphere *b) {
    SDBoundingSphere new_sphere = sd_bounding_sphere_create(a, b);
    // We return a value proportional to the change in surface
    // area of the sphere.
    return new_sphere.radius*new_sphere.radius - a->radius*a->radius;
}

f32 sd_bounding_sphere_get_volume(SDBoundingSphere *bs) {
    return 1.333333f * SD_PI * bs->radius * bs->radius * bs->radius;
}

static void recalculate_bounding_volume(SDBVHNode *node, bool recurse = true) {
    if(sd_bvh_node_is_leaf(node)) {
        return;
    }

    // Use the bounding volume combining create function
    node->volume = sd_bounding_sphere_create(&node->children[0]->volume,
                                             &node->children[1]->volume);
    // recurse up the tree
    if(node->parent) {
        recalculate_bounding_volume(node->parent, true);
    }
}

SDBVHNode *sd_bvh_node_create(SDBlockAllocator *allocator, SDBVHNode *parent, SDBoundingSphere bs, SDRigidBody *body) {
    SDBVHNode *node = (SDBVHNode *)sd_block_allocator_alloc(allocator);
    node->parent = parent;
    node->volume = bs;
    node->body = body;
    node->children[0] = node->children[1] = 0;
    return node;
}

bool sd_bvh_node_is_leaf(SDBVHNode *node) {
    return (node->body != 0);
}

void sd_bvh_node_insert(SDBlockAllocator *allocator, SDBVHNode *node, SDRigidBody *body, SDBoundingSphere *volume) {
    // if we are a leaf, then the only option is to spawn two
    // new children and place the new body in one
    if(sd_bvh_node_is_leaf(node)) {
        // child one is a copy of us
        node->children[0] = sd_bvh_node_create(allocator, node, node->volume, node->body);
        // child two holds the new body
        node->children[1] = sd_bvh_node_create(allocator, node, *volume, body);
        // and we now loose the body
        node->body = 0;
        recalculate_bounding_volume(node);
    }
    // Otherwise we need to work out wich child gets to keep
    // the inserted body. We give it to whoever would grow the
    // least to incorporate it
    else {
        if(sd_bounding_sphere_get_growth(&node->children[0]->volume, volume) <
           sd_bounding_sphere_get_growth(&node->children[1]->volume, volume)) {
            sd_bvh_node_insert(allocator, node->children[0], body, volume);
        }
        else {
            sd_bvh_node_insert(allocator, node->children[1], body, volume);
        }
    }
}

void sd_bvh_node_remove(SDBlockAllocator *allocator, SDBVHNode *node) {
    // if we dont have a parent, then we ignore the sibling
    // processing
    SDBVHNode *parent = node->parent;
    if(parent) {
        // find our sibling
        SDBVHNode *sibling = (parent->children[0] == node) ? parent->children[1] : parent->children[0];
        // write its data to our parent
        parent->volume = sibling->volume;
        parent->body = sibling->body;
        parent->children[0] = sibling->children[0];
        parent->children[1] = sibling->children[1];

        sibling->parent = 0;
        sibling->body = 0;
        sibling->children[0] = 0;
        sibling->children[1] = 0;
        sd_bvh_node_remove(allocator, sibling);

        recalculate_bounding_volume(parent);
    }
    // delete our children (again we remove their
    // parent data so we dont try to process their siblings
    // as they are deleted)
    if(node->children[0]) {
        node->children[0]->parent = 0;
        sd_bvh_node_remove(allocator, node->children[0]);
    }
    if(node->children[1]) {
        node->children[1]->parent = 0;
        sd_bvh_node_remove(allocator, node->children[1]);
    }

    sd_block_allocator_free(allocator, node);
}

bool sd_bvh_node_overlaps(SDBVHNode *a, SDBVHNode *b) {
    return sd_bounding_sphere_overlaps(&a->volume, &b->volume);
}

u32 get_potetial_contacts_with(SDBVHNode *node, SDBVHNode *other, SDPotentialContact *contacts, u32 limit) {
    // early out of we dont overlap or is we have no more room
    // to report contacts
    if(!sd_bvh_node_overlaps(node, other) || limit == 0) {
        return 0;
    }

    // if we are both at leaf nodes, then we have a potential contact
    if(sd_bvh_node_is_leaf(node) && sd_bvh_node_is_leaf(other)) {
        contacts->body[0] = node->body;
        contacts->body[1] = other->body;
        return 1;
    }

    // Determine which node to descend into. if either
    // is a leaf, then we descend the other, if both are branches,
    // the we use the one with the largest size
    if(sd_bvh_node_is_leaf(other) ||
       (sd_bvh_node_is_leaf(node) &&
        sd_bounding_sphere_get_volume(&node->volume) >= sd_bounding_sphere_get_volume(&other->volume))) {
            // resurse into ourself
            u32 count = get_potetial_contacts_with(node->children[0], other, contacts, limit);
            // check we have enough slots to do the other side too
            if(limit > count) {
                return count + get_potetial_contacts_with(node->children[1], other, contacts+count, limit-count);
            }
            else {
                return count;
            }
       }
       else {
            // Recurse into the other node
            u32 count = get_potetial_contacts_with(node, other->children[0], contacts, limit);
            // check we have enough slots to do the other side too
            if(limit > count) {
                return count + get_potetial_contacts_with(node, other->children[1], contacts+count, limit-count);
            }
            else {
                return count;
            }
       }
}

u32 sd_bvh_node_get_potetial_contacts(SDBVHNode *node, SDPotentialContact *contacts, u32 limit) {
    // early out if we dont have room for contacts, or
    // if we are a leaf node
    if(sd_bvh_node_is_leaf(node) || limit == 0) {
        return 0;
    }

    // Get potential contacts of one of our children with
    // the other
    return get_potetial_contacts_with(node->children[0], node->children[1], contacts, limit);
}

//===================================================================
//===================================================================