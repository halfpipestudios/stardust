#include <sd_collision.h>
#include <sd_platform.h>
void sd_collision_detector_sphere_plane(SDSphere *sphere, SDPlane *plane, SDCollisionData *data) {
    SDVec3 position = sphere->body->position;
    f32 ball_distance = sd_vec3_dot(plane->normal, position) - sphere->radius - plane->offset;
    if(ball_distance >= 0) {
        return;
    }

    SDContact *contact = data->contact_array + data->contact_count++;
    contact->penetration = -ball_distance;
    contact->contact_point = position - plane->normal * (ball_distance + sphere->radius);
    contact->contact_normal = plane->normal;// sd_vec3_normalized(position - contact->contact_point);
    contact->body[0] = sphere->body;
    contact->body[1] = nullptr;
}

void sd_collision_detector_box_plane(SDBox *box, SDPlane *plane, SDCollisionData *data) {

    // TODO: check for intersection as a early out

    SDVec3 vertices[8] = {
        SDVec3(-box->half_size.x, -box->half_size.y, -box->half_size.z),
        SDVec3(-box->half_size.x, -box->half_size.y, +box->half_size.z),
        SDVec3(-box->half_size.x, +box->half_size.y, -box->half_size.z),
        SDVec3(-box->half_size.x, +box->half_size.y, +box->half_size.z),
        SDVec3(+box->half_size.x, -box->half_size.y, -box->half_size.z),
        SDVec3(+box->half_size.x, -box->half_size.y, +box->half_size.z),
        SDVec3(+box->half_size.x, +box->half_size.y, -box->half_size.z),
        SDVec3(+box->half_size.x, +box->half_size.y, +box->half_size.z)
    };

    for(i32 i = 0; i < 8; i++) {
        SDVec3 vertex = sd_mat4_transform_point(box->transform, vertices[i]);

        // calculate the distance from the plane
        f32 vertex_distance = sd_vec3_dot(vertex, plane->normal);
        if(vertex_distance <= plane->offset) {
            // create the contact data
            SDContact *contact = data->contact_array + data->contact_count++;
            contact->contact_point = plane->normal;
            contact->contact_point *= (vertex_distance - plane->offset);
            contact->contact_point += vertex;
            contact->contact_normal = plane->normal;
            contact->penetration = plane->offset - vertex_distance;
            contact->body[0] = box->body;
            contact->body[1] = nullptr;
        }
    }
}

static void swap_bodies(SDContact *contact) {
    contact->contact_normal *= -1.0f;
    SDRigidBody *temp = contact->body[0];
    contact->body[0] = contact->body[1];
    contact->body[1] = temp;
}

static SDVec3 calculate_local_velocity(SDContact *contact, u32 body_index, f32 duration) {
    SDRigidBody *this_body = contact->body[body_index];

    // work out the velocity of the contact point
    SDVec3 velocity = sd_vec3_cross(this_body->rotation, contact->relative_contact_position[body_index]);
    velocity += this_body->velocity;

    // turn the velocity into contact coords
    SDVec3 contact_velocity = sd_mat3_transposed(contact->contact_to_world) * velocity;

    // calculate the amount of velocity that is due to force without
    // reactions
    SDVec3 acc_velocity = this_body->last_frame_acceleration * duration;

    // calculate the velocity in contact coords
    acc_velocity = sd_mat3_transposed(contact->contact_to_world) * acc_velocity;

    // we ignore any component of acceleration in the contact normal
    // direction, we are only interested in planar acceleration
    acc_velocity.x = 0;

    contact_velocity += acc_velocity;

    return contact_velocity;
}

static f32 calculateDesiredDeltaVelocity(SDContact *contact, f32 duration) {
    const static f32 velocity_limit = 0.25f;

    // calculate the acceleration induce velocity accumulated this frame
    f32 velocity_from_acc = 0;
    if(contact->body[0]->is_awake) {
        velocity_from_acc += sd_vec3_dot(contact->body[0]->last_frame_acceleration, contact->contact_normal * duration);
    }
    if(contact->body[1] && contact->body[1]->is_awake) {
        velocity_from_acc -= sd_vec3_dot(contact->body[1]->last_frame_acceleration , contact->contact_normal * duration);
    }
    // is the velocity is very slow limit the restitution
    f32 this_restitution = 0.4f;
    if(fabsf(contact->contact_velocity.x) < velocity_limit) {
        this_restitution = 0.0f;
    }

    return -contact->contact_velocity.x - this_restitution * (contact->contact_velocity.x - velocity_from_acc);
}

static void prepate_contacts(SDContact *contacts, u32 contacts_count, f32 duration) {
    for(i32 i = 0; i < contacts_count; i++) {
        SDContact *contact = contacts + i;

        if(!contact->body[0]) {
            swap_bodies(contact);
        }

        contact->contact_to_world = sd_mat3_orthonormal_basis(contact->contact_normal);

        // store the relative position of the contact relative to each body
        contact->relative_contact_position[0] = contact->contact_point - contact->body[0]->position;
        if(contact->body[1]) {
            contact->relative_contact_position[1] = contact->contact_point - contact->body[1]->position;
        }

        // find the relative velocity of the bodies at the contact point
        contact->contact_velocity = calculate_local_velocity(contact, 0, duration);
        if(contact->body[1]) {
            contact->contact_velocity -= calculate_local_velocity(contact, 1, duration);
        }

        // calculate the desired change in velocity for resolution;
        contact->desired_delta_velocity = calculateDesiredDeltaVelocity(contact, duration);
    }
}

static void apply_position_change(SDContact *contact, SDVec3 linear_change[2], SDVec3 angular_change[2], f32 penetration) {
    const f32 angular_limit = 0.2f;
    f32 angular_move[2];
    f32 linear_move[2];

    f32 total_inertia = 0;
    f32 linear_inertia[2];
    f32 angular_inertia[2];

    // we need to work out the inertia of each object in the direction
    // of the contact normal, due to angular inertia only
    for(u32 i = 0; i < 2; i++) {
        if(contact->body[i]) {
            SDMat3 inverse_inertia_tensor = contact->body[i]->inverse_inertia_tensor_world;

            // calculate velocity change to work out the angular inertia
            SDVec3 angular_inertia_world = sd_vec3_cross(contact->relative_contact_position[i], contact->contact_normal);
            angular_inertia_world = inverse_inertia_tensor * angular_inertia_world;
            angular_inertia_world = sd_vec3_cross(angular_inertia_world, contact->relative_contact_position[i]);
            angular_inertia[i] = sd_vec3_dot(angular_inertia_world, contact->contact_normal);

            // the linear component is simply the inverse mass'
            linear_inertia[i] = contact->body[i]->inv_mass;

            // keep track of the total inertia for all components
            total_inertia += linear_inertia[i] + angular_inertia[i];
        }
    }

    // loop through again calculating and applying the changes
    for(u32 i = 0; i < 2; i++) if (contact->body[i]) {

        f32 sign = (i==0)?1.0f:-1.0f;
        angular_move[i] = sign * penetration * (angular_inertia[i] / total_inertia);
        linear_move[i] = sign * penetration * (linear_inertia[i] / total_inertia);

        // to avoid angular projections that are too great limit the angular move
        SDVec3 projection = contact->relative_contact_position[i];
        projection += contact->contact_normal * -sd_vec3_dot(contact->relative_contact_position[i], contact->contact_normal);

        // use the small angle approximation for the sine of the angle (i.e.
        // the magnitude would be sine(angularLimit) * projection.magnitude
        // but we approximate sine(angularLimit) to angularLimit).
        f32 max_magnitude = angular_limit * sd_vec3_len(projection);

        if(angular_move[i] < -max_magnitude) {
            f32 total_move = angular_move[i] + linear_move[i];
            angular_move[i] = -max_magnitude;
            linear_move[i] = total_move - angular_move[i];
        }
        else if(angular_move[i] > max_magnitude) {
            f32 total_move = angular_move[i] + linear_move[i];
            angular_move[i] = max_magnitude;
            linear_move[i] = total_move - angular_move[i];
        }

        // we have the linear amount of movement required by turning
        // the rigid body. we now need to calculate the desired rotation to achive that
        if(angular_move[i] == 0) {
            // easy case - no angular means no rotation
            angular_change[i] = SDVec3();
        }
        else {
            // work out the direction we like to rotate in
            SDVec3 target_angular_direction = sd_vec3_cross(contact->relative_contact_position[i], contact->contact_normal);
            SDMat3 inverse_inertia_tensor = contact->body[i]->inverse_inertia_tensor_world;
            // Work out the direction we'd need to rotate to achieve that
            angular_change[i] = (inverse_inertia_tensor * target_angular_direction) * (angular_move[i] / angular_inertia[i]);
        }
        // velocity change is easy it is just the linear movement along the contact normal
        linear_change[i] = contact->contact_normal * linear_move[i];


        // now we can start to apply the values we calculate
        // apply linear movement
        SDVec3 pos = contact->body[i]->position;
        pos += contact->contact_normal * linear_move[i];
        contact->body[i]->position = pos;

        // and the change in orientation
        SDQuat q = contact->body[i]->orientation;
        sd_quat_add_scale_vec3(q, angular_change[i], 1.0f);
        contact->body[i]->orientation = q;

        // We need to calculate the derived data for any body that is
        // asleep, so that the changes are reflected in the object's
        // data. Otherwise the resolution will not change the position
        // of the object, and the next collision detection round will
        // have the same penetration.
        sd_body_calculate_derived_data(contact->body[i]);

    }
}

static void resolve_interpenetration(SDCollisionResolver *cr, SDContact *c, u32 contacts_count, f32 duration) {
    u32 i, index;
    SDVec3 linear_change[2];
    SDVec3 angular_change[2];
    f32 max;
    SDVec3 delta_position;

    cr->position_iterations_used = 0;
    while(cr->position_iterations_used < cr->position_iterations) {
        // find the biggest penetration
        max = cr->position_epsilon;
        index = contacts_count;
        for(i = 0; i < contacts_count; i++) {
            if(c[i].penetration > max) {
                max = c[i].penetration;
                index = i;
            }
        }
        if(index == contacts_count) {
            break;
        }

        // resolve the penetration
        apply_position_change(c + index, linear_change, angular_change, max);


        // this action may have change the penetration of other
        // bodies, so we update contacts
        for(i = 0; i < contacts_count; i++) {
            // check each body in the contact
            for(u32 b = 0; b < 2; b++)  if (c[i].body[b]) {
                // Check for a match with each body in the newly
                // resolved contact
                for (unsigned d = 0; d < 2; d++) {
                    if (c[i].body[b] == c[index].body[d]) {
                        delta_position = linear_change[d] + sd_vec3_cross(angular_change[d], c[i].relative_contact_position[b]);
                        // The sign of the change is positive if we're
                        // dealing with the second body in a contact
                        // and negative otherwise (because we're
                        // subtracting the resolution)..
                        c[i].penetration += sd_vec3_dot(delta_position, c[i].contact_normal) * (b?1.0f:-1.0f);
                    }
                }
            }
        }
        cr->position_iterations_used++;
    }
}

static SDVec3 calculate_friction_impulse(SDContact *contact, SDMat3 *inverse_inertia_tensor) {
    SDVec3 impulse_contact;

    f32 inverse_mass = contact->body[0]->inv_mass;

    SDMat3 impulse_to_torque = sd_mat3_skew_symmetric(contact->relative_contact_position[0]);

    SDMat3 delta_vel_world = impulse_to_torque;
    delta_vel_world = delta_vel_world * inverse_inertia_tensor[0];
    delta_vel_world = delta_vel_world * impulse_to_torque;
    delta_vel_world = delta_vel_world * -1.0f;

    if(contact->body[1]) {
        impulse_to_torque = sd_mat3_skew_symmetric(contact->relative_contact_position[1]);
        SDMat3 delta_vel_world2 = impulse_to_torque;
        delta_vel_world2 = delta_vel_world2 * inverse_inertia_tensor[1];
        delta_vel_world2 = delta_vel_world2 * impulse_to_torque;
        delta_vel_world2 = delta_vel_world2 * -1.0f;

        delta_vel_world = delta_vel_world + delta_vel_world2;

        inverse_mass += contact->body[1]->inv_mass;

    }

    SDMat3 delta_velocity = sd_mat3_transposed(contact->contact_to_world);
    delta_velocity = delta_velocity * delta_vel_world;
    delta_velocity = delta_velocity * contact->contact_to_world;

    delta_velocity.m[0][0] += inverse_mass;
    delta_velocity.m[1][1] += inverse_mass;
    delta_velocity.m[2][2] += inverse_mass;

    SDMat3 impulse_matrix = sd_mat3_inverse(delta_velocity);

    SDVec3 vel_kill = SDVec3(contact->desired_delta_velocity, -contact->contact_velocity.y, -contact->contact_velocity.z);

    impulse_contact = impulse_matrix * vel_kill;

    f32 planar_impulse = std::sqrtf(impulse_contact.y*impulse_contact.y + impulse_contact.z*impulse_contact.z);
    if(planar_impulse > impulse_contact.x * contact->friction) {
        impulse_contact.y /= planar_impulse;
        impulse_contact.z /= planar_impulse;

        impulse_contact.x = delta_velocity.m[0][0] +
        delta_velocity.m[0][1]*contact->friction*impulse_contact.y +
        delta_velocity.m[0][2]*contact->friction*impulse_contact.z;
        impulse_contact.x = contact->desired_delta_velocity / impulse_contact.x;
        impulse_contact.y *= contact->friction * impulse_contact.x;
        impulse_contact.z *= contact->friction * impulse_contact.x;

    }


    return impulse_contact;
}

static SDVec3 calculate_frictionless_impulse(SDContact *contact, SDMat3 *inverse_inertia_tensor) {
    SDVec3 impulse_contact;
    // build a vector that shows the change in velocity in
    // world space for a unit impulse in the direction of the contact normal
    SDVec3 delta_vel_world = sd_vec3_cross(contact->relative_contact_position[0], contact->contact_normal);
    delta_vel_world = inverse_inertia_tensor[0] * delta_vel_world;
    delta_vel_world = sd_vec3_cross(delta_vel_world, contact->relative_contact_position[0]);

    // work out the change in velocity in contact coordinate
    //f32 delta_velocity = sd_vec3_dot(delta_vel_world, contact->contact_normal);
    f32 delta_velocity = (sd_mat3_transposed(contact->contact_to_world) * delta_vel_world).x;

    // add the linear component of velocity change
    delta_velocity += contact->body[0]->inv_mass;

    // chack if we need the second body data
    if(contact->body[1]) {
        SDVec3 delta_vel_world = sd_vec3_cross(contact->relative_contact_position[1], contact->contact_normal);
        delta_vel_world = inverse_inertia_tensor[1] * delta_vel_world;
        delta_vel_world = sd_vec3_cross(delta_vel_world, contact->relative_contact_position[1]);

        // work out the change in velocity in contact coordinate
        //delta_velocity += sd_vec3_dot(delta_vel_world, contact->contact_normal);
        delta_velocity += (sd_mat3_transposed(contact->contact_to_world) * delta_vel_world).x;

        // add the linear component of velocity change
        delta_velocity += contact->body[1]->inv_mass;
    }
    impulse_contact.x = contact->desired_delta_velocity / delta_velocity;
    impulse_contact.y = 0;
    impulse_contact.z = 0;
    return impulse_contact;
}

static void apply_velocity_change(SDContact *contact, SDVec3 velocity_change[2], SDVec3 rotation_change[2]) {
    // get hold of the inverse mass and the inverse inertia tensor both in world coords

    SDMat3 inverse_inertia_tensor[2];
    inverse_inertia_tensor[0] = contact->body[0]->inverse_inertia_tensor_world;
    if(contact->body[1]) {
        inverse_inertia_tensor[1] = contact->body[1]->inverse_inertia_tensor_world;
    }

    // we will calculate the impulse for each contact axis
    SDVec3 impulse_contact;
    contact->friction = 1.4f;
    //if(contact->friction == 0.0f) {
    //    impulse_contact = calculate_frictionless_impulse(contact, inverse_inertia_tensor);
    //}
    //else {
        impulse_contact = calculate_friction_impulse(contact, inverse_inertia_tensor);
    //}

    // Convert impulse to world coords
    SDVec3 impulse = contact->contact_to_world * impulse_contact;
    //sd_body_add_force_at_point(contact->body[0], impulse, contact->contact_point);

    // split the impulse into linear and rotational component
    SDVec3 impulsive_torque = sd_vec3_cross(contact->relative_contact_position[0], impulse);
    rotation_change[0] = inverse_inertia_tensor[0] * impulsive_torque;

    velocity_change[0] = SDVec3();
    velocity_change[0] += impulse * contact->body[0]->inv_mass;


    contact->body[0]->velocity += velocity_change[0];
    contact->body[0]->rotation += rotation_change[0];

    if(contact->body[1]) {
        SDVec3 impulsive_torque = sd_vec3_cross(impulse, contact->relative_contact_position[1]);
        rotation_change[1] = inverse_inertia_tensor[1] * impulsive_torque;
        velocity_change[1] = SDVec3();
        velocity_change[1] += impulse * -contact->body[1]->inv_mass;

        contact->body[1]->velocity += velocity_change[1];
        contact->body[1]->rotation += rotation_change[1];
    }
}

static void resolve_velocities(SDCollisionResolver *cr, SDContact *c, u32 contacts_count, f32 duration) {
    SDVec3 velocity_change[2]{};
    SDVec3 rotation_change[2]{};
    SDVec3 delta_vel;

    // iteratively handle impacts in order of severity
    cr->velocity_iterations_used = 0;
    while(cr->velocity_iterations_used < cr->velocity_iterations) {
        // find contact with maximun magnitude of probable velocity change
        f32 max = cr->velocity_epsilon;
        u32 index = contacts_count;
        for(u32 i = 0; i < contacts_count; i++) {
            if(c[i].desired_delta_velocity > max) {
                max = c[i].desired_delta_velocity;
                index = i;
            }
        }
        if(index == contacts_count) {
            break;
        }

        // do the resolution on the contact that came out top
        apply_velocity_change(c + index, velocity_change, rotation_change);
        // with the change in velocity of the two bodies, the update of
        // contact velocities means that some of the relative closing
        // velocities need recomputing

        for(u32 i = 0; i < contacts_count; i++) {
            // check for each body in the contact
            for(u32 b = 0; b < 2; b++) if(c[i].body[b]) {
                // check for a match with each body in the newly
                // resolve contact
                for(u32 d = 0; d < 2; d++) {
                    if(c[i].body[b] == c[index].body[d]) {
                        delta_vel = velocity_change[d] + sd_vec3_cross(rotation_change[d], c[i].relative_contact_position[b]);

                        // the sign of the change is negative if we are dealing
                        // with the second body ina contact
                        c[i].contact_velocity += (sd_mat3_transposed(c[i].contact_to_world) * delta_vel) * (b?-1.0f:1.0f);
                        c[i].desired_delta_velocity = calculateDesiredDeltaVelocity(c + i, duration);
                    }
                }

            }
        }


        cr->velocity_iterations_used++;
    }

}

void sd_collision_resolver(SDCollisionResolver *cr, SDContact *contacts, u32 contacts_count, f32 duration) {
    // TODO: Prepare Contact Data
    prepate_contacts(contacts, contacts_count, duration);
    // TODO: Resolve interpenetration problems with the contacts
    resolve_interpenetration(cr, contacts, contacts_count, duration);
    // TODO: Resolve the velocity problems with the contacts
    resolve_velocities(cr, contacts, contacts_count, duration);
}