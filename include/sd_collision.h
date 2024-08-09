#pragma once
#include <sd_physics.h>

struct SDContact {
    // holds the bodies that are involved in the contact
    SDRigidBody *body[2];
    // holds the position of the contact in world space
    SDVec3 contact_point;
    // holds the direction of the contact in world space
    SDVec3 contact_normal;
    // hold the depth of the penetration
    f32 penetration;

    // a transform matrix that converts coordinates in the
    // contact's frame of reference to world coords
    SDMat3 contact_to_world;

    // holds the closing velocity at the point of contact
    // update before the resolution algorithm
    SDVec3 contact_velocity;
    // holds the required change in velocity for this contact to be resolve
    f32 desired_delta_velocity;

    // holds the world space position of the contact point relative to
    // center of each body. this is set before the resolution algorithm
    SDVec3 relative_contact_position[2];

    f32 friction{0.5f};
};

struct SDCollisionData {
    // holds the contact array to write into
    SDContact *contact_array;
    u32 contact_count;
    i32 contacts_left;
};

struct SDPrimitive {
    SDRigidBody *body;
    SDMat4 transform;
};

struct SDPlane : public SDPrimitive {
    SDVec3 normal;
    f32 offset;
};

struct SDSphere : public SDPrimitive {
    f32 radius;
};

struct SDBox : public SDPrimitive {
    SDVec3 half_size;
};


struct SDCollisionResolver {
    u32 velocity_iterations {2048};
    u32 position_iterations {2048};
    f32 velocity_epsilon {0.01f}; // 0.01
    f32 position_epsilon {0.01f}; // 0.01
    u32 velocity_iterations_used {0};
    u32 position_iterations_used {0};
};

void sd_collision_detector_sphere_plane(SDSphere *sphere, SDPlane *plane, SDCollisionData *data);
void sd_collision_detector_box_plane(SDBox *box, SDPlane *plane, SDCollisionData *data);

void sd_collision_resolver(SDCollisionResolver *cr, SDContact *contacts, u32 contacts_count, f32 duration);