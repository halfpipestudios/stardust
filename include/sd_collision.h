#pragma once
#include <sd_physics.h>

struct SDContact {
    // holds the position of the contact in world space
    SDVec3 contact_point;
    // holds the direction of the contact in world space
    SDVec3 contact_normal;
    // hold the depth of the penetration
    f32 penetration;
};

struct SDCollisionData {
    // holds the contact array to write into
    SDContact *contacts;
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


u32 collision_detector_box_plane(SDBox *box, SDPlane *plane, SDCollisionData *data);
