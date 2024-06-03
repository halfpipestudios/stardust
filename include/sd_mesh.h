#pragma once

#include <sd_software_renderer.h>

struct SDArena;

struct SDKeyPosition {
    SDVec3 pos;
    f32 time_stamp;
};

struct SDKeyRotation {
    SDQuat rot;
    f32 time_stamp;
};

struct SDKeyScale {
    SDVec3 scale;
    f32 time_stamp;
};

struct SDBone {
    SDKeyPosition *positions;
    SDKeyRotation *rotations;
    SDKeyScale    *scales;
    i32 positions_count;
    i32 rotations_count;
    i32 scales_count;

    SDMat4 local_transform;
    SDMat4 offset;
    
    i32 id;
};

struct SDBoneInfo {
    char name[256];
    SDMat4 offset;
};

struct SDStaticMesh {
    SDVertexBuffer *vbuffer;
};

struct SDAnimationNode {
    SDMat4 transformation;
    i32 bone_index;
    i32 children_count;
    SDAnimationNode *children;
};

struct SDAnimation {
    f32 duration;
    i32 ticks_per_second;
    
    SDBone *bones;
    i32 bones_count;

    SDAnimationNode root_node;
};

// CPU base skeletial animation system
struct SDAnimMesh {
    SDVertexBuffer *vbuffer;
};

struct SDAnimator {
    SDMat4 final_bone_matrices[100];
    SDAnimation *current_animation;
    SDAnimMesh *current_mesh;
    f32 current_time;
    f32 delta_time;
};

SDStaticMesh *sd_mesh_create(SDArena *arena, const char *path);
SDAnimMesh *sd_anim_mesh_create(SDArena *arena, const char *path);

SDAnimation *sd_animation_create(SDArena *arena, const char *path);

SDAnimator *sd_animator_create(SDArena *arena, SDAnimMesh *mesh, SDAnimation *animation);
void sd_animator_update(SDAnimator *animator, float dt);
void sd_animator_play(SDAnimator *animator);