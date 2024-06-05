#pragma once

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
    i32 id;
};

struct SDAnimation {
    f32 duration;
    i32 ticks_per_second;
    SDBone *bones;
    i32 bones_count;
};

struct SDSkeletonNode {
    SDMat4 transformation;
    SDMat4 inv_bind_pose;
    i32 bone_index;
    
    SDSkeletonNode *children;
    i32 children_count;
};

struct SDSkeleton {
    SDMat4 final_bone_matrices[100];
    f32 current_time[10];
    f32 delta_time;
    SDSkeletonNode root_node;
};

SDSkeleton *sd_skeleton_create(SDArena *arena, const char *path);
SDAnimation *sd_animation_create(SDArena *arena, const char *path);
void sd_skeleton_animate(SDSkeleton *skeleton, SDAnimation *animation, f32 dt);
void sd_skeleton_interpolate_animations(SDSkeleton *skeleton, SDAnimation *a, SDAnimation *b, f32 t, f32 dt);
void sd_skeleton_interpolate_4_animations(SDSkeleton *skeleton, SDAnimation *a, SDAnimation *b, SDAnimation *c, SDAnimation *d, f32 in_t, f32 dt);
