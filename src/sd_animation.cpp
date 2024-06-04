#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <sd_platform.h>
#include <sd_math.h>
#include <sd_animation.h>
//==========================================================================================
// Utilities
//==========================================================================================
static inline SDMat4 ai_mat4_to_sd_mat4(aiMatrix4x4 m) {
    return SDMat4(m.a1, m.a2, m.a3, m.a4,
                  m.b1, m.b2, m.b3, m.b4, 
                  m.c1, m.c2, m.c3, m.c4, 
                  m.d1, m.d2, m.d3, m.d4);
}

static inline SDVec3 ai_vec3_to_sd_vec3(aiVector3D m) {
    return SDVec3(m.x, m.y, m.z);
}

static inline SDQuat ai_quat_to_sd_quat(aiQuaternion m) {
    return SDQuat(m.w, m.x, m.y, m.z);
}

static inline f32 get_scale_factor(f32 last_time_stamp, f32 next_time_stamp, f32 animation_time) {
    f32 scale_factor = (animation_time - last_time_stamp) / (next_time_stamp - last_time_stamp);
    return scale_factor;
}

//==========================================================================================
// Bones functions
//==========================================================================================
static SDBone sd_bone_create(SDArena *arena, i32 id, aiNodeAnim *channel) {
    SDBone bone;
    bone.id = id;
    bone.local_transform = SDMat4();
    bone.positions_count = channel->mNumPositionKeys;
    bone.positions = sd_arena_push_array(arena, bone.positions_count, SDKeyPosition);
    for(i32 i = 0; i < bone.positions_count; i++) {
        aiVector3D ai_position = channel->mPositionKeys[i].mValue;
        f32 time_stamp = channel->mPositionKeys[i].mTime;
        SDKeyPosition *key_position = bone.positions + i;
        key_position->pos = ai_vec3_to_sd_vec3(ai_position);
        key_position->time_stamp = time_stamp;
    }

    bone.rotations_count = channel->mNumRotationKeys;
    bone.rotations = sd_arena_push_array(arena, bone.rotations_count, SDKeyRotation);
    for(i32 i = 0; i < bone.rotations_count; i++) {
        aiQuaternion ai_rotation = channel->mRotationKeys[i].mValue;
        f32 time_stamp = channel->mRotationKeys[i].mTime;
        SDKeyRotation *key_rotation = bone.rotations + i;
        key_rotation->rot = ai_quat_to_sd_quat(ai_rotation);
        key_rotation->time_stamp = time_stamp;
    }

    bone.scales_count = channel->mNumScalingKeys;
    bone.scales = sd_arena_push_array(arena, bone.scales_count, SDKeyScale);
    for(i32 i = 0; i < bone.scales_count; i++) {
        aiVector3D ai_scale = channel->mScalingKeys[i].mValue;
        f32 time_stamp = channel->mScalingKeys[i].mTime;
        SDKeyScale *key_scale = bone.scales + i;
        key_scale->scale = ai_vec3_to_sd_vec3(ai_scale);
        key_scale->time_stamp = time_stamp;
    }
    return bone;
}

i32 sd_bone_get_pos_index(SDBone *bone, f32 animation_time) {
    for(i32 i = 0; i < bone->positions_count - 1; i++) {
        if(animation_time < bone->positions[i + 1].time_stamp)
            return i;
    }
    SD_FATAL("Error in the animation system!");
    return 0;
}

i32 sd_bone_get_rot_index(SDBone *bone, f32 animation_time) {
    for(i32 i = 0; i < bone->rotations_count - 1; i++) {
        if(animation_time < bone->rotations[i + 1].time_stamp)
            return i;
    }
    SD_FATAL("Error in the animation system!");
    return 0;
}

i32 sd_bone_get_scale_index(SDBone *bone, f32 animation_time) {
    for(i32 i = 0; i < bone->scales_count - 1; i++) {
        if(animation_time < bone->scales[i + 1].time_stamp)
            return i;
    }
    SD_FATAL("Error in the animation system!");
    return 0; 
}

SDMat4 sd_bone_interpolate_pos(SDBone *bone, f32 animation_time) {
    if(bone->positions_count == 1) {
        SDVec3 pos = bone->positions[0].pos;
        return sd_mat4_translation(pos.x, pos.y, pos.z);
    }
    i32 index0 = sd_bone_get_pos_index(bone, animation_time);
    i32 index1 = index0 + 1;
    f32 scale_factor = get_scale_factor(bone->positions[index0].time_stamp,
                                        bone->positions[index1].time_stamp,
                                        animation_time);
    SDVec3 pos = sd_vec3_lerp(bone->positions[index0].pos, bone->positions[index1].pos, scale_factor);
    return sd_mat4_translation(pos.x, pos.y, pos.z);
}

SDMat4 sd_bone_interpolate_rot(SDBone *bone, f32 animation_time) {
   if(bone->rotations_count == 1) {
        SDQuat rot = sd_quat_normalized(bone->rotations[0].rot);
        return sd_quat_to_mat4(rot);
   }
   i32 index0 = sd_bone_get_rot_index(bone, animation_time);
   i32 index1 = index0 + 1;
   f32 scale_factor = get_scale_factor(bone->rotations[index0].time_stamp,
                                       bone->rotations[index1].time_stamp,
                                       animation_time);
    SDQuat rot = sd_quat_slerp(bone->rotations[index0].rot, bone->rotations[index1].rot, scale_factor);
    rot = sd_quat_normalized(rot);
    return sd_quat_to_mat4(rot);
}

SDMat4 sd_bone_interpolate_scale(SDBone *bone, f32 animation_time) {
    if(bone->scales_count == 1) {
        SDVec3 scale = bone->scales[0].scale;
        return sd_mat4_translation(scale.x, scale.y, scale.z);
    }
    i32 index0 = sd_bone_get_scale_index(bone, animation_time);
    i32 index1 = index0 + 1;
    f32 scale_factor = get_scale_factor(bone->scales[index0].time_stamp,
                                        bone->scales[index1].time_stamp,
                                        animation_time);
    SDVec3 scale = sd_vec3_lerp(bone->scales[index0].scale, bone->scales[index1].scale, scale_factor);
    return sd_mat4_translation(scale.x, scale.y, scale.z); 
}

inline void sd_bone_update(SDBone *bone, f32 animation_time) {
    SDMat4 translation = sd_bone_interpolate_pos(bone, animation_time);
    SDMat4 rotation = sd_bone_interpolate_rot(bone, animation_time);
    SDMat4 scale = sd_bone_interpolate_scale(bone, animation_time);
    bone->local_transform = translation * rotation * scale;
}

//==========================================================================================
// Animation functions
//==========================================================================================

SDAnimation *sd_animation_create(SDArena *arena, const char *path) {
    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(path, aiProcess_Triangulate);
    if(!scene) {
        SD_FATAL("Error: (Assimp) %s", import.GetErrorString());
    }
    aiAnimation *ai_animation = scene->mAnimations[0];

    SDAnimation *animation = sd_arena_push_struct(arena, SDAnimation);
    animation->duration = ai_animation->mDuration;
    animation->ticks_per_second = (i32)ai_animation->mTicksPerSecond;
    animation->bones_count = ai_animation->mNumChannels;
    animation->bones = sd_arena_push_array(arena, animation->bones_count, SDBone);

    for(i32 i = 0; i < ai_animation->mNumChannels; i++) {
        animation->bones[i] = sd_bone_create(arena, i, ai_animation->mChannels[i]);
    }

    return animation;
}


//==========================================================================================
// Skeleton functions
//==========================================================================================

static i32 find_bone_info_index(aiBone **bones, i32 count, const char *name) {
    for(i32 i = 0; i < count; i++) {
        aiBone *bone = bones[i];
        if(strncmp(bone->mName.C_Str(), name, strlen(name)) == 0) return i;
    }
    return -1;
}

static void fill_skeleton_node(SDArena *arena, SDSkeletonNode *dst, aiNode *src, aiBone **bones, i32 count) {
    dst->bone_index = find_bone_info_index(bones, count, src->mName.data);
    dst->transformation = ai_mat4_to_sd_mat4(src->mTransformation);
    dst->inv_bind_pose = SDMat4();
    if(dst->bone_index >= 0) {
        dst->inv_bind_pose = ai_mat4_to_sd_mat4(bones[dst->bone_index]->mOffsetMatrix);   
    }
    dst->children_count = src->mNumChildren;
    if(dst->children_count > 0) {
        dst->children = sd_arena_push_array(arena, dst->children_count, SDSkeletonNode);
        for(i32 i = 0; i < dst->children_count; i++) {
            fill_skeleton_node(arena, dst->children + i, src->mChildren[i], bones, count);
        }
    }
}

SDSkeleton *sd_skeleton_create(SDArena *arena, const char *path) {
    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(path, aiProcess_Triangulate);
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        SD_FATAL("Error: (Assimp) %s", import.GetErrorString());
    }
    aiMesh *mesh = scene->mMeshes[0];

    SDSkeleton *skeleton = sd_arena_push_struct(arena, SDSkeleton);
    skeleton->current_time = 0.0f;
    skeleton->delta_time = 0.0f;

    fill_skeleton_node(arena, &skeleton->root_node, scene->mRootNode, mesh->mBones, mesh->mNumBones);

    // init the pallete matrix to identity matrices
    for(i32 i = 0; i < 100; i++) {
        skeleton->final_bone_matrices[i] = SDMat4();
    }

    return skeleton;
}

static void calculate_bone_transform(SDSkeleton *skeleton, SDAnimation *animation, SDSkeletonNode *node, SDMat4 parent_transform) {
    SDMat4 node_transform = node->transformation;
    SDBone *bone = node->bone_index >= 0 ? animation->bones + node->bone_index : nullptr;
    SDMat4 global_transform;
    if(bone) {
        sd_bone_update(bone, skeleton->current_time);
        node_transform = bone->local_transform;
        global_transform = parent_transform * node_transform;
        skeleton->final_bone_matrices[bone->id] = global_transform * node->inv_bind_pose;
    } else {
        global_transform = parent_transform * node_transform;
    }

    for(i32 i = 0; i < node->children_count; i++) {
        calculate_bone_transform(skeleton, animation, node->children + i, global_transform);
    }
}

void sd_skeleton_animate(SDSkeleton *skeleton, SDAnimation *animation, f32 dt) {
    skeleton->delta_time = dt;
    skeleton->current_time += animation->ticks_per_second * dt;
    skeleton->current_time = fmod(skeleton->current_time, animation->duration);
    calculate_bone_transform(skeleton, animation, &skeleton->root_node, SDMat4());
}

