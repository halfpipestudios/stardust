#pragma once

#include <sd_common.h>
#include <sd_math.h>

#define MAX_BONE_INFLUENCE 4

struct SDArena;

struct SDVertex {
    SDVec3 pos;
    SDVec3 nor;
    SDVec2 uvs;
    i32 bone_id[MAX_BONE_INFLUENCE];
    f32 weights[MAX_BONE_INFLUENCE];
};

struct SDVertexBuffer {
    SDVertex *vertices;
    u32 vertices_count;
};

struct SDTexture {
    i32 w, h;
    u32 *data;
};

void sd_clear_back_buffer(f32 r, f32 g, f32 b);
void sd_set_world_mat(SDMat4 &world);
void sd_set_view_mat(SDMat4 &view);
void sd_set_proj_mat(SDMat4 &proj);

SDVertexBuffer *sd_create_vertex_buffer(SDArena *arena, SDVertex *vertices, u32 count);
SDTexture *sd_create_texture(SDArena *arena, char *path);
void sd_set_texture(SDTexture *texture);

void sd_draw_vertex_buffer(SDVertexBuffer *buffer, f32 r, f32 g, f32 b);
void sd_draw_anim_vertex_buffer(SDMat4 *pallete, SDVertexBuffer *buffer);
void sd_draw_line(SDVec3 v0, SDVec3 v1, f32 r, f32 g, f32 b);
