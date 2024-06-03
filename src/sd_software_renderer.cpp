#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <sd_software_renderer.h>
#include <sd_platform.h>
#include <sd_memory.h>
#include <sd_mesh.h>

static SDMat4 world;
static SDMat4 view;
static SDMat4 proj;
static SDTexture *current_texture;
static SDVec3 view_pos;
   
#include "sd_cpu_renderer_utils.inl"

void sd_clear_back_buffer(f32 r_, f32 g_, f32 b_) {
    uint8_t a = 0xFF;
    uint8_t r = (uint8_t)(r_ * 255.0f);
    uint8_t g = (uint8_t)(g_ * 255.0f);
    uint8_t b = (uint8_t)(b_ * 255.0f);
    uint32_t clear_color = (a << 24) | (r << 16) | (g << 8) | b;
    for(int i = 0; i < sd_window_width() * sd_window_height(); i++)
    {
        sd_back_buffer()[i] = clear_color;
        sd_depth_buffer()[i] = 0.0f;
    }
}

void sd_set_world_mat(SDMat4 *world_) {
    world = *world_;
}

void sd_set_view_mat(SDMat4 *view_) {
    view = *view_;
}

void sd_set_proj_mat(SDMat4 *proj_) {
    proj = *proj_;
}

SDVertexBuffer *sd_create_vertex_buffer(SDArena *arena, SDVertex *vertices, u32 count) {
    SDVertexBuffer *buffer = sd_arena_push_struct(arena, SDVertexBuffer);
    buffer->vertices = sd_arena_push_array(arena, count, SDVertex);
    memcpy(buffer->vertices, vertices, count*sizeof(SDVertex));
    buffer->vertices_count = count;
    return buffer;
}

void sd_draw_vertex_buffer(SDVertexBuffer *buffer, f32 r, f32 g, f32 b) {
    SDMat4 view_world = view * world;

    if(current_texture) {
        for(int j = 0; j < buffer->vertices_count; j += 3) {
            DrawTriangle(buffer->vertices + j, view_world, proj, current_texture, sd_depth_buffer());
        }
    }
    else {
        for(int j = 0; j < buffer->vertices_count; j += 3) {
            DrawTriangle(buffer->vertices + j, view_world, proj, r, g, b, sd_depth_buffer());
        }
    }
    
}

void sd_draw_anim_vertex_buffer(SDAnimator *animator, SDVertexBuffer *buffer) {
    SDMat4 view_world = view * world;
    if(current_texture) {
        for(int j = 0; j < buffer->vertices_count; j += 3) {
            DrawTriangleAnim(animator, buffer->vertices + j, view_world, proj, current_texture, sd_depth_buffer());
        }
    }
}

SDTexture *sd_create_texture(SDArena *arena, char *path) {
    SDTexture *texture = sd_arena_push_struct(arena, SDTexture);
    texture->data = (u32 *)LoadImage(arena, path, texture->w, texture->h);
    return texture;
}

void sd_set_texture(SDTexture *texture) {
    current_texture = texture;
}