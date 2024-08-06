#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <immintrin.h>
#include <xmmintrin.h>

#include <sd_software_renderer.h>
#include <sd_platform.h>
#include <sd_memory.h>

static SDMat4 world;
static SDMat4 view;
static SDMat4 proj;
static SDTexture *current_texture;
static SDVec3 view_pos;
   
#include "sd_cpu_renderer_utils.inl"

void sd_clear_back_buffer(f32 r_, f32 g_, f32 b_) {
    u8 a = 0xFF;
    u8 r = (u8)(r_ * 255.0f);
    u8 g = (u8)(g_ * 255.0f);
    u8 b = (u8)(b_ * 255.0f);
    u32 color = (a << 24) | (r << 16) | (g << 8) | b;
    __m256i clear_color = _mm256_set1_epi32(color);
    __m256 zero = _mm256_set1_ps(0.0f);
    for(int i = 0; i < sd_window_width() * sd_window_height(); i += 8) {
        _mm256_store_si256((__m256i *)(sd_back_buffer() + i), clear_color);
        _mm256_store_ps(sd_depth_buffer() + i, zero);
    }
}

void sd_set_world_mat(const SDMat4 &world_) {
    world = world_;
}

void sd_set_view_mat(const SDMat4 &view_) {
    view = view_;
}

void sd_set_proj_mat(const SDMat4 &proj_) {
    proj = proj_;
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
            DrawTriangle(buffer->vertices + j, view_world, proj, current_texture);
        }
    }    
}

void sd_draw_anim_vertex_buffer(SDMat4 *pallete, SDVertexBuffer *buffer) {
    SDMat4 view_world = view * world;
    if(current_texture) {
        for(int j = 0; j < buffer->vertices_count; j += 3) {
            DrawTriangleAnim(pallete, buffer->vertices + j, view_world, proj, current_texture);
        }
    }
}

void sd_draw_line(SDVec3 v0, SDVec3 v1, f32 r, f32 g, f32 b) {
    SDVertex vertices[2];
    vertices[0].pos = v0;
    vertices[1].pos = v1;
    draw_line(vertices, view, proj, r, g, b);
}

SDTexture *sd_create_texture(SDArena *arena, char *path) {
    SDTexture *texture = sd_arena_push_struct(arena, SDTexture);
    texture->data = (u32 *)LoadImage(arena, path, texture->w, texture->h);
    return texture;
}

void sd_set_texture(SDTexture *texture) {
    current_texture = texture;
}