#pragma once

#include <sd_software_renderer.h>

struct SDArena;

struct SDMesh {
    SDVertexBuffer *vbuffer;
};

SDMesh *sd_mesh_create(SDArena *arena, const char *path);
