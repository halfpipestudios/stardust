#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <sd_platform.h>
#include <sd_math.h>
#include <sd_mesh.h>

SDMesh *sd_mesh_create(SDArena *arena, const char *path) {
    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        SD_FATAL("Error: (Assimp) %s", import.GetErrorString());
    }
    aiMesh *mesh = scene->mMeshes[0];

    SDArena *scratch = sd_get_scratch_arena(0);

    u32 vertices_count = mesh->mNumVertices;
    SDVertex *vertices = sd_arena_push_array(scratch, vertices_count, SDVertex);
    
    for(u32 i = 0; i < mesh->mNumVertices; i++) {
        SDVertex *vertex = vertices + i;
        vertex->pos.x = mesh->mVertices[i].x;
        vertex->pos.y = mesh->mVertices[i].y;
        vertex->pos.z = mesh->mVertices[i].z;

        vertex->nor.x = mesh->mNormals[i].x;
        vertex->nor.y = mesh->mNormals[i].y;
        vertex->nor.z = mesh->mNormals[i].z;

        vertex->uvs.x = mesh->mTextureCoords[0][i].x;
        vertex->uvs.y = mesh->mTextureCoords[0][i].y;

        for(u32 j = 0; j < MAX_BONE_INFLUENCE; j++) {
            vertex->bone_id[j] = -1;
            vertex->weights[j] = 0.0f;
        }
    }
    
    for(i32 bone_index = 0; bone_index < mesh->mNumBones; ++bone_index) {
        
        auto weights = mesh->mBones[bone_index]->mWeights;
        i32 weights_count = mesh->mBones[bone_index]->mNumWeights;

        for(i32 weight_index = 0; weight_index < weights_count; ++weight_index) {

            i32 vertex_id = weights[weight_index].mVertexId;
            f32 weight = weights[weight_index].mWeight;
            SD_ASSERT(vertex_id < vertices_count);

            for(i32 i = 0; i < 4; ++i) {
                if(vertices[vertex_id].bone_id[i] < 0) {
                    vertices[vertex_id].weights[i] = weight;
                    vertices[vertex_id].bone_id[i] = bone_index;
                    break;
                }
            }
        }
    }

    SDMesh *result = sd_arena_push_struct(arena, SDMesh);
    result->vbuffer = sd_create_vertex_buffer(arena, vertices, vertices_count);
    sd_arena_clear(scratch);
    return result;
}