static void *LoadImage(SDArena *arena, const char *path, int32_t& outWidth, int32_t& outHeight)
{   
    i32 channels = 4;
    void *data = (void *)stbi_load(path, &outWidth, &outHeight, &channels, 0);
       
    u32 *pixels = (u32 *)data;
    u32 *result = (u32 *)sd_arena_push_size(arena, outWidth * outHeight * sizeof(u32));
    for(i32 i = 0; i < outWidth * outHeight; i++) {
        u32 color = pixels[i];
        u8 a = (u8)(color >> 24);
        u8 b = (u8)(color >> 16);
        u8 g = (u8)(color >> 8);
        u8 r = (u8)(color >> 0);
        result[i] = (a << 24) | (r << 16) | (g << 8) | b;
    }

    stbi_image_free(data);
    return result;
}


static SDVec3 Barycentric(const SDVec3& a, const SDVec3& b, const SDVec3& c, const SDVec3& p) {
    SDVec3 v0 = b - a;
    SDVec3 v1 = c - a;
    SDVec3 v2 = p - a;

    float d00 = sd_vec3_dot(v0, v0);
    float d10 = sd_vec3_dot(v1, v0);
    float d11 = sd_vec3_dot(v1, v1);
    float d20 = sd_vec3_dot(v2, v0);
    float d21 = sd_vec3_dot(v2, v1);

    float denom = (d00 * d11) - (d10 * d10);

    float y = ((d20 * d11) - (d10 * d21)) / denom;
    float z = ((d00 * d21) - (d20 * d10)) / denom;
    float x = 1.0f - y - z;
    
    return SDVec3(x, y, z);
}

static uint32_t RgbToUint32(float r_, float g_, float b_, float a_) {
    uint8_t a = (uint8_t)(a_ * 255.0f);
    uint8_t r = (uint8_t)(r_ * 255.0f);
    uint8_t g = (uint8_t)(g_ * 255.0f);
    uint8_t b = (uint8_t)(b_ * 255.0f);
    uint32_t clearColor = (a << 24) | (r << 16) | (g << 8) | b;
    return clearColor;
}

static void HomogenousClipping(Vec4 *srcVertices, SDVec2 *srcUvs, int32_t srcCount, 
                                Vec4 *dstVertices, SDVec2 *dstUvs, int32_t *dstCount,
                                int32_t index, float sign) {
    *dstCount = 0;

    Vec4 prevVert = srcVertices[srcCount - 1];
    SDVec2 prevUv   = srcUvs[srcCount - 1];
    float prevComponent = prevVert.v[index] * sign;
    bool prevInside = prevComponent <= prevVert.w;
    for(int i = 0; i < srcCount; ++i) {
        Vec4 currentVert = srcVertices[i];
        SDVec2 currentUv = srcUvs[i];
        float currentComponent = currentVert[index] * sign;
        bool currentInside = currentComponent <= currentVert.w;
        if(currentInside ^ prevInside) {
            float t = (prevVert.w - prevComponent) / ((prevVert.w - prevComponent) - (currentVert.w - currentComponent));
            Vec4 newVertex = {
                (1.0f - t) * prevVert.x + t * currentVert.x,
                (1.0f - t) * prevVert.y + t * currentVert.y, 
                (1.0f - t) * prevVert.z + t * currentVert.z, 
                (1.0f - t) * prevVert.w + t * currentVert.w  
            };
            dstVertices[*dstCount] = newVertex;
            SDVec2 newUv = {
                (1.0f - t) * prevUv.x + t * currentUv.x,
                (1.0f - t) * prevUv.y + t * currentUv.y                    
            };
            dstUvs[*dstCount] = newUv;
            *dstCount = *dstCount + 1;
        }
        if(currentInside) {
            dstVertices[*dstCount] = currentVert;
            dstUvs[*dstCount] = currentUv;
            *dstCount = *dstCount + 1;
        }
        prevVert = currentVert;
        prevUv = currentUv;
        prevComponent = currentComponent;
        prevInside = currentInside;
    }
}

static void DrawTriangle(SDVertex *vertices, SDMat4& view_world, SDMat4& proj, float r_, float g_, float b_, float *depthBuffer) {
    uint32_t color = RgbToUint32(r_, g_, b_, 1);

    Vec4 transformVertex[3];
    for(int i = 0; i < 3; i++)
    {
        transformVertex[i] = view_world * Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
    }
    
    SDVec3 a = sd_vec4_to_vec3(transformVertex[0]);
    SDVec3 b = sd_vec4_to_vec3(transformVertex[1]);
    SDVec3 c = sd_vec4_to_vec3(transformVertex[2]);
    SDVec3 ab = b - a;
    SDVec3 ac = c - a;
    SDVec3 normal = sd_vec3_normalized(sd_vec3_cross(ab, ac));
    SDVec3 camera_origin = sd_mat4_get_col_as_vec3(view, 3) * -1.0f;
    SDVec3 camera_ray = camera_origin - a;
    if(sd_vec3_dot(normal, camera_ray) < 0) return;
    
    for(int i = 0; i < 3; i++)
    {
        transformVertex[i] = proj * Vec4(transformVertex[i].x, transformVertex[i].y, transformVertex[i].z, 1.0f);
    }

    int32_t clippedVertexACount = 3;
    Vec4 clippedVertexA[16] = { transformVertex[0], transformVertex[1], transformVertex[2] };
    SDVec2 clippedUvsA[16] = { vertices[0].uvs, vertices[1].uvs, vertices[2].uvs };
    int32_t clippedVertexBCount = 0;
    Vec4 clippedVertexB[16];
    SDVec2 clippedUvsB[16];
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        0, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        0,  1.0f);
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        1, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        1,  1.0f);
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        2, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        2,  1.0f);

    for(int i = 0; i < clippedVertexACount - 2; i++)
    {
        Vec4 finalA = clippedVertexA[0];
        Vec4 finalB = clippedVertexA[1 + i];
        Vec4 finalC = clippedVertexA[2 + i];

        finalA.x /= finalA.w; finalA.y /= finalA.w;
        finalB.x /= finalB.w; finalB.y /= finalB.w;
        finalC.x /= finalC.w; finalC.y /= finalC.w;

        finalA *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalA += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};
        finalB *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalB += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};
        finalC *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalC += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};

        SDVec2 min{ FLT_MAX,  FLT_MAX};
        SDVec2 max{-FLT_MAX, -FLT_MAX};
        min.x = SD_MIN(min.x, finalA.x); min.y = SD_MIN(min.y, finalA.y);
        max.x = SD_MAX(max.x, finalA.x); max.y = SD_MAX(max.y, finalA.y);
        min.x = SD_MIN(min.x, finalB.x); min.y = SD_MIN(min.y, finalB.y);
        max.x = SD_MAX(max.x, finalB.x); max.y = SD_MAX(max.y, finalB.y);
        min.x = SD_MIN(min.x, finalC.x); min.y = SD_MIN(min.y, finalC.y);
        max.x = SD_MAX(max.x, finalC.x); max.y = SD_MAX(max.y, finalC.y);

        max.x = SD_MIN(max.x, sd_window_width() - 1);
        max.y = SD_MIN(max.y, sd_window_height() - 1);

        for(int y = (int)min.y; y <= (int)max.y; y++)
        {
            for(int x = (int)min.x; x <= (int)max.x; x++)
            {
                uint32_t *backBuffer = sd_back_buffer();
                Vec4 p{ (float)x, (float)y, 0.0f, 0.0f };

                Vec4 a = finalA - p;
                Vec4 b = finalB - p;
                Vec4 c = finalC - p;
                // ab bc ca
                float ab = (a.x * b.y) - (a.y * b.x);
                float bc = (b.x * c.y) - (b.y * c.x);
                float ca = (c.x * a.y) - (c.y * a.x);             
                if(ab >= 0 && bc >= 0 && ca >= 0)
                {
                    SDVec3 coords = Barycentric(SDVec3(finalA.x, finalA.y, 0.0f), SDVec3(finalB.x, finalB.y, 0.0f), SDVec3(finalC.x, finalC.y, 0.0f), SDVec3((float)x, (float)y, 0.0f));
                    float invZA = 1.0f / finalA.w;
                    float invZB = 1.0f / finalB.w;
                    float invZC = 1.0f / finalC.w;
                    float currentInvZ = coords.x * invZA + coords.y * invZB + coords.z *invZC;
                    float lastInvZ = depthBuffer[y * sd_window_width() + x];
                    if(currentInvZ >= lastInvZ)
                    {
                        backBuffer[y * sd_window_width() + x] = color;
                        depthBuffer[y * sd_window_width() + x] = currentInvZ;
                    }
                }
            }
        }
    }
}


static void DrawTriangle(SDVertex *vertices, SDMat4& view_world, SDMat4& proj, SDTexture *texture, float *depthBuffer) {
    Vec4 transformVertex[3];
    for(int i = 0; i < 3; i++)
    {
        transformVertex[i] = view_world * Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
    }

    SDVec3 a = sd_vec4_to_vec3(transformVertex[0]);
    SDVec3 b = sd_vec4_to_vec3(transformVertex[1]);
    SDVec3 c = sd_vec4_to_vec3(transformVertex[2]);
    SDVec3 ab = b - a;
    SDVec3 ac = c - a;
    SDVec3 normal = sd_vec3_normalized(sd_vec3_cross(ab, ac));
    SDVec3 camera_origin = sd_mat4_get_col_as_vec3(view, 3) * -1.0f;
    SDVec3 camera_ray = camera_origin - a;
    if(sd_vec3_dot(normal, camera_ray) < 0) return;

    for(int i = 0; i < 3; i++)
    {
        transformVertex[i] = proj * Vec4(transformVertex[i].x, transformVertex[i].y, transformVertex[i].z, 1.0f);
    }

    int32_t clippedVertexACount = 3;
    Vec4 clippedVertexA[16] = { transformVertex[0], transformVertex[1], transformVertex[2] };
    SDVec2 clippedUvsA[16] = { vertices[0].uvs, vertices[1].uvs, vertices[2].uvs };
    int32_t clippedVertexBCount = 0;
    Vec4 clippedVertexB[16];
    SDVec2 clippedUvsB[16];
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        0, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        0,  1.0f);
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        1, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        1,  1.0f);
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        2, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        2,  1.0f);

    for(int i = 0; i < clippedVertexACount - 2; i++)
    {
        Vec4 finalA = clippedVertexA[0];
        Vec4 finalB = clippedVertexA[1 + i];
        Vec4 finalC = clippedVertexA[2 + i];
        SDVec2 finalUvA = clippedUvsA[0];
        SDVec2 finalUvB = clippedUvsA[1 + i];
        SDVec2 finalUvC = clippedUvsA[2 + i];

        finalA.x /= finalA.w; finalA.y /= finalA.w;
        finalB.x /= finalB.w; finalB.y /= finalB.w;
        finalC.x /= finalC.w; finalC.y /= finalC.w;

        finalA *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalA += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};
        finalB *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalB += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};
        finalC *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalC += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};

        SDVec2 min{ FLT_MAX,  FLT_MAX};
        SDVec2 max{-FLT_MAX, -FLT_MAX};
        min.x = SD_MIN(min.x, finalA.x); min.y = SD_MIN(min.y, finalA.y);
        max.x = SD_MAX(max.x, finalA.x); max.y = SD_MAX(max.y, finalA.y);
        min.x = SD_MIN(min.x, finalB.x); min.y = SD_MIN(min.y, finalB.y);
        max.x = SD_MAX(max.x, finalB.x); max.y = SD_MAX(max.y, finalB.y);
        min.x = SD_MIN(min.x, finalC.x); min.y = SD_MIN(min.y, finalC.y);
        max.x = SD_MAX(max.x, finalC.x); max.y = SD_MAX(max.y, finalC.y);

        max.x = SD_MIN(max.x, sd_window_width() - 1);
        max.y = SD_MIN(max.y, sd_window_height() - 1);

        for(int y = (int)min.y; y <= (int)max.y; y++)
        {
            for(int x = (int)min.x; x <= (int)max.x; x++)
            {
                uint32_t *backBuffer = sd_back_buffer();
                Vec4 p{ (float)x, (float)y, 0.0f, 0.0f };

                Vec4 a = finalA - p;
                Vec4 b = finalB - p;
                Vec4 c = finalC - p;
                // ab bc ca
                float ab = (a.x * b.y) - (a.y * b.x);
                float bc = (b.x * c.y) - (b.y * c.x);
                float ca = (c.x * a.y) - (c.y * a.x);             
                if(ab >= 0 && bc >= 0 && ca >= 0)
                {
                    SDVec3 coords = Barycentric(SDVec3(finalA.x, finalA.y, 0.0f), SDVec3(finalB.x, finalB.y, 0.0f), SDVec3(finalC.x, finalC.y, 0.0f), SDVec3((float)x, (float)y, 0.0f));
                    float invZA = 1.0f / finalA.w;
                    float invZB = 1.0f / finalB.w;
                    float invZC = 1.0f / finalC.w;
                    float currentInvZ = coords.x * invZA + coords.y * invZB + coords.z *invZC;
                    float lastInvZ = depthBuffer[y * sd_window_width() + x];
                    if(currentInvZ >= lastInvZ)
                    {
                        float invU = (coords.x * (finalUvA.x / finalA.w) + coords.y * (finalUvB.x / finalB.w) + coords.z * (finalUvC.x / finalC.w));
                        float invV = (coords.x * (finalUvA.y / finalA.w) + coords.y * (finalUvB.y / finalB.w) + coords.z * (finalUvC.y / finalC.w));

                        uint32_t u = (uint32_t)((invU / currentInvZ) * (float)texture->w);
                        uint32_t v = (uint32_t)((invV / currentInvZ) * (float)texture->h);

                        backBuffer[y * sd_window_width() + x] = texture->data[v * texture->w + u];
                        depthBuffer[y * sd_window_width() + x] = currentInvZ;
                    }
                }
            }
        }
    }


}

static void DrawTriangleAnim(SDSkeleton *skeleton, SDVertex *vertices, SDMat4& view_world, SDMat4& proj, SDTexture *texture, float *depthBuffer) {
    Vec4 transformVertex[3];
    for(int i = 0; i < 3; i++)
    {
        Vec4 total_position = Vec4();
        for(i32 j = 0; j < 4; j++) {
            if(vertices[i].bone_id[j] == -1) continue;
            if(vertices[i].bone_id[j] >= 100) {
                total_position = Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
                break;
            }
            Vec4 local_position = skeleton->final_bone_matrices[vertices[i].bone_id[j]] * Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
            total_position += local_position * vertices[i].weights[j];
        }

        transformVertex[i] = view_world * total_position;
    }

    // TODO: Fix back face culling (view dir is probably wrong) 
    
    SDVec3 a = sd_vec4_to_vec3(transformVertex[0]);
    SDVec3 b = sd_vec4_to_vec3(transformVertex[1]);
    SDVec3 c = sd_vec4_to_vec3(transformVertex[2]);
    SDVec3 ab = b - a;
    SDVec3 ac = c - a;
    SDVec3 normal = sd_vec3_normalized(sd_vec3_cross(ab, ac));
    SDVec3 camera_origin = sd_mat4_get_col_as_vec3(view, 3) * -1.0f;
    SDVec3 camera_ray = camera_origin - a;
    if(sd_vec3_dot(normal, camera_ray) < 0) return;
    

    for(int i = 0; i < 3; i++)
    {
        transformVertex[i] = proj * Vec4(transformVertex[i].x, transformVertex[i].y, transformVertex[i].z, 1.0f);
    }

    int32_t clippedVertexACount = 3;
    Vec4 clippedVertexA[16] = { transformVertex[0], transformVertex[1], transformVertex[2] };
    SDVec2 clippedUvsA[16] = { vertices[0].uvs, vertices[1].uvs, vertices[2].uvs };
    int32_t clippedVertexBCount = 0;
    Vec4 clippedVertexB[16];
    SDVec2 clippedUvsB[16];
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        0, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        0,  1.0f);
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        1, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        1,  1.0f);
    HomogenousClipping(clippedVertexA, clippedUvsA, clippedVertexACount,
                        clippedVertexB, clippedUvsB, &clippedVertexBCount,
                        2, -1.0f);
    HomogenousClipping(clippedVertexB, clippedUvsB, clippedVertexBCount,
                        clippedVertexA, clippedUvsA, &clippedVertexACount,
                        2,  1.0f);

    for(int i = 0; i < clippedVertexACount - 2; i++)
    {
        Vec4 finalA = clippedVertexA[0];
        Vec4 finalB = clippedVertexA[1 + i];
        Vec4 finalC = clippedVertexA[2 + i];
        SDVec2 finalUvA = clippedUvsA[0];
        SDVec2 finalUvB = clippedUvsA[1 + i];
        SDVec2 finalUvC = clippedUvsA[2 + i];

        finalA.x /= finalA.w; finalA.y /= finalA.w;
        finalB.x /= finalB.w; finalB.y /= finalB.w;
        finalC.x /= finalC.w; finalC.y /= finalC.w;

        finalA *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalA += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};
        finalB *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalB += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};
        finalC *= Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 1.0f, 1.0f};
        finalC += Vec4{(float)sd_window_width()*0.5f, (float)sd_window_height()*0.5f, 0.0f, 0.0f};

        SDVec2 min{ FLT_MAX,  FLT_MAX};
        SDVec2 max{-FLT_MAX, -FLT_MAX};
        min.x = SD_MIN(min.x, finalA.x); min.y = SD_MIN(min.y, finalA.y);
        max.x = SD_MAX(max.x, finalA.x); max.y = SD_MAX(max.y, finalA.y);
        min.x = SD_MIN(min.x, finalB.x); min.y = SD_MIN(min.y, finalB.y);
        max.x = SD_MAX(max.x, finalB.x); max.y = SD_MAX(max.y, finalB.y);
        min.x = SD_MIN(min.x, finalC.x); min.y = SD_MIN(min.y, finalC.y);
        max.x = SD_MAX(max.x, finalC.x); max.y = SD_MAX(max.y, finalC.y);

        max.x = SD_MIN(max.x, sd_window_width() - 1);
        max.y = SD_MIN(max.y, sd_window_height() - 1);

        for(int y = (int)min.y; y <= (int)max.y; y++)
        {
            for(int x = (int)min.x; x <= (int)max.x; x++)
            {
                uint32_t *backBuffer = sd_back_buffer();
                Vec4 p{ (float)x, (float)y, 0.0f, 0.0f };

                Vec4 a = finalA - p;
                Vec4 b = finalB - p;
                Vec4 c = finalC - p;
                // ab bc ca
                float ab = (a.x * b.y) - (a.y * b.x);
                float bc = (b.x * c.y) - (b.y * c.x);
                float ca = (c.x * a.y) - (c.y * a.x);             
                if(ab >= 0 && bc >= 0 && ca >= 0)
                {
                    SDVec3 coords = Barycentric(SDVec3(finalA.x, finalA.y, 0.0f), SDVec3(finalB.x, finalB.y, 0.0f), SDVec3(finalC.x, finalC.y, 0.0f), SDVec3((float)x, (float)y, 0.0f));
                    float invZA = 1.0f / finalA.w;
                    float invZB = 1.0f / finalB.w;
                    float invZC = 1.0f / finalC.w;
                    float currentInvZ = coords.x * invZA + coords.y * invZB + coords.z *invZC;
                    float lastInvZ = depthBuffer[y * sd_window_width() + x];
                    if(currentInvZ >= lastInvZ)
                    {
                        float invU = (coords.x * (finalUvA.x / finalA.w) + coords.y * (finalUvB.x / finalB.w) + coords.z * (finalUvC.x / finalC.w));
                        float invV = (coords.x * (finalUvA.y / finalA.w) + coords.y * (finalUvB.y / finalB.w) + coords.z * (finalUvC.y / finalC.w));

                        uint32_t u = (uint32_t)((invU / currentInvZ) * (float)texture->w);
                        uint32_t v = (uint32_t)((invV / currentInvZ) * (float)texture->h);

                        backBuffer[y * sd_window_width() + x] = texture->data[v * texture->w + u];
                        depthBuffer[y * sd_window_width() + x] = currentInvZ;
                    }
                }
            }
        }
    }


}