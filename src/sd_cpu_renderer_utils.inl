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

#define M(a, i) ((f32 *)&(a))[i]
#define Mi(a, i) ((i32 *)&(a))[i]
#define Mu(a, i) ((u32 *)&(a))[i]

void proj_and_razterization(SDVertex *vertices, Vec4 *transformVertex, SDMat4 proj, SDTexture *texture) {

    //===========================================================================================
    // TODO: Fix back face culling (view dir is probably wrong) 
    //===========================================================================================
    SDVec3 a = sd_vec4_to_vec3(transformVertex[0]);
    SDVec3 b = sd_vec4_to_vec3(transformVertex[1]);
    SDVec3 c = sd_vec4_to_vec3(transformVertex[2]);
    SDVec3 ab = b - a;
    SDVec3 ac = c - a;
    SDVec3 normal = sd_vec3_normalized(sd_vec3_cross(ab, ac));
    SDVec3 camera_origin = sd_mat4_get_col_as_vec3(view, 3) * -1.0f; // global variable view ...
    SDVec3 camera_ray = camera_origin - a;
    if(sd_vec3_dot(normal, camera_ray) < 0) return;
    //===========================================================================================

    u32 *backBuffer = sd_back_buffer();
    f32 *depthBuffer = sd_depth_buffer();    

    for(int i = 0; i < 3; i++) {
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

    __m128 zero = _mm_set1_ps(0.0f);
    __m128 one  = _mm_set1_ps(1.0f);
    __m128 texture_w = _mm_set1_ps((f32)texture->w);
    __m128 texture_h = _mm_set1_ps((f32)texture->h);

    for(int i = 0; i < clippedVertexACount - 2; i++) {
        Vec4 finalA = clippedVertexA[0];
        Vec4 finalB = clippedVertexA[1 + i];
        Vec4 finalC = clippedVertexA[2 + i];
        SDVec2 finalUvA = clippedUvsA[0];
        SDVec2 finalUvB = clippedUvsA[1 + i];
        SDVec2 finalUvC = clippedUvsA[2 + i];

        finalA.x /= finalA.w; finalA.y /= finalA.w;
        finalB.x /= finalB.w; finalB.y /= finalB.w;
        finalC.x /= finalC.w; finalC.y /= finalC.w;

        f32 h_window_width = (f32)sd_window_width() * 0.5f;
        f32 h_window_height = (f32)sd_window_height() * 0.5f;
        finalA *= Vec4{h_window_width, h_window_height, 1.0f, 1.0f};
        finalA += Vec4{h_window_width, h_window_height, 0.0f, 0.0f};
        finalB *= Vec4{h_window_width, h_window_height, 1.0f, 1.0f};
        finalB += Vec4{h_window_width, h_window_height, 0.0f, 0.0f};
        finalC *= Vec4{h_window_width, h_window_height, 1.0f, 1.0f};
        finalC += Vec4{h_window_width, h_window_height, 0.0f, 0.0f};

        i32 min_x, min_y, max_x, max_y;
        min_x = (i32)SD_MIN(finalA.x, SD_MIN(finalB.x, finalC.x));
        min_y = (i32)SD_MIN(finalA.y, SD_MIN(finalB.y, finalC.y));
        max_x = (i32)SD_MAX(finalA.x, SD_MAX(finalB.x, finalC.x));
        max_y = (i32)SD_MAX(finalA.y, SD_MAX(finalB.y, finalC.y));

        max_x = SD_MIN(max_x, sd_window_width() - 1);
        max_y = SD_MIN(max_y, sd_window_height() - 1);

        // TODO: clip to the tile ...

        if((min_x >= max_x) || (min_y >= max_y)) continue;

        __m128i start_clip_mask = _mm_set1_epi8(-1);
        __m128i end_clip_mask = _mm_set1_epi8(-1);
        __m128i start_clip_masks[] = {
            _mm_slli_si128(start_clip_mask, 0*4),
            _mm_slli_si128(start_clip_mask, 1*4),
            _mm_slli_si128(start_clip_mask, 2*4),
            _mm_slli_si128(start_clip_mask, 3*4),
        };
        __m128i end_clip_masks[] = {
            _mm_srli_si128(end_clip_mask, 3*4),
            _mm_srli_si128(end_clip_mask, 2*4),
            _mm_srli_si128(end_clip_mask, 1*4),
            _mm_srli_si128(end_clip_mask, 0*4),
        };

        if(min_x & 3) {
            start_clip_mask = start_clip_masks[min_x & 3];
            min_x = min_x & ~3;
        }
        if(max_x & 3) {
            end_clip_mask = end_clip_masks[max_x & 3];
            max_x = (max_x & ~3) + 4;
        }

        // Vertices
        __m128 vert_a_x = _mm_set1_ps(finalA.x);
        __m128 vert_a_y = _mm_set1_ps(finalA.y);
        __m128 vert_a_inv_z = _mm_set1_ps(1.0f/finalA.w);
        __m128 vert_b_x = _mm_set1_ps(finalB.x);
        __m128 vert_b_y = _mm_set1_ps(finalB.y);
        __m128 vert_b_inv_z = _mm_set1_ps(1.0f/finalB.w);
        __m128 vert_c_x = _mm_set1_ps(finalC.x);
        __m128 vert_c_y = _mm_set1_ps(finalC.y);
        __m128 vert_c_inv_z = _mm_set1_ps(1.0f/finalC.w);
        // Uvs
        __m128 uv_a_x = _mm_set1_ps(finalUvA.x);
        __m128 uv_a_y = _mm_set1_ps(finalUvA.y);
        __m128 uv_b_x = _mm_set1_ps(finalUvB.x);
        __m128 uv_b_y = _mm_set1_ps(finalUvB.y);
        __m128 uv_c_x = _mm_set1_ps(finalUvC.x);
        __m128 uv_c_y = _mm_set1_ps(finalUvC.y);
        // Barycentric
        __m128 v0x = _mm_sub_ps(vert_b_x, vert_a_x);
        __m128 v0y = _mm_sub_ps(vert_b_y, vert_a_y);
        __m128 v1x = _mm_sub_ps(vert_c_x, vert_a_x);
        __m128 v1y = _mm_sub_ps(vert_c_y, vert_a_y);
        __m128 d00 = _mm_add_ps(_mm_mul_ps(v0x, v0x), _mm_mul_ps(v0y, v0y));
        __m128 d10 = _mm_add_ps(_mm_mul_ps(v1x, v0x), _mm_mul_ps(v1y, v0y));
        __m128 d11 = _mm_add_ps(_mm_mul_ps(v1x, v1x), _mm_mul_ps(v1y, v1y));
        __m128 denom = _mm_sub_ps(_mm_mul_ps(d00, d11), _mm_mul_ps(d10, d10));


        for(i32 y = min_y; y <= max_y; y++) {
            __m128 test_y = _mm_set1_ps(y);
            __m128i clip_mask = start_clip_mask;
            for(i32 x = min_x; x <= max_x; x += 4) {
                u32 *pixel_pt = backBuffer + (y * sd_window_width() + x);
                __m128i original_dest = _mm_load_si128((__m128i *)pixel_pt);

                f32 *depth_pt = depthBuffer + (y * sd_window_width() + x);
                __m128 depth = _mm_load_ps(depth_pt);

                __m128 test_x = _mm_set_ps(x + 3, x + 2, x + 1, x);

                __m128 ax = _mm_sub_ps(vert_a_x, test_x);
                __m128 ay = _mm_sub_ps(vert_a_y, test_y); 
                __m128 bx = _mm_sub_ps(vert_b_x, test_x);
                __m128 by = _mm_sub_ps(vert_b_y, test_y); 
                __m128 cx = _mm_sub_ps(vert_c_x, test_x);
                __m128 cy = _mm_sub_ps(vert_c_y, test_y); 

                __m128 ab = _mm_sub_ps(_mm_mul_ps(ax, by), _mm_mul_ps(ay, bx));
                __m128 bc = _mm_sub_ps(_mm_mul_ps(bx, cy), _mm_mul_ps(by, cx));
                __m128 ca = _mm_sub_ps(_mm_mul_ps(cx, ay), _mm_mul_ps(cy, ax));

                __m128 test_ab = _mm_cmpge_ps(ab, zero);
                __m128 test_bc = _mm_cmpge_ps(bc, zero);
                __m128 test_ca = _mm_cmpge_ps(ca, zero);
                __m128 write_mask = _mm_and_ps(_mm_and_ps(test_ab, test_bc), test_ca);
                if(_mm_movemask_ps(write_mask)) {
                    __m128i write_maski = _mm_castps_si128(write_mask);

                    __m128 v2x = _mm_sub_ps(test_x, vert_a_x);
                    __m128 v2y = _mm_sub_ps(test_y, vert_a_y);
                    __m128 d20 = _mm_add_ps(_mm_mul_ps(v2x, v0x), _mm_mul_ps(v2y, v0y));
                    __m128 d21 = _mm_add_ps(_mm_mul_ps(v2x, v1x), _mm_mul_ps(v2y, v1y));
                    __m128 gamma = _mm_div_ps(_mm_sub_ps(_mm_mul_ps(d20, d11), _mm_mul_ps(d10, d21)), denom);
                    __m128 beta =  _mm_div_ps(_mm_sub_ps(_mm_mul_ps(d00, d21), _mm_mul_ps(d20, d10)), denom);
                    __m128 alpha = _mm_sub_ps(one, gamma);
                    alpha = _mm_sub_ps(alpha, beta);

                    __m128 inv_z = _mm_add_ps(_mm_add_ps(_mm_mul_ps(vert_a_inv_z, alpha), _mm_mul_ps(vert_b_inv_z, gamma)), _mm_mul_ps(vert_c_inv_z, beta));
                    __m128 depth_test_mask = _mm_cmpge_ps(inv_z, depth);

                    if(_mm_movemask_ps(depth_test_mask)) {
                        __m128i depth_test_maski = _mm_castps_si128(depth_test_mask);

                        // Update the writeMask with the new information
                        write_maski = _mm_and_si128(write_maski, depth_test_maski);
                        write_mask = _mm_and_ps(write_mask, depth_test_mask);
                        write_maski = _mm_and_si128(write_maski, clip_mask);
                        write_mask = _mm_and_ps(write_mask, _mm_castsi128_ps(clip_mask));

                        __m128 inv_u_a = _mm_mul_ps(alpha, _mm_mul_ps(uv_a_x, vert_a_inv_z));
                        __m128 inv_u_b = _mm_mul_ps(gamma, _mm_mul_ps(uv_b_x, vert_b_inv_z));
                        __m128 inv_u_c = _mm_mul_ps(beta,  _mm_mul_ps(uv_c_x, vert_c_inv_z));
                        __m128 int_u = _mm_div_ps(_mm_add_ps(_mm_add_ps(inv_u_a, inv_u_b), inv_u_c), inv_z);

                        __m128 inv_v_a = _mm_mul_ps(alpha, _mm_mul_ps(uv_a_y, vert_a_inv_z));
                        __m128 inv_v_b = _mm_mul_ps(gamma, _mm_mul_ps(uv_b_y, vert_b_inv_z));
                        __m128 inv_v_c = _mm_mul_ps(beta,  _mm_mul_ps(uv_c_y, vert_c_inv_z));
                        __m128 int_v = _mm_div_ps(_mm_add_ps(_mm_add_ps(inv_v_a, inv_v_b), inv_v_c), inv_z);

                        __m128i u = _mm_cvtps_epi32(_mm_mul_ps(int_u, texture_w));
                        __m128i v = _mm_cvtps_epi32(_mm_mul_ps(int_v, texture_h));

                        __m128i color;
                        for(i32 j = 0; j < 4; ++j) {
                            i32 textureX = Mi(u, j);
                            i32 textureY = Mi(v, j);
                            if(textureX >= texture->w) {
                                continue;
                            }
                            if(textureY < 0 || textureY >= texture->h) {
                                continue;
                            }
                            Mi(color, j) = texture->data[textureY * texture->w + textureX];
                        }

                        __m128i color_masked_out = _mm_or_si128(_mm_and_si128(write_maski, color), _mm_andnot_si128(write_maski, original_dest));
                        __m128 depth_mask_out = _mm_or_ps(_mm_and_ps(write_mask, inv_z), _mm_andnot_ps(write_mask, depth));
                        _mm_store_si128((__m128i *)pixel_pt, color_masked_out);
                        _mm_store_ps(depth_pt, depth_mask_out);
                    }
                }
                
                if((x + 4) >= max_x) {
                    clip_mask = end_clip_mask;
                } else {
                    clip_mask = _mm_set1_epi8(-1);
                }
            }
        }
    }

}

static void DrawTriangle(SDVertex *vertices, SDMat4& view_world, SDMat4& proj, SDTexture *texture) {
    Vec4 transformVertex[3];
    for(int i = 0; i < 3; i++)
    {
        transformVertex[i] = view_world * Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
    }
    proj_and_razterization(vertices, transformVertex, proj, texture);
}

static void DrawTriangleAnim(SDMat4 *pallete, SDVertex *vertices, SDMat4& view_world, SDMat4& proj, SDTexture *texture) {
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
            Vec4 local_position = pallete[vertices[i].bone_id[j]] * Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
            total_position += local_position * vertices[i].weights[j];
        }
        transformVertex[i] = view_world * total_position;
    }
    proj_and_razterization(vertices, transformVertex, proj, texture);
}


