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

static void HomogenousClipping(Vec4 *srcVertices, int32_t srcCount, 
                                Vec4 *dstVertices, int32_t *dstCount,
                                int32_t index, float sign) {
    *dstCount = 0;

    Vec4 prevVert = srcVertices[srcCount - 1];
    float prevComponent = prevVert[index] * sign;
    bool prevInside = prevComponent <= prevVert.w;

    Vec4 currentVert = srcVertices[0];
    float currentComponent = currentVert[index] * sign;
    bool currentInside = currentComponent <= currentVert.w;

    if(srcCount == 0) return;

    if(currentInside ^ prevInside) {
        float t = (prevVert.w - prevComponent) / ((prevVert.w - prevComponent) - (currentVert.w - currentComponent));
        Vec4 newVertex = {
            (1.0f - t) * prevVert.x + t * currentVert.x,
            (1.0f - t) * prevVert.y + t * currentVert.y, 
            (1.0f - t) * prevVert.z + t * currentVert.z, 
            (1.0f - t) * prevVert.w + t * currentVert.w  
        };
        dstVertices[*dstCount] = newVertex;
        *dstCount = *dstCount + 1;
    }

    if(currentInside) {
        dstVertices[*dstCount] = currentVert;
        *dstCount = *dstCount + 1;
    }

    if(prevInside) {
        dstVertices[*dstCount] = prevVert;
        *dstCount = *dstCount + 1;
    }  

}


#define M(a, i) ((f32 *)&(a))[i]
#define Mi(a, i) ((i32 *)&(a))[i]
#define Mu(a, i) ((u32 *)&(a))[i]

static void triangle_proj_and_razterization(SDVertex *vertices, Vec4 *transformVertex, SDMat4 proj, SDTexture *texture) {

    //===========================================================================================
    // back face culling
    //===========================================================================================
    
    SDVec3 a = sd_vec4_to_vec3(transformVertex[0]);
    SDVec3 b = sd_vec4_to_vec3(transformVertex[1]);
    SDVec3 c = sd_vec4_to_vec3(transformVertex[2]);
    SDVec3 ab = b - a;
    SDVec3 ac = c - a;
    SDVec3 normal = sd_vec3_normalized(sd_vec3_cross(ab, ac));
    SDVec3 camera_origin = SDVec3();
    SDVec3 camera_ray = sd_vec3_normalized(camera_origin - a);
    if(sd_vec3_dot(normal, camera_ray) < 0) return;
    
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

    __m256 zero = _mm256_set1_ps(0.0f);
    __m256 one  = _mm256_set1_ps(1.0f);
    __m256 texture_w = _mm256_set1_ps((f32)texture->w);
    __m256 texture_h = _mm256_set1_ps((f32)texture->h);

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

        if((min_x >= max_x) || (min_y >= max_y)) continue;

        if(min_x & 7) {
            min_x = min_x & ~7;
        }
        if(max_x & 7) {
            max_x = (max_x & ~7) + 8;
        }
        
        // Vertices
        __m256 vert_a_x = _mm256_set1_ps(finalA.x);
        __m256 vert_a_y = _mm256_set1_ps(finalA.y);
        __m256 vert_a_inv_z = _mm256_set1_ps(1.0f/finalA.w);
        __m256 vert_b_x = _mm256_set1_ps(finalB.x);
        __m256 vert_b_y = _mm256_set1_ps(finalB.y);
        __m256 vert_b_inv_z = _mm256_set1_ps(1.0f/finalB.w);
        __m256 vert_c_x = _mm256_set1_ps(finalC.x);
        __m256 vert_c_y = _mm256_set1_ps(finalC.y);
        __m256 vert_c_inv_z = _mm256_set1_ps(1.0f/finalC.w);
        // Uvs
        __m256 uv_a_x = _mm256_set1_ps(finalUvA.x);
        __m256 uv_a_y = _mm256_set1_ps(finalUvA.y);
        __m256 uv_b_x = _mm256_set1_ps(finalUvB.x);
        __m256 uv_b_y = _mm256_set1_ps(finalUvB.y);
        __m256 uv_c_x = _mm256_set1_ps(finalUvC.x);
        __m256 uv_c_y = _mm256_set1_ps(finalUvC.y);
        // Barycentric
        __m256 v0x = _mm256_sub_ps(vert_b_x, vert_a_x);
        __m256 v0y = _mm256_sub_ps(vert_b_y, vert_a_y);
        __m256 v1x = _mm256_sub_ps(vert_c_x, vert_a_x);
        __m256 v1y = _mm256_sub_ps(vert_c_y, vert_a_y);
        __m256 d00 = _mm256_add_ps(_mm256_mul_ps(v0x, v0x), _mm256_mul_ps(v0y, v0y));
        __m256 d10 = _mm256_add_ps(_mm256_mul_ps(v1x, v0x), _mm256_mul_ps(v1y, v0y));
        __m256 d11 = _mm256_add_ps(_mm256_mul_ps(v1x, v1x), _mm256_mul_ps(v1y, v1y));
        __m256 denom = _mm256_sub_ps(_mm256_mul_ps(d00, d11), _mm256_mul_ps(d10, d10));


        for(i32 y = min_y; y <= max_y; y++) {
            __m256 test_y = _mm256_set1_ps((f32)y);
            for(i32 x = min_x; x <= max_x; x += 8) {

                u32 *pixel_pt = backBuffer + (y * sd_window_width() + x);
                __m256i original_dest = _mm256_load_si256((__m256i *)pixel_pt);
                f32 *depth_pt = depthBuffer + (y * sd_window_width() + x);
                __m256 depth = _mm256_load_ps(depth_pt);
                __m256 test_x = _mm256_set_ps((f32)x + 7, (f32)x + 6, (f32)x + 5, (f32)x + 4,
                                              (f32)x + 3, (f32)x + 2, (f32)x + 1, (f32)x + 0);

                __m256 ax = _mm256_sub_ps(vert_a_x, test_x);
                __m256 ay = _mm256_sub_ps(vert_a_y, test_y); 
                __m256 bx = _mm256_sub_ps(vert_b_x, test_x);
                __m256 by = _mm256_sub_ps(vert_b_y, test_y); 
                __m256 cx = _mm256_sub_ps(vert_c_x, test_x);
                __m256 cy = _mm256_sub_ps(vert_c_y, test_y); 

                __m256 ab = _mm256_sub_ps(_mm256_mul_ps(ax, by), _mm256_mul_ps(ay, bx));
                __m256 bc = _mm256_sub_ps(_mm256_mul_ps(bx, cy), _mm256_mul_ps(by, cx));
                __m256 ca = _mm256_sub_ps(_mm256_mul_ps(cx, ay), _mm256_mul_ps(cy, ax));

                __m256 test_ab = _mm256_cmp_ps(ab, zero, _CMP_GE_OS);
                __m256 test_bc = _mm256_cmp_ps(bc, zero, _CMP_GE_OS);
                __m256 test_ca = _mm256_cmp_ps(ca, zero, _CMP_GE_OS);
                __m256 write_mask = _mm256_and_ps(_mm256_and_ps(test_ab, test_bc), test_ca);
                if(_mm256_movemask_ps(write_mask)) {
                    __m256i write_maski = _mm256_castps_si256(write_mask);

                    __m256 v2x = _mm256_sub_ps(test_x, vert_a_x);
                    __m256 v2y = _mm256_sub_ps(test_y, vert_a_y);
                    __m256 d20 = _mm256_add_ps(_mm256_mul_ps(v2x, v0x), _mm256_mul_ps(v2y, v0y));
                    __m256 d21 = _mm256_add_ps(_mm256_mul_ps(v2x, v1x), _mm256_mul_ps(v2y, v1y));
                    __m256 gamma = _mm256_div_ps(_mm256_sub_ps(_mm256_mul_ps(d20, d11), _mm256_mul_ps(d10, d21)), denom);
                    __m256 beta =  _mm256_div_ps(_mm256_sub_ps(_mm256_mul_ps(d00, d21), _mm256_mul_ps(d20, d10)), denom);
                    __m256 alpha = _mm256_sub_ps(one, gamma);
                    alpha = _mm256_sub_ps(alpha, beta);

                    __m256 inv_z = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vert_a_inv_z, alpha), _mm256_mul_ps(vert_b_inv_z, gamma)), _mm256_mul_ps(vert_c_inv_z, beta));
                    __m256 depth_test_mask = _mm256_cmp_ps(inv_z, depth, _CMP_GE_OS);

                    if(_mm256_movemask_ps(depth_test_mask)) {
                        __m256i depth_test_maski = _mm256_castps_si256(depth_test_mask);

                        // Update the writeMask with the new information
                        write_maski = _mm256_and_si256(write_maski, depth_test_maski);
                        write_mask = _mm256_and_ps(write_mask, depth_test_mask);

                        __m256 inv_u_a = _mm256_mul_ps(alpha, _mm256_mul_ps(uv_a_x, vert_a_inv_z));
                        __m256 inv_u_b = _mm256_mul_ps(gamma, _mm256_mul_ps(uv_b_x, vert_b_inv_z));
                        __m256 inv_u_c = _mm256_mul_ps(beta,  _mm256_mul_ps(uv_c_x, vert_c_inv_z));
                        __m256 int_u = _mm256_div_ps(_mm256_add_ps(_mm256_add_ps(inv_u_a, inv_u_b), inv_u_c), inv_z);

                        __m256 inv_v_a = _mm256_mul_ps(alpha, _mm256_mul_ps(uv_a_y, vert_a_inv_z));
                        __m256 inv_v_b = _mm256_mul_ps(gamma, _mm256_mul_ps(uv_b_y, vert_b_inv_z));
                        __m256 inv_v_c = _mm256_mul_ps(beta,  _mm256_mul_ps(uv_c_y, vert_c_inv_z));
                        __m256 int_v = _mm256_div_ps(_mm256_add_ps(_mm256_add_ps(inv_v_a, inv_v_b), inv_v_c), inv_z);

                        __m256i u = _mm256_cvtps_epi32(_mm256_mul_ps(int_u, texture_w));
                        __m256i v = _mm256_cvtps_epi32(_mm256_mul_ps(int_v, texture_h));

                        __m256i color;
                        for(i32 j = 0; j < 8; ++j) {
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

                        __m256i color_masked_out = _mm256_or_si256(_mm256_and_si256(write_maski, color), _mm256_andnot_si256(write_maski, original_dest));
                        __m256 depth_mask_out = _mm256_or_ps(_mm256_and_ps(write_mask, inv_z), _mm256_andnot_ps(write_mask, depth));
                        _mm256_store_si256((__m256i *)pixel_pt, color_masked_out);
                        _mm256_store_ps(depth_pt, depth_mask_out);
                    }
                }

            }
        }
    }

}

static void line_clipping_and_razterization(Vec4 *transform_vertex, f32 r_, f32 g_, f32 b_) {
    u32 *backBuffer = sd_back_buffer();
    f32 *depthBuffer = sd_depth_buffer();    
    u32 color = RgbToUint32(r_, g_, b_, 1.0f);

    int32_t clippedVertexACount = 2;
    Vec4 clippedVertexA[2] = { transform_vertex[0], transform_vertex[1] };
    int32_t clippedVertexBCount = 0;
    Vec4 clippedVertexB[2];
    HomogenousClipping(clippedVertexA, clippedVertexACount,
                        clippedVertexB, &clippedVertexBCount,
                        0, -1.0f);
    HomogenousClipping(clippedVertexB, clippedVertexBCount,
                        clippedVertexA, &clippedVertexACount,
                        0,  1.0f);
    HomogenousClipping(clippedVertexA, clippedVertexACount,
                        clippedVertexB, &clippedVertexBCount,
                        1, -1.0f);
    HomogenousClipping(clippedVertexB, clippedVertexBCount,
                        clippedVertexA, &clippedVertexACount,
                        1,  1.0f);
    HomogenousClipping(clippedVertexA, clippedVertexACount,
                        clippedVertexB, &clippedVertexBCount,
                        2, -1.0f);
    HomogenousClipping(clippedVertexB, clippedVertexBCount,
                        clippedVertexA, &clippedVertexACount,
                        2,  1.0f);

    if(clippedVertexACount < 2) return;

    Vec4 a = clippedVertexA[0];
    Vec4 b = clippedVertexA[1];

    // perpective divide
    a.x /= a.w; a.y /= a.w;
    b.x /= b.w; b.y /= b.w;
    f32 h_window_width = (f32)sd_window_width() * 0.5f;
    f32 h_window_height = (f32)sd_window_height() * 0.5f;
    a *= Vec4{h_window_width, h_window_height, 1.0f, 1.0f};
    a += Vec4{h_window_width, h_window_height, 0.0f, 0.0f};
    b *= Vec4{h_window_width, h_window_height, 1.0f, 1.0f};
    b += Vec4{h_window_width, h_window_height, 0.0f, 0.0f};

    a.x = SD_MIN(a.x, sd_window_width() - 1);
    a.y = SD_MIN(a.y, sd_window_height() - 1);
    b.x = SD_MIN(b.x, sd_window_width() - 1);
    b.y = SD_MIN(b.y, sd_window_height() - 1);

    i32 x_delta = (i32)(b.x - a.x);
    i32 y_delta = (i32)(b.y - a.y);
    i32 side_length = std::abs(x_delta) >= std::abs(y_delta) ? std::abs(x_delta) : std::abs(y_delta);
    f32 x_inc = (f32)x_delta / (f32)side_length;
    f32 y_inc = (f32)y_delta / (f32)side_length;
    f32 x = a.x;
    f32 y = a.y;
    for(i32 i = 0; i <= side_length; ++i) {
        SDVec2 p = SDVec2(x, y);
        SDVec2 start = SDVec2(a.x, a.y);
        SDVec2 delta = SDVec2(b.x - a.x, b.y - a.y);

        f32 t = sd_vec2_len(start - p) / sd_vec2_len(delta);
        f32 int_inv_z = ((1.0f/a.w) + ((1.0f/b.w) - (1.0f/a.w)) * t); 

        i32 window_width = sd_window_width();
        if(int_inv_z >= depthBuffer[(i32)y * window_width + (i32)x]) {
            depthBuffer[(i32)y * window_width + (i32)x] = int_inv_z;
            backBuffer[(i32)y * window_width + (i32)x] = color;
        }

        x += x_inc;
        y += y_inc;
    }
}


static void DrawTriangle(SDVertex *vertices, SDMat4& view_world, SDMat4& proj, SDTexture *texture) {
    Vec4 transformVertex[3];
    for(int i = 0; i < 3; i++)
    {
        transformVertex[i] = view_world * Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
    }
    triangle_proj_and_razterization(vertices, transformVertex, proj, texture);
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
    triangle_proj_and_razterization(vertices, transformVertex, proj, texture);
}

static void draw_line(SDVertex *vertices, SDMat4& view, SDMat4& proj, f32 r, f32 g, f32 b) {
    Vec4 transform_vertex[2];
    for(i32 i = 0; i < 2; i++) {
        transform_vertex[i] = proj * view * Vec4(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f);
    }
    line_clipping_and_razterization(transform_vertex, r, g, b);
}


