#include <sd_math.h>

f32 sd_lerp(f32 a, f32 b, f32 t) {
    return (1.0f - t) * a + b * t;
}

f32 sd_inv_lerp(f32 a, f32 b, f32 v) {
    return (v - a) / (b - a);
}

f32 sd_remap(f32 i_min, f32 i_max, f32 o_min, f32 o_max, f32 v) {
    f32 t = sd_inv_lerp(i_min, i_max, v);
    return sd_lerp(o_min, o_max, t);
}

//==================================================================
// Vector 2
//==================================================================
float SDVec2::operator[](int index)
{
    return v[index];
}

SDVec2 SDVec2::operator+(const SDVec2& rhs)
{
    SDVec2 result { x + rhs.x, y + rhs.y };
    return result;
}

SDVec2 SDVec2::operator-(const SDVec2& rhs)
{
    SDVec2 result { x - rhs.x, y - rhs.y };
    return result;
}

SDVec2 SDVec2::operator*(const SDVec2& rhs)
{
    SDVec2 result { x * rhs.x, y * rhs.y };
    return result;
}

SDVec2 SDVec2::operator/(const SDVec2& rhs)
{
    SDVec2 result { x / rhs.x, y / rhs.y };
    return result;
}

void SDVec2::operator+=(const SDVec2& rhs)
{
    x += rhs.x;
    y += rhs.y;
}

void SDVec2::operator-=(const SDVec2& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
}

void SDVec2::operator*=(const SDVec2& rhs)
{
    x *= rhs.x;
    y *= rhs.y;
}

void SDVec2::operator/=(const SDVec2& rhs)
{
    x /= rhs.x;
    y /= rhs.y;
}

float sd_vec2_dot(const SDVec2& lhs, const SDVec2& rhs)
{
    return (lhs.x * rhs.x) + (lhs.y * rhs.y);
}

float sd_vec2_len_sq(const SDVec2& v)
{
    return sd_vec2_dot(v, v);
}

float sd_vec2_len(const SDVec2& v)
{
    return std::sqrt(sd_vec2_len_sq(v));
}

SDVec2 sd_vec2_normalized(const SDVec2& v)
{
    float lenSq = sd_vec2_len_sq(v);
    if(lenSq > 0)
    {
        float invLen = 1.0f / std::sqrt(lenSq);
        SDVec2 result {
            v.x * invLen,
            v.y * invLen
        };
        return result;
    }
    return v;
}


void sd_vec2_normalize(SDVec2& v)
{
    float lenSq = sd_vec2_len_sq(v);
    if(lenSq > 0)
    {
        float invLen = 1.0f / std::sqrt(lenSq);
        v.x *= invLen;
        v.y *= invLen;
    }
}


//==================================================================
// Vector 3
//==================================================================
float SDVec3::operator[](int index)
{
    return v[index];
}

SDVec3 SDVec3::operator+(const SDVec3& rhs)
{
    SDVec3 result { x + rhs.x, y + rhs.y, z + rhs.z };
    return result;
}

SDVec3 SDVec3::operator-(const SDVec3& rhs)
{
    SDVec3 result { x - rhs.x, y - rhs.y, z - rhs.z };
    return result;
}

SDVec3 SDVec3::operator*(const SDVec3& rhs)
{
    SDVec3 result { x * rhs.x, y * rhs.y, z * rhs.z };
    return result;
}

SDVec3 SDVec3::operator/(const SDVec3& rhs)
{
    SDVec3 result { x / rhs.x, y / rhs.y, z / rhs.z };
    return result;
}


SDVec3 SDVec3::operator+(const SDVec3& rhs) const
{
    SDVec3 result { x + rhs.x, y + rhs.y, z + rhs.z };
    return result;
}

SDVec3 SDVec3::operator-(const SDVec3& rhs) const
{
    SDVec3 result { x - rhs.x, y - rhs.y, z - rhs.z };
    return result;
}

SDVec3 SDVec3::operator*(const SDVec3& rhs) const
{
    SDVec3 result { x * rhs.x, y * rhs.y, z * rhs.z };
    return result;
}

SDVec3 SDVec3::operator/(const SDVec3& rhs) const
{
    SDVec3 result { x / rhs.x, y / rhs.y, z / rhs.z };
    return result;
}





void SDVec3::operator+=(const SDVec3& rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
}

void SDVec3::operator-=(const SDVec3& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
}

void SDVec3::operator*=(const SDVec3& rhs)
{
    x *= rhs.x;
    y *= rhs.y;
    z *= rhs.z;
}

void SDVec3::operator/=(const SDVec3& rhs)
{
    x /= rhs.x;
    y /= rhs.y;
    z /= rhs.z;
}

SDVec3 SDVec3::operator+(float rhs)
{
    SDVec3 result { x + rhs, y + rhs, z + rhs };
    return result;    
}
SDVec3 SDVec3::operator-(float rhs)
{
    SDVec3 result { x - rhs, y - rhs, z - rhs };
    return result;    
}
SDVec3 SDVec3::operator*(float rhs)
{
    SDVec3 result { x * rhs, y * rhs, z * rhs };
    return result;    
}

SDVec3 SDVec3::operator/(float rhs)
{
    SDVec3 result { x / rhs, y / rhs, z / rhs };
    return result;    
}


void SDVec3::operator+=(float rhs)
{
    x += rhs;
    y += rhs;
    z += rhs;
}
void SDVec3::operator-=(float rhs)
{
    x -= rhs;
    y -= rhs;
    z -= rhs;   
}
void SDVec3::operator*=(float rhs)
{
    x *= rhs;
    y *= rhs;
    z *= rhs;   
}

void SDVec3::operator/=(float rhs)
{
    x /= rhs;
    y /= rhs;
    z /= rhs;  
}


float sd_vec3_dot(const SDVec3& lhs, const SDVec3& rhs)
{
    return (lhs.x * rhs.x) + (lhs.y * rhs.y) + (lhs.z * rhs.z);
}

SDVec3 sd_vec3_cross(const SDVec3& lhs, const SDVec3& rhs)
{
    SDVec3 result {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
    return result;
}


float sd_vec3_len_sq(const SDVec3& v)
{
    return sd_vec3_dot(v, v);
}

float sd_vec3_len(const SDVec3& v)
{
    return std::sqrt(sd_vec3_len_sq(v));
}

SDVec3 sd_vec3_normalized(const SDVec3& v)
{
    float lenSq = sd_vec3_len_sq(v);
    if(lenSq > 0)
    {
        float invLen = 1.0f / std::sqrt(lenSq);
        SDVec3 result {
            v.x * invLen,
            v.y * invLen,
            v.z * invLen
        };
        return result;
    }
    return v;
}


void sd_vec3_normalize(SDVec3& v)
{
    float lenSq = sd_vec3_len_sq(v);
    if(lenSq > 0)
    {
        float invLen = 1.0f / std::sqrt(lenSq);
        v.x *= invLen;
        v.y *= invLen;
        v.z *= invLen;
    }
}

SDVec3 sd_vec3_lerp(SDVec3 a, SDVec3 b, f32 t) {
    SDVec3 result = a * (1.0f - t) + b * t;
    return result;
}

f32 sd_vec3_angle(SDVec3 a, SDVec3 b) {
    f32 a_len_sq = sd_vec3_len_sq(a);
    f32 b_len_sq = sd_vec3_len_sq(b);
    if(a_len_sq < SD_VEC_EPSILON || b_len_sq < SD_VEC_EPSILON) {
        return 0.0f;
    }

    f32 dot = sd_vec3_dot(a, b);
    f32 len = std::sqrtf(a_len_sq) * std::sqrtf(b_len_sq);
    f32 result = std::acosf(dot / len);
    return result;
}




//==================================================================
// Vector 4
//==================================================================
float Vec4::operator[](int index)
{
    return v[index];
}

Vec4 Vec4::operator+(const Vec4& rhs)
{
    Vec4 result { x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w };
    return result;
}

Vec4 Vec4::operator-(const Vec4& rhs)
{
    Vec4 result { x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w };
    return result;
}

Vec4 Vec4::operator*(const Vec4& rhs)
{
    Vec4 result { x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w };
    return result;
}

Vec4 Vec4::operator/(const Vec4& rhs)
{
    Vec4 result { x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w };
    return result;
}

void Vec4::operator+=(const Vec4& rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    w += rhs.w;
}

void Vec4::operator-=(const Vec4& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    w -= rhs.w;
}

void Vec4::operator*=(const Vec4& rhs)
{
    x *= rhs.x;
    y *= rhs.y;
    z *= rhs.z;
    w *= rhs.w;
}

void Vec4::operator/=(const Vec4& rhs)
{
    x /= rhs.x;
    y /= rhs.y;
    z /= rhs.z;
    w /= rhs.w;
}

Vec4 Vec4::operator+(float rhs)
{
    Vec4 result { x + rhs, y + rhs, z + rhs, w + rhs };
    return result;    
}
Vec4 Vec4::operator-(float rhs)
{
    Vec4 result { x - rhs, y - rhs, z - rhs, w - rhs };
    return result;    
}
Vec4 Vec4::operator*(float rhs)
{
    Vec4 result { x * rhs, y * rhs, z * rhs, w * rhs };
    return result;    
}

Vec4 Vec4::operator/(float rhs)
{
    Vec4 result { x / rhs, y / rhs, z / rhs, w / rhs };
    return result;    
}

SDVec3 sd_vec4_to_vec3(const Vec4& v)
{
    return SDVec3(v.x, v.y, v.z);
}


//==================================================================
// Matrix 3
//==================================================================

float SDMat3::operator()(int i, int j)
{
    return m[j][i];
}

float SDMat3::operator()(int i, int j) const
{
    return m[j][i];
}


SDVec3& SDMat3::operator[](int col)
{
    return *reinterpret_cast<SDVec3 *>(m[col]);
}

const SDVec3& SDMat3::operator[](int col) const
{
    return *reinterpret_cast<const SDVec3 *>(m[col]);
}

SDMat3 SDMat3::operator*(const SDMat3& rhs)
{
    SDMat3 result = SDMat3((*this)(0,0)*rhs(0,0) + (*this)(0,1)*rhs(1,0) + (*this)(0,2)*rhs(2,0),
                       (*this)(0,0)*rhs(0,1) + (*this)(0,1)*rhs(1,1) + (*this)(0,2)*rhs(2,1),
                       (*this)(0,0)*rhs(0,2) + (*this)(0,1)*rhs(1,2) + (*this)(0,2)*rhs(2,2),
                       (*this)(1,0)*rhs(0,0) + (*this)(1,1)*rhs(1,0) + (*this)(1,2)*rhs(2,0),
                       (*this)(1,0)*rhs(0,1) + (*this)(1,1)*rhs(1,1) + (*this)(1,2)*rhs(2,1),
                       (*this)(1,0)*rhs(0,2) + (*this)(1,1)*rhs(1,2) + (*this)(1,2)*rhs(2,2),
                       (*this)(2,0)*rhs(0,0) + (*this)(2,1)*rhs(1,0) + (*this)(2,2)*rhs(2,0),
                       (*this)(2,0)*rhs(0,1) + (*this)(2,1)*rhs(1,1) + (*this)(2,2)*rhs(2,1),
                       (*this)(2,0)*rhs(0,2) + (*this)(2,1)*rhs(1,2) + (*this)(2,2)*rhs(2,2));
    return result;
}

SDVec3 SDMat3::operator*(const SDVec3& rhs)
{
    SDVec3 result = SDVec3((*this)(0,0)*rhs.x + (*this)(0,1)*rhs.y + (*this)(0,2)*rhs.z,
                       (*this)(1,0)*rhs.x + (*this)(1,1)*rhs.y + (*this)(1,2)*rhs.z,
                       (*this)(2,0)*rhs.x + (*this)(2,1)*rhs.y + (*this)(2,2)*rhs.z);
    return result;
}

SDMat3 sd_mat3_rotation_x(float angle)
{
    SDMat3 result = SDMat3(
        1,                0,                 0,
        0, std::cosf(angle), -std::sinf(angle),
        0, std::sinf(angle),  std::cosf(angle)
    );
    return result;
}

SDMat3 sd_mat3_rotation_y(float angle)
{
    SDMat3 result = SDMat3(
        std::cosf(angle), 0,  std::sinf(angle),
        0,                1,                 0,
       -std::sinf(angle), 0,  std::cosf(angle)
    );
    return result;
}

SDMat3 sd_mat3_rotation_z(float angle)
{
    SDMat3 result = SDMat3(
        std::cosf(angle), -std::sinf(angle), 0,
        std::sinf(angle),  std::cosf(angle), 0,
                       0,                 0, 1
    );
    return result;
}

SDMat3 sd_mat3_scale(float x, float y, float z)
{
    SDMat3 result = SDMat3(
        x, 0, 0,
        0, y, 0,
        0, 0, z
    );
    return result;
}

SDMat3 sd_mat3_inverse(const SDMat3& m) {
    const SDVec3& a = m[0];
    const SDVec3& b = m[1];
    const SDVec3& c = m[2];

    SDVec3 r0 = sd_vec3_cross(b, c);
    SDVec3 r1 = sd_vec3_cross(c, a);
    SDVec3 r2 = sd_vec3_cross(a, b);

    f32 inv_det = 1.0f / sd_vec3_dot(r2, c);

    return SDMat3(r0.x * inv_det, r0.y * inv_det, r0.z * inv_det,
                  r1.x * inv_det, r1.y * inv_det, r1.z * inv_det,
                  r2.x * inv_det, r2.y * inv_det, r2.z * inv_det);
}


SDMat3 sd_mat3_transposed(SDMat3 &m) {
    SDMat3 result;
    result.m[0][0] = m.m[0][0];
    result.m[0][1] = m.m[1][0];
    result.m[0][2] = m.m[2][0];
    result.m[1][0] = m.m[0][1];
    result.m[1][1] = m.m[1][1];
    result.m[1][2] = m.m[2][1];
    result.m[2][0] = m.m[0][2];
    result.m[2][1] = m.m[1][2];
    result.m[2][2] = m.m[2][2];
    return result;
}

SDMat3 sd_mat3_orthonormal_basis(SDVec3 x) {

    SDVec3 y = SDVec3(0, 1, 0);
    if(std::fabsf(x.x) <= std::fabsf(x.y)) {
        y = SDVec3(1, 0, 0);
    }

    sd_vec3_normalize(x);
    SDVec3 z = sd_vec3_cross(x, y);
    sd_vec3_normalize(z);
    y = sd_vec3_cross(z, x);

    // TODO: check if this is ok or should it be transposed
    return SDMat3(x.x, y.x, z.x,
                  x.y, y.y, z.y,
                  x.z, y.z, z.z);

}


//==================================================================
// Matrix 4
//==================================================================
float SDMat4::operator()(int i, int j)
{
    return m[j][i];
}

float SDMat4::operator()(int i, int j) const
{
    return m[j][i];
}

Vec4& SDMat4::operator[](int col)
{
    return *reinterpret_cast<Vec4 *>(m[col]);
}

const Vec4& SDMat4::operator[](int col) const {
    return *reinterpret_cast<const Vec4 *>(m[col]);
}

SDMat4 SDMat4::operator*(const SDMat4& rhs)
{
    SDMat4 result = SDMat4((*this)(0,0)*rhs(0,0) + (*this)(0,1)*rhs(1,0) + (*this)(0,2)*rhs(2,0) + (*this)(0,3)*rhs(3,0),
                       (*this)(0,0)*rhs(0,1) + (*this)(0,1)*rhs(1,1) + (*this)(0,2)*rhs(2,1) + (*this)(0,3)*rhs(3,1),
                       (*this)(0,0)*rhs(0,2) + (*this)(0,1)*rhs(1,2) + (*this)(0,2)*rhs(2,2) + (*this)(0,3)*rhs(3,2),
                       (*this)(0,0)*rhs(0,3) + (*this)(0,1)*rhs(1,3) + (*this)(0,2)*rhs(2,3) + (*this)(0,3)*rhs(3,3),
                       (*this)(1,0)*rhs(0,0) + (*this)(1,1)*rhs(1,0) + (*this)(1,2)*rhs(2,0) + (*this)(1,3)*rhs(3,0),
                       (*this)(1,0)*rhs(0,1) + (*this)(1,1)*rhs(1,1) + (*this)(1,2)*rhs(2,1) + (*this)(1,3)*rhs(3,1),
                       (*this)(1,0)*rhs(0,2) + (*this)(1,1)*rhs(1,2) + (*this)(1,2)*rhs(2,2) + (*this)(1,3)*rhs(3,2),
                       (*this)(1,0)*rhs(0,3) + (*this)(1,1)*rhs(1,3) + (*this)(1,2)*rhs(2,3) + (*this)(1,3)*rhs(3,3),
                       (*this)(2,0)*rhs(0,0) + (*this)(2,1)*rhs(1,0) + (*this)(2,2)*rhs(2,0) + (*this)(2,3)*rhs(3,0),
                       (*this)(2,0)*rhs(0,1) + (*this)(2,1)*rhs(1,1) + (*this)(2,2)*rhs(2,1) + (*this)(2,3)*rhs(3,1),
                       (*this)(2,0)*rhs(0,2) + (*this)(2,1)*rhs(1,2) + (*this)(2,2)*rhs(2,2) + (*this)(2,3)*rhs(3,2),
                       (*this)(2,0)*rhs(0,3) + (*this)(2,1)*rhs(1,3) + (*this)(2,2)*rhs(2,3) + (*this)(2,3)*rhs(3,3),
                       (*this)(3,0)*rhs(0,0) + (*this)(3,1)*rhs(1,0) + (*this)(3,2)*rhs(2,0) + (*this)(3,3)*rhs(3,0),
                       (*this)(3,0)*rhs(0,1) + (*this)(3,1)*rhs(1,1) + (*this)(3,2)*rhs(2,1) + (*this)(3,3)*rhs(3,1),
                       (*this)(3,0)*rhs(0,2) + (*this)(3,1)*rhs(1,2) + (*this)(3,2)*rhs(2,2) + (*this)(3,3)*rhs(3,2),
                       (*this)(3,0)*rhs(0,3) + (*this)(3,1)*rhs(1,3) + (*this)(3,2)*rhs(2,3) + (*this)(3,3)*rhs(3,3));
    return result;
}

Vec4 SDMat4::operator*(const Vec4& rhs)
{
    Vec4 result = Vec4((*this)(0,0)*rhs.x + (*this)(0,1)*rhs.y + (*this)(0,2)*rhs.z + (*this)(0,3)*rhs.w,
                       (*this)(1,0)*rhs.x + (*this)(1,1)*rhs.y + (*this)(1,2)*rhs.z + (*this)(1,3)*rhs.w,
                       (*this)(2,0)*rhs.x + (*this)(2,1)*rhs.y + (*this)(2,2)*rhs.z + (*this)(2,3)*rhs.w,
                       (*this)(3,0)*rhs.x + (*this)(3,1)*rhs.y + (*this)(3,2)*rhs.z + (*this)(3,3)*rhs.w);
    return result;
}


SDVec3 sd_mat4_get_col_as_vec3(SDMat4& m, int col)
{
    Vec4 v = *reinterpret_cast<Vec4 *>(m.m[col]);
    return SDVec3(v.x, v.y, v.z);
}


SDMat4 sd_mat4_translation(float x, float y, float z)
{
    SDMat4 result = SDMat4(
        1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1
    );
    return result;
}

SDMat4 sd_mat4_translation(SDVec3 v)
{
    SDMat4 result = SDMat4(
        1, 0, 0, v.x,
        0, 1, 0, v.y,
        0, 0, 1, v.z,
        0, 0, 0, 1
    );
    return result;
}

SDMat4 sd_mat4_rotation_x(float angle)
{
    SDMat4 result = SDMat4(
        1,                0,                 0, 0,
        0, std::cosf(angle), -std::sinf(angle), 0,
        0, std::sinf(angle),  std::cosf(angle), 0,
        0,                0,                 0, 1
    );
    return result;
}

SDMat4 sd_mat4_rotation_y(float angle)
{
    SDMat4 result = SDMat4(
        std::cosf(angle), 0,  std::sinf(angle), 0,
        0,                1,                 0, 0,
       -std::sinf(angle), 0,  std::cosf(angle), 0,
        0,                0,                 0, 1
    );
    return result;
}

SDMat4 sd_mat4_rotation_z(float angle)
{
    SDMat4 result = SDMat4(
        std::cosf(angle), -std::sinf(angle), 0, 0,
        std::sinf(angle),  std::cosf(angle), 0, 0,
                       0,                 0, 1, 0,
                       0,                 0, 0, 1
    );
    return result;
}

SDMat4 sd_mat4_scale(float x, float y, float z)
{
    SDMat4 result = SDMat4(
        x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1
    );
    return result;
}

SDMat4 sd_mat4_scale(SDVec3 v)
{
    SDMat4 result = SDMat4(
        v.x,   0,   0, 0,
          0, v.y,   0, 0,
          0,   0, v.z, 0,
          0,   0,   0, 1
    );
    return result;
}

SDMat4 sd_mat4_lookat(SDVec3 pos, SDVec3 tar, SDVec3 up)
{
    SDVec3 z = (tar - pos);
    z.x *= -1.0f;
    z.y *= -1.0f;
    z.z *= -1.0f;
    sd_vec3_normalize(z);

    SDVec3 x = sd_vec3_cross(up, z);
    sd_vec3_normalize(x);

    SDVec3 y = sd_vec3_cross(z, x);
    sd_vec3_normalize(y);


    SDMat4 result = SDMat4(
        x.x, x.y, x.z, -sd_vec3_dot(x, pos),
        y.x, y.y, y.z, -sd_vec3_dot(y, pos),
        z.x, z.y, z.z, -sd_vec3_dot(z, pos),
          0,   0,   0, 1
    );
    return result;
}


SDMat4 sd_mat4_frustum(float l, float r, float b, float t, float n, float f)
{
    SDMat4 result = SDMat4(
        (2.0f * n) / (r - l),                    0,    (r + l) / (r - l),                     0, 
                           0, (2.0f * n) / (t - b),    (t + b) / (t - b),                     0, 
                           0,                    0, (-(f + n)) / (f - n), (-2 * f * n) / (f - n),  
                           0,                    0,                   -1,                     0
    );
    return result;
}

SDMat4 sd_mat4_perspective(float fov, float aspect, float znear, float zfar) 
{
    float ymax = znear * std::tanf(fov * 3.14159265359f / 360.0f);
    float xmax = ymax * aspect;
    return sd_mat4_frustum(-xmax, xmax, -ymax, ymax, znear, zfar);
}

SDMat4 sd_mat4_inverse(const SDMat4& m) {
    SDVec3 a = reinterpret_cast<const SDVec3&>(m[0]);
    SDVec3 b = reinterpret_cast<const SDVec3&>(m[1]);
    SDVec3 c = reinterpret_cast<const SDVec3&>(m[2]);
    SDVec3 d = reinterpret_cast<const SDVec3&>(m[3]);

    f32 x = m(3, 0);
    f32 y = m(3, 1);
    f32 z = m(3, 2);
    f32 w = m(3, 3);

    SDVec3 s = sd_vec3_cross(a, b);
    SDVec3 t = sd_vec3_cross(c, d);
    SDVec3 u = a * y - b * x;
    SDVec3 v = c * w - d * z;

    f32 inv_det = 1.0f / (sd_vec3_dot(s, v) + sd_vec3_dot(t, u));
    s *= inv_det;
    t *= inv_det;
    u *= inv_det;
    v *= inv_det;

    SDVec3 r0 = sd_vec3_cross(b, v) + t * y;
    SDVec3 r1 = sd_vec3_cross(v, a) - t * x;
    SDVec3 r2 = sd_vec3_cross(d, u) + s * w;
    SDVec3 r3 = sd_vec3_cross(u, c) - s * z;

    return SDMat4(r0.x, r0.y, r0.z, -sd_vec3_dot(b, t),
                  r1.x, r1.y, r1.z,  sd_vec3_dot(a, t),
                  r2.x, r2.y, r2.z, -sd_vec3_dot(d, s),
                  r3.x, r3.y, r3.z,  sd_vec3_dot(c, s));

}


SDMat3 sd_mat4_to_mat3(SDMat4 m) {
    SDMat3 result;
    result.m[0][0] = m.m[0][0];
    result.m[0][1] = m.m[0][1];
    result.m[0][2] = m.m[0][2];
    result.m[1][0] = m.m[1][0];
    result.m[1][1] = m.m[1][1];
    result.m[1][2] = m.m[1][2];
    result.m[2][0] = m.m[2][0];
    result.m[2][1] = m.m[2][1];
    result.m[2][2] = m.m[2][2];
    return result;
}

SDVec3 sd_mat4_transform_vector(SDMat4 m, SDVec3 v) {
    SDVec3 result = sd_vec4_to_vec3(m * Vec4(v.x, v.y, v.z, 0.0f));
    return result;
}

SDVec3 sd_mat4_transform_point(SDMat4 m, SDVec3 v) {
    SDVec3 result = sd_vec4_to_vec3(m * Vec4(v.x, v.y, v.z, 1.0f));
    return result;
}

//==================================================================
// Quaternions
//==================================================================

f32 SDQuat::operator[](i32 index) {
    return v[index];
}

SDQuat SDQuat::operator*(float f) {
    SDQuat result;
    result.w = w * f;
    result.x = x * f;
    result.y = y * f;
    result.z = z * f;
    return result;
}

SDQuat SDQuat::operator/(float f) {
    SDQuat result = (*this) * (1.0f / f);
    return result;
}

SDQuat SDQuat::operator+(const SDQuat &q) {
    SDQuat result;
    result.w = w * q.w;
    result.x = x * q.x;
    result.y = y * q.y;
    result.z = z * q.z;
    return result;
}

SDQuat SDQuat::operator*(const SDQuat &q) {
	SDQuat result = SDQuat(
		-q.x * x - q.y * y - q.z * z + q.w * w,
         q.x * w + q.y * z - q.z * y + q.w * x,
		-q.x * z + q.y * w + q.z * x + q.w * y,
	     q.x * y - q.y * x + q.z * w + q.w * z
	);
    return result;
}

void SDQuat::operator*=(const SDQuat &q) {
	SDQuat result = SDQuat(
		-q.x * x - q.y * y - q.z * z + q.w * w,
         q.x * w + q.y * z - q.z * y + q.w * x,
		-q.x * z + q.y * w + q.z * x + q.w * y,
	     q.x * y - q.y * x + q.z * w + q.w * z
	);
    (*this) = result;
}

SDQuat SDQuat::operator+(SDVec3 v) {
    SDQuat q = SDQuat(0, v.x, v.y, v.z);
    q *= *this;
    SDQuat result;
    result.w = w + q.w * 0.5f;
    result.x = x + q.x * 0.5f;
    result.y = y + q.y * 0.5f;
    result.z = z + q.z * 0.5f;
    return result;
}

void SDQuat::operator+=(SDVec3 v) {
    SDQuat q = SDQuat(0, v.x, v.y, v.z);
    q *= *this;
    w += q.w * 0.5f;
    x += q.x * 0.5f;
    y += q.y * 0.5f;
    z += q.z * 0.5f;   
}

SDVec3 SDQuat::operator*(SDVec3 v) {
    SDVec3 result =  vector * 2.0f * sd_vec3_dot(vector, v) +
                     v * (scalar * scalar - sd_vec3_dot(vector, vector)) +
                     sd_vec3_cross(vector, v) * 2.0f * scalar;
    return result;
}

void sd_quat_normalize(SDQuat& q) {
    f32 len_sq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if(len_sq > 0) {
        f32 inv_len = 1.0f / std::sqrtf(len_sq);
        q.w *= inv_len;
        q.x *= inv_len;
        q.y *= inv_len;
        q.z *= inv_len;
    }
}

SDQuat sd_quat_normalized(const SDQuat& q) {

    f32 len_sq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if(len_sq > 0) {
        float inv_len = 1.0f / std::sqrt(len_sq);
        SDQuat result {
            q.w * inv_len,
            q.x * inv_len,
            q.y * inv_len,
            q.z * inv_len
        };
        return result;
    }
    return q;
}
SDMat4 sd_quat_to_mat4(const SDQuat& q) {
    SDMat4 result;

    result.m[0][0] = 1 - 2*q.y*q.y - 2*q.z*q.z;
    result.m[1][0] = 2 * (q.x*q.y - q.w*q.z);
    result.m[2][0] = 2 * (q.x*q.z + q.w*q.y);
    result.m[3][0] = 0;

    result.m[0][1] = 2 * (q.x*q.y + q.w*q.z);
    result.m[1][1] = 1 - 2*q.x*q.x - 2*q.z*q.z;
    result.m[2][1] = 2 * (q.y*q.z - q.w*q.x);
    result.m[3][1] = 0;

    result.m[0][2]  = 2 * (q.x*q.z - q.w*q.y);
    result.m[1][2]  = 2 * (q.y*q.z + q.w*q.x);
    result.m[2][2] = 1 - 2*q.x*q.x - 2*q.y*q.y;
    result.m[3][2] = 0;

    result.m[0][3] = 0;
    result.m[1][3] = 0;
    result.m[2][3] = 0;
    result.m[3][3] = 1;

    return result;
}
SDQuat sd_quat_slerp(SDQuat a, SDQuat b, f32 t) {
    f32 k0, k1;

    f32 cos_omega = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    if(cos_omega < 0.0f) {
        b.w = -b.w;
        b.x = -b.x;
        b.y = -b.y;
        b.z = -b.z;
        cos_omega = -cos_omega;
    }
    
    if(cos_omega > 0.9999f) {
        k0 = (1 - t);
        k1 = t;
    } else {
        
        f32 sin_omega = std::sqrtf(1.0f - cos_omega*cos_omega);
        f32 omega = std::atan2(sin_omega, cos_omega);
        f32 one_over_sin_omega = 1.0f / sin_omega;
        k0 = std::sin((1.0f - t) * omega) * one_over_sin_omega;
        k1 = std::sin(t * omega) * one_over_sin_omega;
    }

    SDQuat result;

    result.w = a.w*k0 + b.w*k1;
    result.x = a.x*k0 + b.x*k1;
    result.y = a.y*k0 + b.y*k1;
    result.z = a.z*k0 + b.z*k1;

    return result; 
}
SDQuat sd_quat_angle_axis(f32 angle, const SDVec3 &axis) {
    SDVec3 norm = sd_vec3_normalized(axis);
    f32 s = std::sinf(angle * 0.5f);
    return  SDQuat(std::cosf(angle * 0.5f),
                 norm.x * s,
                 norm.y * s,
                 norm.z * s);
}


