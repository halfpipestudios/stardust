#pragma once

#include <cmath>
#include <algorithm>

#include <sd_common.h>

#define VEC_EPSILON 0.000001f

// TODO: Important unify the use of const ref or the not use of it

struct SDVec2
{
    union 
    {
        struct
        {
            float x;
            float y;
        };
        float v[2];
    };
    inline SDVec2() : x(0.0f), y(0.0f) { }
    inline SDVec2(float x_, float y_) : x(x_), y(y_) { }
    inline SDVec2(float *fv) : x(fv[0]), y(fv[1]) { }
    float operator[](int index);

    SDVec2 operator+(const SDVec2& rhs);
    SDVec2 operator-(const SDVec2& rhs);
    SDVec2 operator*(const SDVec2& rhs);
    SDVec2 operator/(const SDVec2& rhs);

    void operator+=(const SDVec2& rhs);
    void operator-=(const SDVec2& rhs);
    void operator*=(const SDVec2& rhs);
    void operator/=(const SDVec2& rhs);
};

float sd_vec2_dot(const SDVec2& lhs, const SDVec2& rhs);
float sd_vec2_len_sq(const SDVec2& v);
float sd_vec2_len(const SDVec2& v);
SDVec2 sd_vec2_normalized(const SDVec2& v);
void sd_vec2_normalize(SDVec2& v);

struct SDVec3
{
    union 
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        float v[3];
    };
    inline SDVec3() : x(0.0f), y(0.0f), z(0.0f) { }
    inline SDVec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) { }
    inline SDVec3(float *fv) : x(fv[0]), y(fv[1]), z(fv[2]) { }
    float operator[](int index);

    SDVec3 operator+(const SDVec3& rhs);
    SDVec3 operator-(const SDVec3& rhs);
    SDVec3 operator*(const SDVec3& rhs);
    SDVec3 operator/(const SDVec3& rhs);

    SDVec3 operator+(const SDVec3& rhs) const;
    SDVec3 operator-(const SDVec3& rhs) const;
    SDVec3 operator*(const SDVec3& rhs) const;
    SDVec3 operator/(const SDVec3& rhs) const;

    void operator+=(const SDVec3& rhs);
    void operator-=(const SDVec3& rhs);
    void operator*=(const SDVec3& rhs);
    void operator/=(const SDVec3& rhs);

    SDVec3 operator+(float rhs);
    SDVec3 operator-(float rhs);
    SDVec3 operator*(float rhs);
    SDVec3 operator/(float rhs);

    void operator+=(float rhs);
    void operator-=(float rhs);
    void operator*=(float rhs);
    void operator/=(float rhs);
};

float sd_vec3_dot(const SDVec3& lhs, const SDVec3& rhs);
SDVec3 sd_vec3_cross(const SDVec3& lhs, const SDVec3& rhs);
float sd_vec3_len_sq(const SDVec3& v);
float sd_vec3_len(const SDVec3& v);
SDVec3 sd_vec3_normalized(const SDVec3& v);
void sd_vec3_normalize(SDVec3& v);
SDVec3 sd_vec3_lerp(SDVec3 a, SDVec3 b, f32 t);
f32 sd_vec3_angle(SDVec3 a, SDVec3 b);

struct Vec4
{
    union 
    {
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
        float v[4];
    };
    inline Vec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) { }
    inline Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) { }
    inline Vec4(float *fv) : x(fv[0]), y(fv[1]), z(fv[2]), w(fv[3]) { }
    float operator[](int index);


    Vec4 operator+(const Vec4& rhs);
    Vec4 operator-(const Vec4& rhs);
    Vec4 operator*(const Vec4& rhs);
    Vec4 operator/(const Vec4& rhs);

    void operator+=(const Vec4& rhs);
    void operator-=(const Vec4& rhs);
    void operator*=(const Vec4& rhs);
    void operator/=(const Vec4& rhs);

    Vec4 operator+(float rhs);
    Vec4 operator-(float rhs);
    Vec4 operator*(float rhs);
    Vec4 operator/(float rhs);

};

SDVec3 sd_vec4_to_vec3(const Vec4& v);

struct SDMat3
{
    float m[3][3];

    inline SDMat3()
    {
        // Colum major matrix
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
    }

    inline SDMat3(float m00, float m01, float m02,
                float m10, float m11, float m12,
                float m20, float m21, float m22)
    {
        // Colum major matrix
        m[0][0] = m00; m[0][1] = m10; m[0][2] = m20;
        m[1][0] = m01; m[1][1] = m11; m[1][2] = m21;
        m[2][0] = m01; m[2][1] = m12; m[2][2] = m22;
    }

    float operator()(int i, int j);
    float operator()(int i, int j) const;
    SDVec3& operator[](int col);
    const SDVec3& operator[](int col) const;

    SDMat3 operator*(const SDMat3& rhs);
    SDVec3 operator*(const SDVec3& rhs);
};

SDMat3 sd_mat3_rotation_x(float angle);
SDMat3 sd_mat3_rotation_y(float angle);
SDMat3 sd_mat3_rotation_z(float angle);
SDMat3 sd_mat3_scale(float x, float y, float z);
SDMat3 sd_mat3_inverse(const SDMat3& m);

struct SDMat4
{
    float m[4][4];

    inline SDMat4()
    {
        // Colum major matrix
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
        m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
    }

    inline SDMat4(float m00, float m01, float m02, float m03,
                float m10, float m11, float m12, float m13,
                float m20, float m21, float m22, float m23,
                float m30, float m31, float m32, float m33)
    {
        // Colum major matrix
        m[0][0] = m00; m[0][1] = m10; m[0][2] = m20; m[0][3] = m30;
        m[1][0] = m01; m[1][1] = m11; m[1][2] = m21; m[1][3] = m31;
        m[2][0] = m02; m[2][1] = m12; m[2][2] = m22; m[2][3] = m32;
        m[3][0] = m03; m[3][1] = m13; m[3][2] = m23; m[3][3] = m33;
    }
    
    float operator()(int i, int j);
    float operator()(int i, int j) const;
    Vec4& operator[](int col);
    const Vec4& operator[](int col) const;

    SDMat4 operator*(const SDMat4& rhs);
    Vec4 operator*(const Vec4& rhs);
};

SDVec3 sd_mat4_get_col_as_vec3(SDMat4& m, int col);

SDMat4 sd_mat4_translation(float x, float y, float z);
SDMat4 sd_mat4_rotation_x(float angle);
SDMat4 sd_mat4_rotation_y(float angle);
SDMat4 sd_mat4_rotation_z(float angle);
SDMat4 sd_mat4_scale(float x, float y, float z);

SDMat4 sd_mat4_translation(SDVec3 v);
SDMat4 sd_mat4_scale(SDVec3 v);

SDMat4 sd_mat4_lookat(SDVec3 pos, SDVec3 tar, SDVec3 up);
SDMat4 sd_mat4_frustum(float l, float r, float b, float t, float n, float f);
SDMat4 sd_mat4_perspective(float fov, float aspect, float znear, float zfar);

SDMat4 sd_mat4_inverse(const SDMat4& m);

struct SDQuat {
    union {
        struct {
            f32 w, x, y, z;
        };
        struct {
            f32 scalar;
            SDVec3 vector;
        };
        f32 v[4];
    };

    SDQuat() : w(1), x(0), y(0), z(0) {}
    SDQuat(f32 w_, f32 x_, f32 y_, f32 z_) : w(w_), x(x_), y(y_), z(z_) {}
    SDQuat(SDVec3 v) : w(1), x(v.x), y(v.y), z(v.z) {}
    
    f32 operator[](i32 index);
    SDQuat operator*(float f);
    SDQuat operator/(float f);
    SDQuat operator+(const SDQuat &q);
    SDQuat operator*(const SDQuat &q);
    SDVec3 operator*(SDVec3 v);
};

void sd_quat_normalize(SDQuat& q);
SDQuat sd_quat_normalized(const SDQuat& q);
SDMat4 sd_quat_to_mat4(const SDQuat& q);
SDQuat sd_quat_slerp(SDQuat a, SDQuat b, f32 t);
SDQuat sd_quat_angle_axis(f32 angle, const SDVec3 &axis);