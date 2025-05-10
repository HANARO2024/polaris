/**
 * @file vector3f.c
 * @brief 3차원 벡터 연산을 위한 라이브러리 구현
 */

#include "math/vector3f.h"
#include <math.h>

/**
 * @brief 영벡터 생성
 */
Vector3f vector3f_zero(void) {
    Vector3f v;
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
    return v;
}

/**
 * @brief 벡터 생성
 */
Vector3f vector3f_create(float x, float y, float z) {
    Vector3f v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

/**
 * @brief 벡터 덧셈
 */
Vector3f vector3f_add(Vector3f v1, Vector3f v2) {
    Vector3f result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
    return result;
}

/**
 * @brief 벡터 뺄셈
 */
Vector3f vector3f_subtract(Vector3f v1, Vector3f v2) {
    Vector3f result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    result.z = v1.z - v2.z;
    return result;
}

/**
 * @brief 벡터 스칼라 곱
 */
Vector3f vector3f_scale(Vector3f v, float scalar) {
    Vector3f result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    result.z = v.z * scalar;
    return result;
}

/**
 * @brief 벡터 내적
 */
float vector3f_dot(Vector3f v1, Vector3f v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/**
 * @brief 벡터 외적
 */
Vector3f vector3f_cross(Vector3f v1, Vector3f v2) {
    Vector3f result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

/**
 * @brief 벡터 크기
 */
float vector3f_magnitude(Vector3f v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * @brief 벡터의 제곱 크기
 */
float vector3f_magnitude_squared(Vector3f v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

/**
 * @brief 벡터 정규화
 */
Vector3f vector3f_normalize(Vector3f v) {
    float mag = vector3f_magnitude(v);
    Vector3f result = v;
    
    if (mag > 1e-6f) {
        float inv_mag = 1.0f / mag;
        result.x *= inv_mag;
        result.y *= inv_mag;
        result.z *= inv_mag;
    }
    
    return result;
}

/**
 * @brief 두 벡터 사이의 각도
 */
float vector3f_angle(Vector3f v1, Vector3f v2) {
    float mag1 = vector3f_magnitude(v1);
    float mag2 = vector3f_magnitude(v2);
    
    if (mag1 < 1e-6f || mag2 < 1e-6f) {
        return 0.0f;
    }
    
    float dot = vector3f_dot(v1, v2);
    float cos_angle = dot / (mag1 * mag2);
    
    // 수치 오차 처리
    if (cos_angle > 1.0f) {
        cos_angle = 1.0f;
    } else if (cos_angle < -1.0f) {
        cos_angle = -1.0f;
    }
    
    return acosf(cos_angle);
}

/**
 * @brief 두 벡터가 같은지 확인
 */
bool vector3f_equals(Vector3f v1, Vector3f v2, float epsilon) {
    if (fabsf(v1.x - v2.x) > epsilon) {
        return false;
    }
    
    if (fabsf(v1.y - v2.y) > epsilon) {
        return false;
    }
    
    if (fabsf(v1.z - v2.z) > epsilon) {
        return false;
    }
    
    return true;
}
