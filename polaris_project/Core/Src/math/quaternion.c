/**
 * @file quaternion.c
 * @brief 사원수(Quaternion) 연산을 위한 라이브러리 구현
 */

#include "math/quaternion.h"
#include <math.h>

/**
 * @brief 단위 사원수 생성
 */
Quaternion quaternion_identity(void) {
    Quaternion q;
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    return q;
}

/**
 * @brief 사원수 초기화
 */
Quaternion quaternion_create(float w, float x, float y, float z) {
    Quaternion q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}

/**
 * @brief 사원수 정규화
 */
Quaternion quaternion_normalize(Quaternion q) {
    float magnitude = quaternion_magnitude(q);
    
    // 0으로 나누기 방지
    if (magnitude < 1e-6f) {
        return quaternion_identity();
    }
    
    Quaternion result;
    float inv_magnitude = 1.0f / magnitude;
    result.w = q.w * inv_magnitude;
    result.x = q.x * inv_magnitude;
    result.y = q.y * inv_magnitude;
    result.z = q.z * inv_magnitude;
    
    return result;
}

/**
 * @brief 사원수 곱셈
 */
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    
    return result;
}

/**
 * @brief 사원수 켤레(Conjugate)
 */
Quaternion quaternion_conjugate(Quaternion q) {
    Quaternion result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

/**
 * @brief 사원수 역원(Inverse)
 */
Quaternion quaternion_inverse(Quaternion q) {
    Quaternion conjugate = quaternion_conjugate(q);
    float magnitude_squared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    
    // 0으로 나누기 방지
    if (magnitude_squared < 1e-6f) {
        return quaternion_identity();
    }
    
    float inv_magnitude_squared = 1.0f / magnitude_squared;
    
    Quaternion result;
    result.w = conjugate.w * inv_magnitude_squared;
    result.x = conjugate.x * inv_magnitude_squared;
    result.y = conjugate.y * inv_magnitude_squared;
    result.z = conjugate.z * inv_magnitude_squared;
    
    return result;
}

/**
 * @brief 사원수 크기(Magnitude)
 */
float quaternion_magnitude(Quaternion q) {
    return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

/**
 * @brief 사원수로 벡터 회전
 */
Vector3f quaternion_rotate_vector(Quaternion q, Vector3f v) {
    // v' = q * v * q^(-1)
    // 여기서 v는 (0, v.x, v.y, v.z) 형태의 순수 사원수
    
    // 최적화된 구현 (사원수 곱셈 공식 직접 적용)
    Vector3f result;
    
    float qw2 = q.w * q.w;
    float qx2 = q.x * q.x;
    float qy2 = q.y * q.y;
    float qz2 = q.z * q.z;
    
    float qwx = q.w * q.x;
    float qwy = q.w * q.y;
    float qwz = q.w * q.z;
    float qxy = q.x * q.y;
    float qxz = q.x * q.z;
    float qyz = q.y * q.z;
    
    // 회전 행렬 요소들
    float m11 = qw2 + qx2 - qy2 - qz2;
    float m12 = 2.0f * (qxy - qwz);
    float m13 = 2.0f * (qxz + qwy);
    
    float m21 = 2.0f * (qxy + qwz);
    float m22 = qw2 - qx2 + qy2 - qz2;
    float m23 = 2.0f * (qyz - qwx);
    
    float m31 = 2.0f * (qxz - qwy);
    float m32 = 2.0f * (qyz + qwx);
    float m33 = qw2 - qx2 - qy2 + qz2;
    
    // 행렬-벡터 곱
    result.x = m11 * v.x + m12 * v.y + m13 * v.z;
    result.y = m21 * v.x + m22 * v.y + m23 * v.z;
    result.z = m31 * v.x + m32 * v.y + m33 * v.z;
    
    return result;
}

/**
 * @brief 각속도 벡터로부터 사원수 미분 계산
 */
Quaternion quaternion_derivative(Quaternion q, Vector3f omega) {
    // q̇ = 0.5 * q ⊗ ω
    // 여기서 ω는 (0, ω.x, ω.y, ω.z) 형태의 순수 사원수
    
    Quaternion omega_quat;
    omega_quat.w = 0.0f;
    omega_quat.x = omega.x;
    omega_quat.y = omega.y;
    omega_quat.z = omega.z;
    
    Quaternion q_dot = quaternion_multiply(q, omega_quat);
    
    // 0.5를 곱함
    q_dot.w *= 0.5f;
    q_dot.x *= 0.5f;
    q_dot.y *= 0.5f;
    q_dot.z *= 0.5f;
    
    return q_dot;
}

/**
 * @brief 오일러 각(roll, pitch, yaw)에서 사원수 생성
 */
Quaternion quaternion_from_euler(float roll, float pitch, float yaw) {
    // 각 축에 대한 회전을 나타내는 사원수 계산
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    // ZYX 순서로 회전 (항공우주 컨벤션)
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return quaternion_normalize(q);
}

/**
 * @brief 사원수에서 오일러 각 추출
 */
void quaternion_to_euler(Quaternion q, float *roll, float *pitch, float *yaw) {
    // 사원수를 정규화
    Quaternion qn = quaternion_normalize(q);
    
    // Roll (x-axis rotation)
    *roll = atan2f(2.0f * (qn.w * qn.x + qn.y * qn.z), 
                  1.0f - 2.0f * (qn.x * qn.x + qn.y * qn.y));
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qn.w * qn.y - qn.z * qn.x);
    if (fabsf(sinp) >= 1.0f) {
        // 특이점 처리 (90도)
        *pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        *pitch = asinf(sinp);
    }
    
    // Yaw (z-axis rotation)
    *yaw = atan2f(2.0f * (qn.w * qn.z + qn.x * qn.y), 
                 1.0f - 2.0f * (qn.y * qn.y + qn.z * qn.z));
}

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
 * @brief 벡터 정규화
 */
Vector3f vector3f_normalize(Vector3f v) {
    float magnitude = vector3f_magnitude(v);
    
    // 0으로 나누기 방지
    if (magnitude < 1e-6f) {
        return vector3f_zero();
    }
    
    Vector3f result;
    float inv_magnitude = 1.0f / magnitude;
    result.x = v.x * inv_magnitude;
    result.y = v.y * inv_magnitude;
    result.z = v.z * inv_magnitude;
    
    return result;
}

/**
 * @brief 벡터 크기
 */
float vector3f_magnitude(Vector3f v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}
