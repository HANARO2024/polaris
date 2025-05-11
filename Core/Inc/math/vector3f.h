/**
 * @file vector3f.h
 * @brief 3차원 벡터 연산을 위한 라이브러리
 */

#ifndef VECTOR3F_H
#define VECTOR3F_H

#include <stdbool.h>

/**
 * @brief 3차원 벡터 구조체
 */
typedef struct {
    float x;  /**< x 성분 */
    float y;  /**< y 성분 */
    float z;  /**< z 성분 */
} Vector3f;

/**
 * @brief 영벡터 생성
 * 
 * @return Vector3f 영벡터 (0, 0, 0)
 */
Vector3f vector3f_zero(void);

/**
 * @brief 벡터 생성
 * 
 * @param x x 성분
 * @param y y 성분
 * @param z z 성분
 * @return Vector3f 생성된 벡터
 */
Vector3f vector3f_create(float x, float y, float z);

/**
 * @brief 벡터 덧셈
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return Vector3f 덧셈 결과 벡터
 */
Vector3f vector3f_add(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 뺄셈
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return Vector3f 뺄셈 결과 벡터 (v1 - v2)
 */
Vector3f vector3f_subtract(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 스칼라 곱
 * 
 * @param v 벡터
 * @param scalar 스칼라 값
 * @return Vector3f 스칼라 곱 결과 벡터
 */
Vector3f vector3f_scale(Vector3f v, float scalar);

/**
 * @brief 벡터 내적
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return float 내적 결과 값
 */
float vector3f_dot(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 외적
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return Vector3f 외적 결과 벡터
 */
Vector3f vector3f_cross(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 크기
 * 
 * @param v 벡터
 * @return float 벡터의 크기
 */
float vector3f_magnitude(Vector3f v);

/**
 * @brief 벡터 정규화
 * 
 * @param v 벡터
 * @return Vector3f 정규화된 벡터
 */
Vector3f vector3f_normalize(Vector3f v);

/**
 * @brief 두 벡터 사이의 각도
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return float 두 벡터 사이의 각도 (라디안)
 */
float vector3f_angle(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터의 제곱 크기
 * 
 * @param v 벡터
 * @return float 벡터의 제곱 크기
 */
float vector3f_magnitude_squared(Vector3f v);

/**
 * @brief 두 벡터가 같은지 확인
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @param epsilon 허용 오차
 * @return bool 두 벡터가 같으면 true, 다르면 false
 */
bool vector3f_equals(Vector3f v1, Vector3f v2, float epsilon);

#endif /* VECTOR3F_H */
