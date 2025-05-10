/**
 * @file quaternion.h
 * @brief 사원수(Quaternion) 연산을 위한 라이브러리
 */

#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdint.h>
#include <math.h>

/**
 * @brief 사원수 구조체
 * 
 * 사원수는 q = w + xi + yj + zk 형태로 표현됨
 * w는 스칼라 부분, (x, y, z)는 벡터 부분
 */
typedef struct {
    float w;  /**< 스칼라 부분 */
    float x;  /**< 벡터 부분 x */
    float y;  /**< 벡터 부분 y */
    float z;  /**< 벡터 부분 z */
} Quaternion;

/**
 * @brief 3차원 벡터 구조체
 */
typedef struct {
    float x;  /**< x 성분 */
    float y;  /**< y 성분 */
    float z;  /**< z 성분 */
} Vector3f;

/**
 * @brief 단위 사원수 생성
 * 
 * @return Quaternion 단위 사원수 (1, 0, 0, 0)
 */
Quaternion quaternion_identity(void);

/**
 * @brief 사원수 초기화
 * 
 * @param w 스칼라 부분
 * @param x 벡터 부분 x
 * @param y 벡터 부분 y
 * @param z 벡터 부분 z
 * @return Quaternion 초기화된 사원수
 */
Quaternion quaternion_create(float w, float x, float y, float z);

/**
 * @brief 사원수 정규화
 * 
 * @param q 정규화할 사원수
 * @return Quaternion 정규화된 사원수
 */
Quaternion quaternion_normalize(Quaternion q);

/**
 * @brief 사원수 곱셈
 * 
 * @param q1 첫 번째 사원수
 * @param q2 두 번째 사원수
 * @return Quaternion 곱셈 결과
 */
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

/**
 * @brief 사원수 켤레(Conjugate)
 * 
 * @param q 원본 사원수
 * @return Quaternion 켤레 사원수
 */
Quaternion quaternion_conjugate(Quaternion q);

/**
 * @brief 사원수 역원(Inverse)
 * 
 * @param q 원본 사원수
 * @return Quaternion 역원 사원수
 */
Quaternion quaternion_inverse(Quaternion q);

/**
 * @brief 사원수 크기(Magnitude)
 * 
 * @param q 사원수
 * @return float 사원수의 크기
 */
float quaternion_magnitude(Quaternion q);

/**
 * @brief 사원수로 벡터 회전
 * 
 * @param q 회전을 나타내는 사원수
 * @param v 회전할 벡터
 * @return Vector3f 회전된 벡터
 */
Vector3f quaternion_rotate_vector(Quaternion q, Vector3f v);

/**
 * @brief 각속도 벡터로부터 사원수 미분 계산
 * 
 * @param q 현재 사원수
 * @param omega 각속도 벡터 (rad/s)
 * @return Quaternion 사원수 미분값
 */
Quaternion quaternion_derivative(Quaternion q, Vector3f omega);

/**
 * @brief 오일러 각(roll, pitch, yaw)에서 사원수 생성
 * 
 * @param roll x축 회전 (라디안)
 * @param pitch y축 회전 (라디안)
 * @param yaw z축 회전 (라디안)
 * @return Quaternion 변환된 사원수
 */
Quaternion quaternion_from_euler(float roll, float pitch, float yaw);

/**
 * @brief 사원수에서 오일러 각 추출
 * 
 * @param q 사원수
 * @param roll x축 회전 (라디안)을 저장할 포인터
 * @param pitch y축 회전 (라디안)을 저장할 포인터
 * @param yaw z축 회전 (라디안)을 저장할 포인터
 */
void quaternion_to_euler(Quaternion q, float *roll, float *pitch, float *yaw);

/**
 * @brief 영벡터 생성
 * 
 * @return Vector3f (0, 0, 0)
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
 * @return Vector3f 덧셈 결과
 */
Vector3f vector3f_add(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 뺄셈
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return Vector3f 뺄셈 결과
 */
Vector3f vector3f_subtract(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 스칼라 곱
 * 
 * @param v 벡터
 * @param scalar 스칼라 값
 * @return Vector3f 스칼라 곱 결과
 */
Vector3f vector3f_scale(Vector3f v, float scalar);

/**
 * @brief 벡터 내적
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return float 내적 결과
 */
float vector3f_dot(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 외적
 * 
 * @param v1 첫 번째 벡터
 * @param v2 두 번째 벡터
 * @return Vector3f 외적 결과
 */
Vector3f vector3f_cross(Vector3f v1, Vector3f v2);

/**
 * @brief 벡터 정규화
 * 
 * @param v 정규화할 벡터
 * @return Vector3f 정규화된 벡터
 */
Vector3f vector3f_normalize(Vector3f v);

/**
 * @brief 벡터 크기
 * 
 * @param v 벡터
 * @return float 벡터의 크기
 */
float vector3f_magnitude(Vector3f v);

#endif /* QUATERNION_H */
