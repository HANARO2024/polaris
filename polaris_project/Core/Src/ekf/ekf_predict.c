/**
 * @file ekf_predict.c
 * @brief 확장 칼만 필터(Extended Kalman Filter) 예측 단계 구현
 */

#include "ekf/ekf.h"
#include <math.h>

/**
 * @brief 자코비안 행렬 계산 (상태 전이 행렬)
 * 
 * @param ekf EKF 구조체 포인터
 * @param F 자코비안 행렬 (상태 전이 행렬)
 * @param dt 시간 간격 (초)
 * @return bool 계산 성공 여부
 */
static bool ekf_compute_jacobian(EKF *ekf, Matrix *F, float dt) {
    if (ekf == NULL || F == NULL) {
        return false;
    }
    
    // 자코비안 행렬 초기화 (단위 행렬로)
    *F = matrix_identity(EKF_STATE_DIM);
    
    // 위치-속도 관계 (위치 변화율 = 속도)
    matrix_set(F, EKF_STATE_POS_X, EKF_STATE_VEL_X, dt);
    matrix_set(F, EKF_STATE_POS_Y, EKF_STATE_VEL_Y, dt);
    matrix_set(F, EKF_STATE_POS_Z, EKF_STATE_VEL_Z, dt);
    
    // 현재 자세 사원수 추출
    float qw, qx, qy, qz;
    matrix_get(&ekf->x, EKF_STATE_QUAT_W, 0, &qw);
    matrix_get(&ekf->x, EKF_STATE_QUAT_X, 0, &qx);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Y, 0, &qy);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Z, 0, &qz);
    
    // 현재 속도 추출
    float vx, vy, vz;
    matrix_get(&ekf->x, EKF_STATE_VEL_X, 0, &vx);
    matrix_get(&ekf->x, EKF_STATE_VEL_Y, 0, &vy);
    matrix_get(&ekf->x, EKF_STATE_VEL_Z, 0, &vz);
    
    // 현재 가속도 바이어스 추출
    float bax, bay, baz;
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_X, 0, &bax);
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_Y, 0, &bay);
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_Z, 0, &baz);
    
    // 자세 사원수 정규화
    float qnorm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (qnorm < 1e-6f) {
        qw = 1.0f;
        qx = qy = qz = 0.0f;
    } else {
        qw /= qnorm;
        qx /= qnorm;
        qy /= qnorm;
        qz /= qnorm;
    }
    
    // 자세-속도 관계 (회전 행렬의 영향)
    // 이 부분은 가속도가 자세에 따라 어떻게 변환되는지를 나타냄
    // 사원수 업데이트 식: q_dot = 0.5 * q ⊗ (ω - b_ω)
    // 여기서 -0.5 * q ⊗ b_ω는 q_dot에 대한 자세 바이어스의 편미분을 나타냄
    
    // 자이로 바이어스-자세 관계
    matrix_set(F, EKF_STATE_QUAT_W, EKF_STATE_GYRO_BIAS_X, -0.5f * qx * dt);
    matrix_set(F, EKF_STATE_QUAT_W, EKF_STATE_GYRO_BIAS_Y, -0.5f * qy * dt);
    matrix_set(F, EKF_STATE_QUAT_W, EKF_STATE_GYRO_BIAS_Z, -0.5f * qz * dt);
    
    matrix_set(F, EKF_STATE_QUAT_X, EKF_STATE_GYRO_BIAS_X, 0.5f * qw * dt);
    matrix_set(F, EKF_STATE_QUAT_X, EKF_STATE_GYRO_BIAS_Y, -0.5f * qz * dt);
    matrix_set(F, EKF_STATE_QUAT_X, EKF_STATE_GYRO_BIAS_Z, 0.5f * qy * dt);
    
    matrix_set(F, EKF_STATE_QUAT_Y, EKF_STATE_GYRO_BIAS_X, 0.5f * qz * dt);
    matrix_set(F, EKF_STATE_QUAT_Y, EKF_STATE_GYRO_BIAS_Y, 0.5f * qw * dt);
    matrix_set(F, EKF_STATE_QUAT_Y, EKF_STATE_GYRO_BIAS_Z, -0.5f * qx * dt);
    
    matrix_set(F, EKF_STATE_QUAT_Z, EKF_STATE_GYRO_BIAS_X, -0.5f * qy * dt);
    matrix_set(F, EKF_STATE_QUAT_Z, EKF_STATE_GYRO_BIAS_Y, 0.5f * qx * dt);
    matrix_set(F, EKF_STATE_QUAT_Z, EKF_STATE_GYRO_BIAS_Z, 0.5f * qw * dt);
    
    // 가속도 바이어스-속도 관계
    // 회전 행렬 요소 계산 (사원수에서 회전 행렬로 변환)
    float R11 = 1.0f - 2.0f * (qy*qy + qz*qz);
    float R12 = 2.0f * (qx*qy - qw*qz);
    float R13 = 2.0f * (qx*qz + qw*qy);
    float R21 = 2.0f * (qx*qy + qw*qz);
    float R22 = 1.0f - 2.0f * (qx*qx + qz*qz);
    float R23 = 2.0f * (qy*qz - qw*qx);
    float R31 = 2.0f * (qx*qz - qw*qy);
    float R32 = 2.0f * (qy*qz + qw*qx);
    float R33 = 1.0f - 2.0f * (qx*qx + qy*qy);
    
    // 가속도 바이어스가 속도에 미치는 영향
    matrix_set(F, EKF_STATE_VEL_X, EKF_STATE_ACC_BIAS_X, -R11 * dt);
    matrix_set(F, EKF_STATE_VEL_X, EKF_STATE_ACC_BIAS_Y, -R12 * dt);
    matrix_set(F, EKF_STATE_VEL_X, EKF_STATE_ACC_BIAS_Z, -R13 * dt);
    
    matrix_set(F, EKF_STATE_VEL_Y, EKF_STATE_ACC_BIAS_X, -R21 * dt);
    matrix_set(F, EKF_STATE_VEL_Y, EKF_STATE_ACC_BIAS_Y, -R22 * dt);
    matrix_set(F, EKF_STATE_VEL_Y, EKF_STATE_ACC_BIAS_Z, -R23 * dt);
    
    matrix_set(F, EKF_STATE_VEL_Z, EKF_STATE_ACC_BIAS_X, -R31 * dt);
    matrix_set(F, EKF_STATE_VEL_Z, EKF_STATE_ACC_BIAS_Y, -R32 * dt);
    matrix_set(F, EKF_STATE_VEL_Z, EKF_STATE_ACC_BIAS_Z, -R33 * dt);
    
    return true;
}

/**
 * @brief EKF 예측 단계 (IMU 데이터 기반)
 */
bool ekf_predict(EKF *ekf, Vector3f gyro, Vector3f accel, float dt) {
    if (ekf == NULL || dt <= 0.0f) {
        return false;
    }
    
    if (!ekf->initialized) {
        // 초기화되지 않은 경우 예측 수행하지 않음
        return false;
    }
    
    // 현재 상태 추출
    float px, py, pz;           // 위치
    float vx, vy, vz;           // 속도
    float qw, qx, qy, qz;       // 자세 사원수
    float bgx, bgy, bgz;        // 자이로 바이어스
    float bax, bay, baz;        // 가속도 바이어스
    
    matrix_get(&ekf->x, EKF_STATE_POS_X, 0, &px);
    matrix_get(&ekf->x, EKF_STATE_POS_Y, 0, &py);
    matrix_get(&ekf->x, EKF_STATE_POS_Z, 0, &pz);
    
    matrix_get(&ekf->x, EKF_STATE_VEL_X, 0, &vx);
    matrix_get(&ekf->x, EKF_STATE_VEL_Y, 0, &vy);
    matrix_get(&ekf->x, EKF_STATE_VEL_Z, 0, &vz);
    
    matrix_get(&ekf->x, EKF_STATE_QUAT_W, 0, &qw);
    matrix_get(&ekf->x, EKF_STATE_QUAT_X, 0, &qx);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Y, 0, &qy);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Z, 0, &qz);
    
    matrix_get(&ekf->x, EKF_STATE_GYRO_BIAS_X, 0, &bgx);
    matrix_get(&ekf->x, EKF_STATE_GYRO_BIAS_Y, 0, &bgy);
    matrix_get(&ekf->x, EKF_STATE_GYRO_BIAS_Z, 0, &bgz);
    
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_X, 0, &bax);
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_Y, 0, &bay);
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_Z, 0, &baz);
    
    // 사원수 정규화
    Quaternion q = quaternion_create(qw, qx, qy, qz);
    q = quaternion_normalize(q);
    qw = q.w;
    qx = q.x;
    qy = q.y;
    qz = q.z;
    
    // 바이어스 보정된 자이로 및 가속도 계산
    Vector3f gyro_corrected = vector3f_create(
        gyro.x - bgx,
        gyro.y - bgy,
        gyro.z - bgz
    );
    
    Vector3f accel_corrected = vector3f_create(
        accel.x - bax,
        accel.y - bay,
        accel.z - baz
    );
    
    // 1. 사원수 적분 (자세 업데이트)
    Quaternion q_dot = quaternion_derivative(q, gyro_corrected);
    
    // 새로운 사원수 계산 (오일러 적분)
    qw += q_dot.w * dt;
    qx += q_dot.x * dt;
    qy += q_dot.y * dt;
    qz += q_dot.z * dt;
    
    // 사원수 정규화
    q = quaternion_create(qw, qx, qy, qz);
    q = quaternion_normalize(q);
    qw = q.w;
    qx = q.x;
    qy = q.y;
    qz = q.z;
    
    // 2. 중력 벡터 계산 (NED 좌표계)
    Vector3f gravity_ned = vector3f_create(0.0f, 0.0f, ekf->gravity);
    
    // 3. 가속도를 NED 좌표계로 변환
    Vector3f accel_ned = quaternion_rotate_vector(q, accel_corrected);
    
    // 4. 중력 보정
    accel_ned = vector3f_subtract(accel_ned, gravity_ned);
    
    // 5. 속도 적분
    vx += accel_ned.x * dt;
    vy += accel_ned.y * dt;
    vz += accel_ned.z * dt;
    
    // 6. 위치 적분
    px += vx * dt;
    py += vy * dt;
    pz += vz * dt;
    
    // 7. 상태 벡터 업데이트
    matrix_set(&ekf->x, EKF_STATE_POS_X, 0, px);
    matrix_set(&ekf->x, EKF_STATE_POS_Y, 0, py);
    matrix_set(&ekf->x, EKF_STATE_POS_Z, 0, pz);
    
    matrix_set(&ekf->x, EKF_STATE_VEL_X, 0, vx);
    matrix_set(&ekf->x, EKF_STATE_VEL_Y, 0, vy);
    matrix_set(&ekf->x, EKF_STATE_VEL_Z, 0, vz);
    
    matrix_set(&ekf->x, EKF_STATE_QUAT_W, 0, qw);
    matrix_set(&ekf->x, EKF_STATE_QUAT_X, 0, qx);
    matrix_set(&ekf->x, EKF_STATE_QUAT_Y, 0, qy);
    matrix_set(&ekf->x, EKF_STATE_QUAT_Z, 0, qz);
    
    // 바이어스는 변경하지 않음 (측정 갱신 단계에서 조정)
    
    // 8. 자코비안 행렬 계산
    Matrix F;
    ekf_compute_jacobian(ekf, &F, dt);
    
    // 9. 공분산 행렬 전파
    // P = F * P * F^T + Q
    Matrix F_transpose;
    matrix_transpose(&F, &F_transpose);
    
    Matrix temp1, temp2;
    matrix_multiply(&F, &ekf->P, &temp1);
    matrix_multiply(&temp1, &F_transpose, &temp2);
    
    // Q를 더함 (프로세스 노이즈)
    Matrix scaled_Q;
    matrix_scale(&ekf->Q, dt, &scaled_Q);
    matrix_add(&temp2, &scaled_Q, &ekf->P);
    
    return true;
}
