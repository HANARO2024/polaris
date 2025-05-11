/**
 * @file ekf_update.c
 * @brief 확장 칼만 필터(Extended Kalman Filter) 측정 갱신 단계 구현
 */

#include "ekf/ekf.h"
#include <math.h>

/**
 * @brief GPS 측정 갱신을 위한 측정 자코비안 계산
 * 
 * @param ekf EKF 구조체 포인터
 * @param H 측정 자코비안 행렬 (6x16)
 * @return bool 계산 성공 여부
 */
static bool ekf_compute_gps_jacobian(EKF *ekf, Matrix *H) {
    if (ekf == NULL || H == NULL) {
        return false;
    }
    
    // 측정 자코비안 초기화 (영행렬)
    *H = matrix_create(6, EKF_STATE_DIM);
    matrix_zero(H);
    
    // GPS 위치 측정에 대한 자코비안
    // 위치 상태에 대한 직접적인 매핑
    matrix_set(H, 0, EKF_STATE_POS_X, 1.0f);
    matrix_set(H, 1, EKF_STATE_POS_Y, 1.0f);
    matrix_set(H, 2, EKF_STATE_POS_Z, 1.0f);
    
    // GPS 속도 측정에 대한 자코비안
    // 속도 상태에 대한 직접적인 매핑
    matrix_set(H, 3, EKF_STATE_VEL_X, 1.0f);
    matrix_set(H, 4, EKF_STATE_VEL_Y, 1.0f);
    matrix_set(H, 5, EKF_STATE_VEL_Z, 1.0f);
    
    return true;
}

/**
 * @brief 기압계 측정 갱신을 위한 측정 자코비안 계산
 * 
 * @param ekf EKF 구조체 포인터
 * @param H 측정 자코비안 행렬 (1x16)
 * @return bool 계산 성공 여부
 */
static bool ekf_compute_baro_jacobian(EKF *ekf, Matrix *H) {
    if (ekf == NULL || H == NULL) {
        return false;
    }
    
    // 측정 자코비안 초기화 (영행렬)
    *H = matrix_create(1, EKF_STATE_DIM);
    matrix_zero(H);
    
    // 기압계 고도 측정은 Z 위치에만 영향
    matrix_set(H, 0, EKF_STATE_POS_Z, 1.0f);
    
    return true;
}

/**
 * @brief 자력계 측정 갱신을 위한 측정 자코비안 계산
 * 
 * @param ekf EKF 구조체 포인터
 * @param H 측정 자코비안 행렬 (3x16)
 * @return bool 계산 성공 여부
 */
static bool ekf_compute_mag_jacobian(EKF *ekf, Matrix *H) {
    if (ekf == NULL || H == NULL) {
        return false;
    }
    
    // 측정 자코비안 초기화 (영행렬)
    *H = matrix_create(3, EKF_STATE_DIM);
    matrix_zero(H);
    
    // 현재 자세 사원수 추출
    float qw, qx, qy, qz;
    matrix_get(&ekf->x, EKF_STATE_QUAT_W, 0, &qw);
    matrix_get(&ekf->x, EKF_STATE_QUAT_X, 0, &qx);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Y, 0, &qy);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Z, 0, &qz);
    
    // 지구 자기장 벡터
    float mx = ekf->earth_mag_ned.x;
    float my = ekf->earth_mag_ned.y;
    float mz = ekf->earth_mag_ned.z;
    
    // 자력계 측정에 대한 자코비안 (자세 사원수에 대한 편미분)
    
    // 자세 사원수에 대한 자력계 측정의 자코비안 계산
    // dh/dq = d(R(q) * m_earth)/dq
    
    // qw에 대한 편미분
    matrix_set(H, 0, EKF_STATE_QUAT_W, 2.0f * (- qz*my + qy*mz));
    matrix_set(H, 1, EKF_STATE_QUAT_W, 2.0f * (qz*mx - qx*mz));
    matrix_set(H, 2, EKF_STATE_QUAT_W, 2.0f * (- qy*mx + qx*my));
    
    // qx에 대한 편미분
    matrix_set(H, 0, EKF_STATE_QUAT_X, 2.0f * (qy*my + qz*mz));
    matrix_set(H, 1, EKF_STATE_QUAT_X, 2.0f * (qy*mx - 2.0f*qx*my - qw*mz));
    matrix_set(H, 2, EKF_STATE_QUAT_X, 2.0f * (qz*mx + qw*my - 2.0f*qx*mz));
    
    // qy에 대한 편미분
    matrix_set(H, 0, EKF_STATE_QUAT_Y, 2.0f * (- 2.0f*qy*mx + qx*my + qw*mz));
    matrix_set(H, 1, EKF_STATE_QUAT_Y, 2.0f * (qx*mx + qz*mz));
    matrix_set(H, 2, EKF_STATE_QUAT_Y, 2.0f * (- qw*mx + qz*my - 2.0f*qy*mz));
    
    // qz에 대한 편미분
    matrix_set(H, 0, EKF_STATE_QUAT_Z, 2.0f * (- 2.0f*qz*mx - qw*my + qx*mz));
    matrix_set(H, 1, EKF_STATE_QUAT_Z, 2.0f * (qw*mx - 2.0f*qz*my + qy*mz));
    matrix_set(H, 2, EKF_STATE_QUAT_Z, 2.0f * (qx*mx + qy*my));
    
    return true;
}

/**
 * @brief 칼만 게인 계산
 * 
 * @param ekf EKF 구조체 포인터
 * @param H 측정 자코비안 행렬
 * @param R 측정 노이즈 공분산 행렬
 * @param K 칼만 게인 (출력)
 * @return bool 계산 성공 여부
 */
static bool ekf_compute_kalman_gain(EKF *ekf, Matrix *H, Matrix *R, Matrix *K) {
    if (ekf == NULL || H == NULL || R == NULL || K == NULL) {
        return false;
    }
    
    // H * P * H^T + R 계산
    Matrix H_transpose;
    matrix_transpose(H, &H_transpose);
    
    Matrix temp1, temp2, S;
    matrix_multiply(H, &ekf->P, &temp1);
    matrix_multiply(&temp1, &H_transpose, &temp2);
    matrix_add(&temp2, R, &S);
    
    // S 역행렬 계산
    Matrix S_inv;
    if (!matrix_inverse(&S, &S_inv)) {
        // 역행렬 계산 실패
        return false;
    }
    
    // K = P * H^T * S^-1
    Matrix temp3;
    matrix_multiply(&ekf->P, &H_transpose, &temp3);
    matrix_multiply(&temp3, &S_inv, K);
    
    return true;
}

/**
 * @brief 상태 및 공분산 갱신
 * 
 * @param ekf EKF 구조체 포인터
 * @param K 칼만 게인
 * @param y 측정 잔차 (측정값 - 예측값)
 * @param H 측정 자코비안 행렬
 * @return bool 갱신 성공 여부
 */
static bool ekf_update_state_covariance(EKF *ekf, Matrix *K, Matrix *y, Matrix *H) {
    if (ekf == NULL || K == NULL || y == NULL || H == NULL) {
        return false;
    }
    
    // 상태 갱신: x = x + K * y
    Matrix dx;
    matrix_multiply(K, y, &dx);
    matrix_add(&ekf->x, &dx, &ekf->x);
    
    // 사원수 정규화
    float qw, qx, qy, qz;
    matrix_get(&ekf->x, EKF_STATE_QUAT_W, 0, &qw);
    matrix_get(&ekf->x, EKF_STATE_QUAT_X, 0, &qx);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Y, 0, &qy);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Z, 0, &qz);
    
    Quaternion q = quaternion_create(qw, qx, qy, qz);
    q = quaternion_normalize(q);
    
    matrix_set(&ekf->x, EKF_STATE_QUAT_W, 0, q.w);
    matrix_set(&ekf->x, EKF_STATE_QUAT_X, 0, q.x);
    matrix_set(&ekf->x, EKF_STATE_QUAT_Y, 0, q.y);
    matrix_set(&ekf->x, EKF_STATE_QUAT_Z, 0, q.z);
    
    // 공분산 갱신: P = (I - K * H) * P
    Matrix I = matrix_identity(EKF_STATE_DIM);
    Matrix KH, I_KH;
    matrix_multiply(K, H, &KH);
    matrix_subtract(&I, &KH, &I_KH);
    
    Matrix P_new;
    matrix_multiply(&I_KH, &ekf->P, &P_new);
    ekf->P = P_new;
    
    // 공분산 행렬의 대칭성 보장
    Matrix P_transpose;
    matrix_transpose(&ekf->P, &P_transpose);
    matrix_add(&ekf->P, &P_transpose, &P_new);
    matrix_scale(&P_new, 0.5f, &ekf->P);
    
    return true;
}

/**
 * @brief GPS 측정 갱신
 */
bool ekf_update_gps(EKF *ekf, Vector3f pos, Vector3f vel) {
    if (ekf == NULL || !ekf->initialized) {
        return false;
    }
    
    // 1. 측정 자코비안 계산
    Matrix H;
    ekf_compute_gps_jacobian(ekf, &H);
    
    // 2. 예측된 측정값 계산
    Vector3f pos_pred = ekf_get_position(ekf);
    Vector3f vel_pred = ekf_get_velocity(ekf);
    
    // 3. 측정 잔차 계산 (측정값 - 예측값)
    Matrix z = matrix_create(6, 1);
    Matrix z_pred = matrix_create(6, 1);
    
    // 측정값
    matrix_set(&z, 0, 0, pos.x);
    matrix_set(&z, 1, 0, pos.y);
    matrix_set(&z, 2, 0, pos.z);
    matrix_set(&z, 3, 0, vel.x);
    matrix_set(&z, 4, 0, vel.y);
    matrix_set(&z, 5, 0, vel.z);
    
    // 예측값
    matrix_set(&z_pred, 0, 0, pos_pred.x);
    matrix_set(&z_pred, 1, 0, pos_pred.y);
    matrix_set(&z_pred, 2, 0, pos_pred.z);
    matrix_set(&z_pred, 3, 0, vel_pred.x);
    matrix_set(&z_pred, 4, 0, vel_pred.y);
    matrix_set(&z_pred, 5, 0, vel_pred.z);
    
    // 잔차
    Matrix y;
    matrix_subtract(&z, &z_pred, &y);
    
    // 4. 칼만 게인 계산
    Matrix K;
    if (!ekf_compute_kalman_gain(ekf, &H, &ekf->R_gps, &K)) {
        return false;
    }
    
    // 5. 상태 및 공분산 갱신
    return ekf_update_state_covariance(ekf, &K, &y, &H);
}

/**
 * @brief 기압계 측정 갱신
 */
bool ekf_update_baro(EKF *ekf, float altitude) {
    if (ekf == NULL || !ekf->initialized) {
        return false;
    }
    
    // 1. 측정 자코비안 계산
    Matrix H;
    ekf_compute_baro_jacobian(ekf, &H);
    
    // 2. 예측된 측정값 계산
    Vector3f pos_pred = ekf_get_position(ekf);
    
    // 3. 측정 잔차 계산 (측정값 - 예측값)
    Matrix z = matrix_create(1, 1);
    Matrix z_pred = matrix_create(1, 1);
    
    // 측정값 (기압계 고도)
    matrix_set(&z, 0, 0, altitude);
    
    // 예측값 (Z 위치)
    matrix_set(&z_pred, 0, 0, pos_pred.z);
    
    // 잔차
    Matrix y;
    matrix_subtract(&z, &z_pred, &y);
    
    // 4. 칼만 게인 계산
    Matrix K;
    if (!ekf_compute_kalman_gain(ekf, &H, &ekf->R_baro, &K)) {
        return false;
    }
    
    // 5. 상태 및 공분산 갱신
    return ekf_update_state_covariance(ekf, &K, &y, &H);
}

/**
 * @brief 자력계 측정 갱신
 */
bool ekf_update_mag(EKF *ekf, Vector3f mag) {
    if (ekf == NULL || !ekf->initialized) {
        return false;
    }
    
    // 1. 측정 자코비안 계산
    Matrix H;
    ekf_compute_mag_jacobian(ekf, &H);
    
    // 2. 예측된 측정값 계산
    // 현재 자세 사원수 추출
    Quaternion q = ekf_get_attitude(ekf);
    
    // 지구 자기장 벡터를 현재 자세로 회전
    Vector3f mag_pred = quaternion_rotate_vector_inverse(q, ekf->earth_mag_ned);
    
    // 3. 측정 잔차 계산 (측정값 - 예측값)
    Matrix z = matrix_create(3, 1);
    Matrix z_pred = matrix_create(3, 1);
    
    // 측정값 (자력계)
    matrix_set(&z, 0, 0, mag.x);
    matrix_set(&z, 1, 0, mag.y);
    matrix_set(&z, 2, 0, mag.z);
    
    // 예측값
    matrix_set(&z_pred, 0, 0, mag_pred.x);
    matrix_set(&z_pred, 1, 0, mag_pred.y);
    matrix_set(&z_pred, 2, 0, mag_pred.z);
    
    // 잔차
    Matrix y;
    matrix_subtract(&z, &z_pred, &y);
    
    // 4. 칼만 게인 계산
    Matrix K;
    if (!ekf_compute_kalman_gain(ekf, &H, &ekf->R_mag, &K)) {
        return false;
    }
    
    // 5. 상태 및 공분산 갱신
    return ekf_update_state_covariance(ekf, &K, &y, &H);
}
