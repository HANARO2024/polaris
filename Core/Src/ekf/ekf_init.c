/**
 * @file ekf_init.c
 * @brief 확장 칼만 필터(Extended Kalman Filter) 초기화 및 기본 함수 구현
 */

#include "ekf/ekf.h"
#include "ekf/ekf_mag_calibration.h"
#include <string.h>
#include <math.h>

/**
 * @brief EKF 초기화
 */
bool ekf_init(EKF *ekf) {
    if (ekf == NULL) {
        return false;
    }
    
    // 상태 벡터 초기화 (16x1)
    ekf->x = matrix_create(EKF_STATE_DIM, 1);
    
    // 공분산 행렬 초기화 (16x16)
    ekf->P = matrix_create(EKF_STATE_DIM, EKF_STATE_DIM);
    matrix_diagonal(&ekf->P, 1.0f); // 단위 행렬로 초기화
    
    // 프로세스 노이즈 공분산 초기화 (16x16)
    ekf->Q = matrix_create(EKF_STATE_DIM, EKF_STATE_DIM);
    matrix_diagonal(&ekf->Q, 0.01f); // 기본값으로 초기화
    
    // GPS 측정 노이즈 공분산 초기화 (6x6)
    ekf->R_gps = matrix_create(6, 6);
    float gps_std[6] = {5.0f, 5.0f, 10.0f, 0.5f, 0.5f, 1.0f}; // 위치 5m, 5m, 10m, 속도 0.5m/s
    matrix_diagonal_vector(&ekf->R_gps, gps_std, 6);
    
    // 기압계 측정 노이즈 공분산 초기화 (1x1)
    ekf->R_baro = matrix_create(1, 1);
    matrix_set(&ekf->R_baro, 0, 0, 1.0f); // 1m 표준 편차
    
    // 자력계 측정 노이즈 공분산 초기화 (3x3)
    ekf->R_mag = matrix_create(3, 3);
    matrix_diagonal(&ekf->R_mag, 0.1f); // 0.1uT 표준 편차
    
    // 중력 가속도 설정
    ekf->gravity = 9.80665f; // m/s^2
    
    // 지구 자기장 벡터 초기화 (NED 좌표계)
    // 일단은 서울 지역(37.5°N, 127°E)의 지구 자기장 벡터를 사용함.
    // 추후 ekf_mag_calibration.c의 ekf_calibrate_magnetic_field 함수를 사용하여 현장 측정으로 초기화하는 기능 구현할 예정.
    ekf->earth_mag_ned = vector3f_create(0.29f, -0.05f, 0.42f); // 서울(37.5°N, 127°E) 기준 대략적인 값
    
    // 초기화 완료
    ekf->initialized = false;
    
    return true;
}

/**
 * @brief EKF 초기 상태 설정
 */
bool ekf_set_initial_state(EKF *ekf, Vector3f pos, Vector3f vel, Quaternion q) {
    if (ekf == NULL) {
        return false;
    }
    
    // 위치 설정
    matrix_set(&ekf->x, EKF_STATE_POS_X, 0, pos.x);
    matrix_set(&ekf->x, EKF_STATE_POS_Y, 0, pos.y);
    matrix_set(&ekf->x, EKF_STATE_POS_Z, 0, pos.z);
    
    // 속도 설정
    matrix_set(&ekf->x, EKF_STATE_VEL_X, 0, vel.x);
    matrix_set(&ekf->x, EKF_STATE_VEL_Y, 0, vel.y);
    matrix_set(&ekf->x, EKF_STATE_VEL_Z, 0, vel.z);
    
    // 자세 설정 (정규화된 사원수)
    Quaternion qn = quaternion_normalize(q);
    matrix_set(&ekf->x, EKF_STATE_QUAT_W, 0, qn.w);
    matrix_set(&ekf->x, EKF_STATE_QUAT_X, 0, qn.x);
    matrix_set(&ekf->x, EKF_STATE_QUAT_Y, 0, qn.y);
    matrix_set(&ekf->x, EKF_STATE_QUAT_Z, 0, qn.z);
    
    // 바이어스 초기화
    matrix_set(&ekf->x, EKF_STATE_GYRO_BIAS_X, 0, 0.0f);
    matrix_set(&ekf->x, EKF_STATE_GYRO_BIAS_Y, 0, 0.0f);
    matrix_set(&ekf->x, EKF_STATE_GYRO_BIAS_Z, 0, 0.0f);
    matrix_set(&ekf->x, EKF_STATE_ACC_BIAS_X, 0, 0.0f);
    matrix_set(&ekf->x, EKF_STATE_ACC_BIAS_Y, 0, 0.0f);
    matrix_set(&ekf->x, EKF_STATE_ACC_BIAS_Z, 0, 0.0f);
    
    // 공분산 행렬 초기화
    float p_diag[EKF_STATE_DIM] = {
        10.0f, 10.0f, 10.0f,     // 위치 불확실성 (m^2)
        1.0f, 1.0f, 1.0f,        // 속도 불확실성 (m/s)^2
        0.1f, 0.1f, 0.1f, 0.1f,  // 자세 불확실성
        0.01f, 0.01f, 0.01f,     // 자이로 바이어스 불확실성 (rad/s)^2
        0.1f, 0.1f, 0.1f         // 가속도 바이어스 불확실성 (m/s^2)^2
    };
    matrix_diagonal_vector(&ekf->P, p_diag, EKF_STATE_DIM);
    
    ekf->initialized = true;
    
    return true;
}

/**
 * @brief EKF 프로세스 노이즈 설정
 */
bool ekf_set_process_noise(EKF *ekf, float pos_std, float vel_std, float att_std, 
                          float gyro_bias_std, float acc_bias_std) {
    if (ekf == NULL) {
        return false;
    }
    
    // 프로세스 노이즈 공분산 초기화
    matrix_zero(&ekf->Q);
    
    // 위치 프로세스 노이즈
    matrix_set(&ekf->Q, EKF_STATE_POS_X, EKF_STATE_POS_X, pos_std * pos_std);
    matrix_set(&ekf->Q, EKF_STATE_POS_Y, EKF_STATE_POS_Y, pos_std * pos_std);
    matrix_set(&ekf->Q, EKF_STATE_POS_Z, EKF_STATE_POS_Z, pos_std * pos_std);
    
    // 속도 프로세스 노이즈
    matrix_set(&ekf->Q, EKF_STATE_VEL_X, EKF_STATE_VEL_X, vel_std * vel_std);
    matrix_set(&ekf->Q, EKF_STATE_VEL_Y, EKF_STATE_VEL_Y, vel_std * vel_std);
    matrix_set(&ekf->Q, EKF_STATE_VEL_Z, EKF_STATE_VEL_Z, vel_std * vel_std);
    
    // 자세 프로세스 노이즈
    matrix_set(&ekf->Q, EKF_STATE_QUAT_W, EKF_STATE_QUAT_W, att_std * att_std);
    matrix_set(&ekf->Q, EKF_STATE_QUAT_X, EKF_STATE_QUAT_X, att_std * att_std);
    matrix_set(&ekf->Q, EKF_STATE_QUAT_Y, EKF_STATE_QUAT_Y, att_std * att_std);
    matrix_set(&ekf->Q, EKF_STATE_QUAT_Z, EKF_STATE_QUAT_Z, att_std * att_std);
    
    // 자이로 바이어스 프로세스 노이즈
    matrix_set(&ekf->Q, EKF_STATE_GYRO_BIAS_X, EKF_STATE_GYRO_BIAS_X, gyro_bias_std * gyro_bias_std);
    matrix_set(&ekf->Q, EKF_STATE_GYRO_BIAS_Y, EKF_STATE_GYRO_BIAS_Y, gyro_bias_std * gyro_bias_std);
    matrix_set(&ekf->Q, EKF_STATE_GYRO_BIAS_Z, EKF_STATE_GYRO_BIAS_Z, gyro_bias_std * gyro_bias_std);
    
    // 가속도 바이어스 프로세스 노이즈
    matrix_set(&ekf->Q, EKF_STATE_ACC_BIAS_X, EKF_STATE_ACC_BIAS_X, acc_bias_std * acc_bias_std);
    matrix_set(&ekf->Q, EKF_STATE_ACC_BIAS_Y, EKF_STATE_ACC_BIAS_Y, acc_bias_std * acc_bias_std);
    matrix_set(&ekf->Q, EKF_STATE_ACC_BIAS_Z, EKF_STATE_ACC_BIAS_Z, acc_bias_std * acc_bias_std);
    
    return true;
}

/**
 * @brief EKF GPS 측정 노이즈 설정
 */
bool ekf_set_gps_noise(EKF *ekf, float pos_std, float vel_std) {
    if (ekf == NULL) {
        return false;
    }
    
    // GPS 측정 노이즈 공분산 초기화
    matrix_zero(&ekf->R_gps);
    
    // 위치 측정 노이즈
    matrix_set(&ekf->R_gps, 0, 0, pos_std * pos_std);
    matrix_set(&ekf->R_gps, 1, 1, pos_std * pos_std);
    matrix_set(&ekf->R_gps, 2, 2, pos_std * pos_std);
    
    // 속도 측정 노이즈
    matrix_set(&ekf->R_gps, 3, 3, vel_std * vel_std);
    matrix_set(&ekf->R_gps, 4, 4, vel_std * vel_std);
    matrix_set(&ekf->R_gps, 5, 5, vel_std * vel_std);
    
    return true;
}

/**
 * @brief EKF 기압계 측정 노이즈 설정
 */
bool ekf_set_baro_noise(EKF *ekf, float baro_std) {
    if (ekf == NULL) {
        return false;
    }
    
    // 기압계 측정 노이즈 공분산 설정
    matrix_set(&ekf->R_baro, 0, 0, baro_std * baro_std);
    
    return true;
}

/**
 * @brief EKF 자력계 측정 노이즈 설정
 */
bool ekf_set_mag_noise(EKF *ekf, float mag_std) {
    if (ekf == NULL) {
        return false;
    }
    
    // 자력계 측정 노이즈 공분산 초기화
    matrix_zero(&ekf->R_mag);
    
    // 자력계 측정 노이즈
    matrix_set(&ekf->R_mag, 0, 0, mag_std * mag_std);
    matrix_set(&ekf->R_mag, 1, 1, mag_std * mag_std);
    matrix_set(&ekf->R_mag, 2, 2, mag_std * mag_std);
    
    return true;
}

/**
 * @brief EKF 지구 자기장 벡터 설정
 */
bool ekf_set_earth_magnetic_field(EKF *ekf, Vector3f mag_ned) {
    if (ekf == NULL) {
        return false;
    }
    
    // 지구 자기장 벡터 설정
    ekf->earth_mag_ned = mag_ned;
    
    return true;
}

/**
 * @brief EKF 상태에서 위치 추출
 */
Vector3f ekf_get_position(const EKF *ekf) {
    Vector3f pos = vector3f_zero();
    
    if (ekf == NULL || !ekf->initialized) {
        return pos;
    }
    
    float x, y, z;
    matrix_get(&ekf->x, EKF_STATE_POS_X, 0, &x);
    matrix_get(&ekf->x, EKF_STATE_POS_Y, 0, &y);
    matrix_get(&ekf->x, EKF_STATE_POS_Z, 0, &z);
    
    pos.x = x;
    pos.y = y;
    pos.z = z;
    
    return pos;
}

/**
 * @brief EKF 상태에서 속도 추출
 */
Vector3f ekf_get_velocity(const EKF *ekf) {
    Vector3f vel = vector3f_zero();
    
    if (ekf == NULL || !ekf->initialized) {
        return vel;
    }
    
    float vx, vy, vz;
    matrix_get(&ekf->x, EKF_STATE_VEL_X, 0, &vx);
    matrix_get(&ekf->x, EKF_STATE_VEL_Y, 0, &vy);
    matrix_get(&ekf->x, EKF_STATE_VEL_Z, 0, &vz);
    
    vel.x = vx;
    vel.y = vy;
    vel.z = vz;
    
    return vel;
}

/**
 * @brief EKF 상태에서 자세 추출
 */
Quaternion ekf_get_attitude(const EKF *ekf) {
    Quaternion q = quaternion_identity();
    
    if (ekf == NULL || !ekf->initialized) {
        return q;
    }
    
    float w, x, y, z;
    matrix_get(&ekf->x, EKF_STATE_QUAT_W, 0, &w);
    matrix_get(&ekf->x, EKF_STATE_QUAT_X, 0, &x);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Y, 0, &y);
    matrix_get(&ekf->x, EKF_STATE_QUAT_Z, 0, &z);
    
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    
    // 정규화 (수치 오차 방지)
    return quaternion_normalize(q);
}

/**
 * @brief EKF 상태에서 오일러 각 추출
 */
bool ekf_get_euler(const EKF *ekf, float *roll, float *pitch, float *yaw) {
    if (ekf == NULL || !ekf->initialized || roll == NULL || pitch == NULL || yaw == NULL) {
        return false;
    }
    
    Quaternion q = ekf_get_attitude(ekf);
    quaternion_to_euler(q, roll, pitch, yaw);
    
    return true;
}

/**
 * @brief EKF 상태에서 자이로 바이어스 추출
 */
Vector3f ekf_get_gyro_bias(const EKF *ekf) {
    Vector3f bias = vector3f_zero();
    
    if (ekf == NULL || !ekf->initialized) {
        return bias;
    }
    
    float bx, by, bz;
    matrix_get(&ekf->x, EKF_STATE_GYRO_BIAS_X, 0, &bx);
    matrix_get(&ekf->x, EKF_STATE_GYRO_BIAS_Y, 0, &by);
    matrix_get(&ekf->x, EKF_STATE_GYRO_BIAS_Z, 0, &bz);
    
    bias.x = bx;
    bias.y = by;
    bias.z = bz;
    
    return bias;
}

/**
 * @brief EKF 상태에서 가속도 바이어스 추출
 */
Vector3f ekf_get_accel_bias(const EKF *ekf) {
    Vector3f bias = vector3f_zero();
    
    if (ekf == NULL || !ekf->initialized) {
        return bias;
    }
    
    float bx, by, bz;
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_X, 0, &bx);
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_Y, 0, &by);
    matrix_get(&ekf->x, EKF_STATE_ACC_BIAS_Z, 0, &bz);
    
    bias.x = bx;
    bias.y = by;
    bias.z = bz;
    
    return bias;
}

/**
 * @brief EKF 상태 리셋
 */
bool ekf_reset(EKF *ekf) {
    if (ekf == NULL) {
        return false;
    }
    
    // 상태 벡터 초기화
    matrix_zero(&ekf->x);
    
    // 사원수 부분은 단위 사원수로 초기화
    matrix_set(&ekf->x, EKF_STATE_QUAT_W, 0, 1.0f);
    
    // 공분산 행렬 초기화
    float p_diag[EKF_STATE_DIM] = {
        100.0f, 100.0f, 100.0f,  // 위치 불확실성 (m^2)
        10.0f, 10.0f, 10.0f,     // 속도 불확실성 (m/s)^2
        1.0f, 1.0f, 1.0f, 1.0f,  // 자세 불확실성
        0.01f, 0.01f, 0.01f,     // 자이로 바이어스 불확실성 (rad/s)^2
        0.1f, 0.1f, 0.1f         // 가속도 바이어스 불확실성 (m/s^2)^2
    };
    matrix_diagonal_vector(&ekf->P, p_diag, EKF_STATE_DIM);
    
    ekf->initialized = false;
    
    return true;
}

/**
 * @brief 센서 샘플을 수집하여 자기장 벡터 초기화
 */
bool ekf_initialize_magnetic_field(EKF *ekf, Vector3f *mag_samples, Vector3f *accel_samples, int sample_count) {
    if (ekf == NULL || mag_samples == NULL || accel_samples == NULL || sample_count <= 0) {
        return ekf_initialize_default_magnetic_field(ekf);
    }
    
    // 자력계와 가속도계 데이터를 사용하여 NED 좌표계의 자기장 벡터 계산
    Vector3f mag_ned = ekf_calibrate_magnetic_field(mag_samples, accel_samples, sample_count);
    
    // EKF에 설정
    ekf->earth_mag_ned = mag_ned;
    
    return true;
}

/**
 * @brief 기본 자기장 값으로 초기화 (현장 측정이 불가능한 경우)
 */
bool ekf_initialize_default_magnetic_field(EKF *ekf) {
    if (ekf == NULL) {
        return false;
    }
    
    // 서울(37.5°N, 127°E) 기준 대략적인 값
    ekf->earth_mag_ned = vector3f_create(0.29f, -0.05f, 0.42f);
    
    return true;
}
