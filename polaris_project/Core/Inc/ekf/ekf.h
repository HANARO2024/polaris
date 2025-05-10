/**
 * @file ekf.h
 * @brief 확장 칼만 필터(Extended Kalman Filter) 구현
 */

#ifndef EKF_H
#define EKF_H

#include "math/quaternion.h"
#include "math/matrix.h"

/**
 * @brief EKF 상태 벡터 크기
 * 
 * 상태 벡터 구성:
 * - 위치 (x, y, z): 3
 * - 속도 (vx, vy, vz): 3
 * - 자세 (q0, q1, q2, q3): 4
 * - 각속도 바이어스 (bgx, bgy, bgz): 3
 * - 가속도 바이어스 (bax, bay, baz): 3
 * 총 16개 상태 변수
 */
#define EKF_STATE_DIM 16

/**
 * @brief EKF 상태 인덱스 정의
 */
typedef enum {
    EKF_STATE_POS_X = 0,   /**< 위치 X */
    EKF_STATE_POS_Y = 1,   /**< 위치 Y */
    EKF_STATE_POS_Z = 2,   /**< 위치 Z */
    EKF_STATE_VEL_X = 3,   /**< 속도 X */
    EKF_STATE_VEL_Y = 4,   /**< 속도 Y */
    EKF_STATE_VEL_Z = 5,   /**< 속도 Z */
    EKF_STATE_QUAT_W = 6,  /**< 사원수 W */
    EKF_STATE_QUAT_X = 7,  /**< 사원수 X */
    EKF_STATE_QUAT_Y = 8,  /**< 사원수 Y */
    EKF_STATE_QUAT_Z = 9,  /**< 사원수 Z */
    EKF_STATE_GYRO_BIAS_X = 10, /**< 자이로 바이어스 X */
    EKF_STATE_GYRO_BIAS_Y = 11, /**< 자이로 바이어스 Y */
    EKF_STATE_GYRO_BIAS_Z = 12, /**< 자이로 바이어스 Z */
    EKF_STATE_ACC_BIAS_X = 13,  /**< 가속도 바이어스 X */
    EKF_STATE_ACC_BIAS_Y = 14,  /**< 가속도 바이어스 Y */
    EKF_STATE_ACC_BIAS_Z = 15   /**< 가속도 바이어스 Z */
} EKF_StateIndex;

/**
 * @brief EKF 구조체
 */
typedef struct {
    Matrix x;       /**< 상태 벡터 (16x1) */
    Matrix P;       /**< 공분산 행렬 (16x16) */
    Matrix Q;       /**< 프로세스 노이즈 공분산 (16x16) */
    Matrix R_gps;   /**< GPS 측정 노이즈 공분산 (3x3 또는 6x6) */
    Matrix R_baro;  /**< 기압계 측정 노이즈 공분산 (1x1) */
    Matrix R_mag;   /**< 자력계 측정 노이즈 공분산 (3x3) */
    
    float gravity;  /**< 중력 가속도 (m/s^2) */
    
    Vector3f earth_mag_ned; /**< 지구 자기장 벡터 (NED 좌표계) */
    
    bool initialized; /**< 초기화 여부 */
} EKF;

/**
 * @brief EKF 초기화
 * 
 * @param ekf EKF 구조체 포인터
 * @return bool 초기화 성공 여부
 */
bool ekf_init(EKF *ekf);

/**
 * @brief EKF 초기 상태 설정
 * 
 * @param ekf EKF 구조체 포인터
 * @param pos 초기 위치 (NED 좌표계)
 * @param vel 초기 속도 (NED 좌표계)
 * @param q 초기 자세 사원수
 * @return bool 설정 성공 여부
 */
bool ekf_set_initial_state(EKF *ekf, Vector3f pos, Vector3f vel, Quaternion q);

/**
 * @brief EKF 프로세스 노이즈 설정
 * 
 * @param ekf EKF 구조체 포인터
 * @param pos_std 위치 표준 편차 (m)
 * @param vel_std 속도 표준 편차 (m/s)
 * @param att_std 자세 표준 편차 (rad)
 * @param gyro_bias_std 자이로 바이어스 표준 편차 (rad/s)
 * @param acc_bias_std 가속도 바이어스 표준 편차 (m/s^2)
 * @return bool 설정 성공 여부
 */
bool ekf_set_process_noise(EKF *ekf, float pos_std, float vel_std, float att_std, 
                          float gyro_bias_std, float acc_bias_std);

/**
 * @brief EKF GPS 측정 노이즈 설정
 * 
 * @param ekf EKF 구조체 포인터
 * @param pos_std 위치 표준 편차 (m)
 * @param vel_std 속도 표준 편차 (m/s), 속도 측정이 없는 경우 0
 * @return bool 설정 성공 여부
 */
bool ekf_set_gps_noise(EKF *ekf, float pos_std, float vel_std);

/**
 * @brief EKF 기압계 측정 노이즈 설정
 * 
 * @param ekf EKF 구조체 포인터
 * @param baro_std 고도 표준 편차 (m)
 * @return bool 설정 성공 여부
 */
bool ekf_set_baro_noise(EKF *ekf, float baro_std);

/**
 * @brief EKF 자력계 측정 노이즈 설정
 * 
 * @param ekf EKF 구조체 포인터
 * @param mag_std 자기장 표준 편차 (uT)
 * @return bool 설정 성공 여부
 */
bool ekf_set_mag_noise(EKF *ekf, float mag_std);

/**
 * @brief EKF 지구 자기장 벡터 설정
 * 
 * @param ekf EKF 구조체 포인터
 * @param mag_ned 지구 자기장 벡터 (NED 좌표계)
 * @return bool 설정 성공 여부
 */
bool ekf_set_earth_magnetic_field(EKF *ekf, Vector3f mag_ned);

/**
 * @brief EKF 예측 단계 (IMU 데이터 기반)
 * 
 * @param ekf EKF 구조체 포인터
 * @param gyro 자이로 측정값 (rad/s)
 * @param accel 가속도 측정값 (m/s^2)
 * @param dt 시간 간격 (초)
 * @return bool 예측 성공 여부
 */
bool ekf_predict(EKF *ekf, Vector3f gyro, Vector3f accel, float dt);

/**
 * @brief EKF GPS 측정 갱신
 * 
 * @param ekf EKF 구조체 포인터
 * @param gps_pos GPS 위치 측정값 (NED 좌표계, m)
 * @param use_vel GPS 속도 사용 여부
 * @param gps_vel GPS 속도 측정값 (NED 좌표계, m/s)
 * @return bool 갱신 성공 여부
 */
bool ekf_update_gps(EKF *ekf, Vector3f gps_pos, bool use_vel, Vector3f gps_vel);

/**
 * @brief EKF 기압계 측정 갱신
 * 
 * @param ekf EKF 구조체 포인터
 * @param baro_alt 기압계 고도 측정값 (m, 음수는 아래 방향)
 * @return bool 갱신 성공 여부
 */
bool ekf_update_baro(EKF *ekf, float baro_alt);

/**
 * @brief EKF 자력계 측정 갱신
 * 
 * @param ekf EKF 구조체 포인터
 * @param mag 자력계 측정값 (몸체 좌표계, uT)
 * @return bool 갱신 성공 여부
 */
bool ekf_update_mag(EKF *ekf, Vector3f mag);

/**
 * @brief EKF 상태에서 위치 추출
 * 
 * @param ekf EKF 구조체 포인터
 * @return Vector3f 위치 벡터 (NED 좌표계, m)
 */
Vector3f ekf_get_position(const EKF *ekf);

/**
 * @brief EKF 상태에서 속도 추출
 * 
 * @param ekf EKF 구조체 포인터
 * @return Vector3f 속도 벡터 (NED 좌표계, m/s)
 */
Vector3f ekf_get_velocity(const EKF *ekf);

/**
 * @brief EKF 상태에서 자세 추출
 * 
 * @param ekf EKF 구조체 포인터
 * @return Quaternion 자세 사원수
 */
Quaternion ekf_get_attitude(const EKF *ekf);

/**
 * @brief EKF 상태에서 오일러 각 추출
 * 
 * @param ekf EKF 구조체 포인터
 * @param roll 롤 각도를 저장할 포인터 (rad)
 * @param pitch 피치 각도를 저장할 포인터 (rad)
 * @param yaw 요 각도를 저장할 포인터 (rad)
 * @return bool 추출 성공 여부
 */
bool ekf_get_euler(const EKF *ekf, float *roll, float *pitch, float *yaw);

/**
 * @brief EKF 상태에서 자이로 바이어스 추출
 * 
 * @param ekf EKF 구조체 포인터
 * @return Vector3f 자이로 바이어스 (rad/s)
 */
Vector3f ekf_get_gyro_bias(const EKF *ekf);

/**
 * @brief EKF 상태에서 가속도 바이어스 추출
 * 
 * @param ekf EKF 구조체 포인터
 * @return Vector3f 가속도 바이어스 (m/s^2)
 */
Vector3f ekf_get_accel_bias(const EKF *ekf);

/**
 * @brief EKF 상태 리셋
 * 
 * @param ekf EKF 구조체 포인터
 * @return bool 리셋 성공 여부
 */
bool ekf_reset(EKF *ekf);

#endif /* EKF_H */
