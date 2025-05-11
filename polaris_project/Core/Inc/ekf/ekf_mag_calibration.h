/**
 * @file ekf_mag_calibration.h
 * @brief 자력계 현장 측정 및 보정을 위한 함수 선언
 */

#ifndef EKF_MAG_CALIBRATION_H
#define EKF_MAG_CALIBRATION_H

#include "math/vector3f.h"

/**
 * @brief 자력계와 가속도계 샘플을 수집하여 NED 좌표계로 변환된 자기장 벡터 계산
 * 
 * @param mag_samples 자력계 샘플 배열
 * @param accel_samples 가속도계 샘플 배열
 * @param sample_count 샘플 개수
 * @return Vector3f NED 좌표계의 자기장 단위 벡터
 */
Vector3f ekf_calibrate_magnetic_field(Vector3f *mag_samples, Vector3f *accel_samples, int sample_count);

/**
 * @brief 가속도계 데이터를 사용하여 NED 좌표계 변환 행렬 계산
 * 
 * @param accel 가속도계 벡터 (중력 방향은 -Z에 대응)
 * @return Matrix3f 본체 좌표계에서 NED 좌표계로의 변환 행렬
 */
void ekf_compute_ned_transform(Vector3f accel, float *dcm);

/**
 * @brief 본체 좌표계의 벡터를 NED 좌표계로 변환
 * 
 * @param vec_body 본체 좌표계 벡터
 * @param dcm 본체->NED 변환 행렬 (3x3)
 * @return Vector3f NED 좌표계 벡터
 */
Vector3f ekf_convert_to_ned(Vector3f vec_body, const float *dcm);

#endif /* EKF_MAG_CALIBRATION_H */
