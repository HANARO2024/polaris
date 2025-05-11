/**
 * @file ekf_mag_calibration.c
 * @brief 자력계 현장 측정 및 보정을 위한 함수 구현
 */

#include "ekf/ekf_mag_calibration.h"
#include <math.h>
#include <string.h>

/**
 * @brief 자력계와 가속도계 샘플을 수집하여 NED 좌표계로 변환된 자기장 벡터 계산
 */
Vector3f ekf_calibrate_magnetic_field(Vector3f *mag_samples, Vector3f *accel_samples, int sample_count) {
    if (mag_samples == NULL || accel_samples == NULL || sample_count <= 0) {
        // 기본값 반환
        return vector3f_create(0.29f, -0.05f, 0.42f);
    }
    
    // 평균 자력계 및 가속도계 측정값 계산
    Vector3f avg_mag = vector3f_zero();
    Vector3f avg_accel = vector3f_zero();
    
    for (int i = 0; i < sample_count; i++) {
        avg_mag = vector3f_add(avg_mag, mag_samples[i]);
        avg_accel = vector3f_add(avg_accel, accel_samples[i]);
    }
    
    avg_mag = vector3f_scale(avg_mag, 1.0f / (float)sample_count);
    avg_accel = vector3f_scale(avg_accel, 1.0f / (float)sample_count);
    
    // 가속도계로부터 본체->NED 좌표계 변환 행렬 계산
    float dcm[9];
    ekf_compute_ned_transform(avg_accel, dcm);
    
    // 자력계 데이터를 NED 좌표계로 변환
    Vector3f mag_ned = ekf_convert_to_ned(avg_mag, dcm);
    
    // 정규화
    return vector3f_normalize(mag_ned);
}

/**
 * @brief 가속도계 데이터를 사용하여 NED 좌표계 변환 행렬 계산
 * 
 * 가속도계는 중력 방향(Down)을 제공하고,
 * East와 North는 이를 기준으로 계산됨
 */
void ekf_compute_ned_transform(Vector3f accel, float *dcm) {
    if (dcm == NULL) {
        return;
    }
    
    // 1. Down 축은 가속도계 방향의 반대 (중력 방향)
    Vector3f down = vector3f_scale(accel, -1.0f);
    down = vector3f_normalize(down);
    
    // 2. 임의의 벡터 선택 (여기서는 동쪽 방향을 [0,1,0]으로 가정)
    // 실제로는 자북 방향과 정확히 맞추려면 더 복잡한 계산이 필요하지만,
    // 우리는 단지 현장 자기장 벡터의 방향을 측정하기 위함이므로 이 방법으로 충분함
    Vector3f east_approx = vector3f_create(0.0f, 1.0f, 0.0f);
    
    // 3. North 축은 Down과 East의 외적
    Vector3f north = vector3f_cross(down, east_approx);
    north = vector3f_normalize(north);
    
    // 4. 실제 East 축은 North와 Down의 외적
    Vector3f east = vector3f_cross(north, down);
    east = vector3f_normalize(east);
    
    // 5. DCM 행렬 구성 (본체 좌표계 -> NED 좌표계)
    // 첫 번째 행: North 축
    dcm[0] = north.x;
    dcm[1] = north.y;
    dcm[2] = north.z;
    
    // 두 번째 행: East 축
    dcm[3] = east.x;
    dcm[4] = east.y;
    dcm[5] = east.z;
    
    // 세 번째 행: Down 축
    dcm[6] = down.x;
    dcm[7] = down.y;
    dcm[8] = down.z;
}

/**
 * @brief 본체 좌표계의 벡터를 NED 좌표계로 변환
 */
Vector3f ekf_convert_to_ned(Vector3f vec_body, const float *dcm) {
    if (dcm == NULL) {
        return vec_body;
    }
    
    // 행렬-벡터 곱셈 수행
    Vector3f vec_ned;
    
    // North 성분
    vec_ned.x = dcm[0] * vec_body.x + dcm[1] * vec_body.y + dcm[2] * vec_body.z;
    
    // East 성분
    vec_ned.y = dcm[3] * vec_body.x + dcm[4] * vec_body.y + dcm[5] * vec_body.z;
    
    // Down 성분
    vec_ned.z = dcm[6] * vec_body.x + dcm[7] * vec_body.y + dcm[8] * vec_body.z;
    
    return vec_ned;
}
