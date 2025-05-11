/**
 * @file matrix.h
 * @brief 행렬 연산을 위한 라이브러리
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 최대 행렬 크기 정의
 * 
 * EKF 구현에 필요한 최대 크기 (16x16 행렬)
 */
#define MATRIX_MAX_SIZE 16

/**
 * @brief 행렬 구조체
 */
typedef struct {
    float data[MATRIX_MAX_SIZE][MATRIX_MAX_SIZE]; /**< 행렬 데이터 */
    uint8_t rows;                                 /**< 행 수 */
    uint8_t cols;                                 /**< 열 수 */
} Matrix;

/**
 * @brief 행렬 초기화
 * 
 * @param rows 행 수
 * @param cols 열 수
 * @return Matrix 초기화된 행렬 (모든 요소는 0)
 */
Matrix matrix_create(uint8_t rows, uint8_t cols);

/**
 * @brief 단위 행렬 생성
 * 
 * @param size 행렬 크기 (정방행렬)
 * @return Matrix 단위 행렬
 */
Matrix matrix_identity(uint8_t size);

/**
 * @brief 행렬 요소 설정
 * 
 * @param m 행렬
 * @param row 행 인덱스
 * @param col 열 인덱스
 * @param value 설정할 값
 * @return bool 성공 여부
 */
bool matrix_set(Matrix *m, uint8_t row, uint8_t col, float value);

/**
 * @brief 행렬 요소 가져오기
 * 
 * @param m 행렬
 * @param row 행 인덱스
 * @param col 열 인덱스
 * @param value 값을 저장할 포인터
 * @return bool 성공 여부
 */
bool matrix_get(const Matrix *m, uint8_t row, uint8_t col, float *value);

/**
 * @brief 행렬 덧셈
 * 
 * @param a 첫 번째 행렬
 * @param b 두 번째 행렬
 * @param result 결과 행렬
 * @return bool 성공 여부
 */
bool matrix_add(const Matrix *a, const Matrix *b, Matrix *result);

/**
 * @brief 행렬 뺄셈
 * 
 * @param a 첫 번째 행렬
 * @param b 두 번째 행렬
 * @param result 결과 행렬
 * @return bool 성공 여부
 */
bool matrix_subtract(const Matrix *a, const Matrix *b, Matrix *result);

/**
 * @brief 행렬 곱셈
 * 
 * @param a 첫 번째 행렬
 * @param b 두 번째 행렬
 * @param result 결과 행렬
 * @return bool 성공 여부
 */
bool matrix_multiply(const Matrix *a, const Matrix *b, Matrix *result);

/**
 * @brief 행렬 스칼라 곱
 * 
 * @param m 행렬
 * @param scalar 스칼라 값
 * @param result 결과 행렬
 * @return bool 성공 여부
 */
bool matrix_scale(const Matrix *m, float scalar, Matrix *result);

/**
 * @brief 행렬 전치
 * 
 * @param m 행렬
 * @param result 결과 행렬
 * @return bool 성공 여부
 */
bool matrix_transpose(const Matrix *m, Matrix *result);

/**
 * @brief 행렬 역행렬 계산
 * 
 * @param m 행렬
 * @param result 결과 행렬
 * @return bool 성공 여부 (역행렬이 존재하지 않으면 false)
 */
bool matrix_inverse(const Matrix *m, Matrix *result);

/**
 * @brief 행렬 복사
 * 
 * @param src 원본 행렬
 * @param dst 대상 행렬
 * @return bool 성공 여부
 */
bool matrix_copy(const Matrix *src, Matrix *dst);

/**
 * @brief 행렬에 벡터 추가 (행 벡터로 추가)
 * 
 * @param m 행렬
 * @param row 행 인덱스
 * @param vec 벡터 배열
 * @param vec_size 벡터 크기
 * @return bool 성공 여부
 */
bool matrix_set_row(Matrix *m, uint8_t row, const float *vec, uint8_t vec_size);

/**
 * @brief 행렬에 벡터 추가 (열 벡터로 추가)
 * 
 * @param m 행렬
 * @param col 열 인덱스
 * @param vec 벡터 배열
 * @param vec_size 벡터 크기
 * @return bool 성공 여부
 */
bool matrix_set_column(Matrix *m, uint8_t col, const float *vec, uint8_t vec_size);

/**
 * @brief 행렬 행 가져오기
 * 
 * @param m 행렬
 * @param row 행 인덱스
 * @param vec 벡터를 저장할 배열
 * @param vec_size 벡터 크기
 * @return bool 성공 여부
 */
bool matrix_get_row(const Matrix *m, uint8_t row, float *vec, uint8_t vec_size);

/**
 * @brief 행렬 열 가져오기
 * 
 * @param m 행렬
 * @param col 열 인덱스
 * @param vec 벡터를 저장할 배열
 * @param vec_size 벡터 크기
 * @return bool 성공 여부
 */
bool matrix_get_column(const Matrix *m, uint8_t col, float *vec, uint8_t vec_size);

/**
 * @brief 행렬 초기화 (모든 요소를 0으로 설정)
 * 
 * @param m 행렬
 * @return bool 성공 여부
 */
bool matrix_zero(Matrix *m);

/**
 * @brief 행렬 초기화 (대각 요소를 지정된 값으로 설정)
 * 
 * @param m 행렬
 * @param value 대각 요소 값
 * @return bool 성공 여부
 */
bool matrix_diagonal(Matrix *m, float value);

/**
 * @brief 행렬 초기화 (대각 요소를 지정된 벡터로 설정)
 * 
 * @param m 행렬
 * @param values 대각 요소 값 배열
 * @param size 배열 크기
 * @return bool 성공 여부
 */
bool matrix_diagonal_vector(Matrix *m, const float *values, uint8_t size);

/**
 * @brief 행렬 출력 (디버깅용)
 * 
 * @param m 행렬
 * @param name 행렬 이름
 */
void matrix_print(const Matrix *m, const char *name);

#endif /* MATRIX_H */
