/**
 * @file matrix.c
 * @brief 행렬 연산을 위한 라이브러리 구현
 */

#include "math/matrix.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/**
 * @brief 행렬 초기화
 */
Matrix matrix_create(uint8_t rows, uint8_t cols) {
    Matrix m;
    
    // 범위 검사
    if (rows > MATRIX_MAX_SIZE) rows = MATRIX_MAX_SIZE;
    if (cols > MATRIX_MAX_SIZE) cols = MATRIX_MAX_SIZE;
    
    m.rows = rows;
    m.cols = cols;
    
    // 모든 요소를 0으로 초기화
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            m.data[i][j] = 0.0f;
        }
    }
    
    return m;
}

/**
 * @brief 단위 행렬 생성
 */
Matrix matrix_identity(uint8_t size) {
    Matrix m = matrix_create(size, size);
    
    // 대각 요소를 1로 설정
    for (uint8_t i = 0; i < size; i++) {
        m.data[i][i] = 1.0f;
    }
    
    return m;
}

/**
 * @brief 행렬 요소 설정
 */
bool matrix_set(Matrix *m, uint8_t row, uint8_t col, float value) {
    if (row >= m->rows || col >= m->cols) {
        return false;
    }
    
    m->data[row][col] = value;
    return true;
}

/**
 * @brief 행렬 요소 가져오기
 */
bool matrix_get(const Matrix *m, uint8_t row, uint8_t col, float *value) {
    if (row >= m->rows || col >= m->cols || value == NULL) {
        return false;
    }
    
    *value = m->data[row][col];
    return true;
}

/**
 * @brief 행렬 덧셈
 */
bool matrix_add(const Matrix *a, const Matrix *b, Matrix *result) {
    if (a->rows != b->rows || a->cols != b->cols) {
        return false;
    }
    
    *result = matrix_create(a->rows, a->cols);
    
    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
    
    return true;
}

/**
 * @brief 행렬 뺄셈
 */
bool matrix_subtract(const Matrix *a, const Matrix *b, Matrix *result) {
    if (a->rows != b->rows || a->cols != b->cols) {
        return false;
    }
    
    *result = matrix_create(a->rows, a->cols);
    
    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] - b->data[i][j];
        }
    }
    
    return true;
}

/**
 * @brief 행렬 곱셈
 */
bool matrix_multiply(const Matrix *a, const Matrix *b, Matrix *result) {
    if (a->cols != b->rows) {
        return false;
    }
    
    *result = matrix_create(a->rows, b->cols);
    
    for (uint8_t i = 0; i < a->rows; i++) {
        for (uint8_t j = 0; j < b->cols; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < a->cols; k++) {
                sum += a->data[i][k] * b->data[k][j];
            }
            result->data[i][j] = sum;
        }
    }
    
    return true;
}

/**
 * @brief 행렬 스칼라 곱
 */
bool matrix_scale(const Matrix *m, float scalar, Matrix *result) {
    *result = matrix_create(m->rows, m->cols);
    
    for (uint8_t i = 0; i < m->rows; i++) {
        for (uint8_t j = 0; j < m->cols; j++) {
            result->data[i][j] = m->data[i][j] * scalar;
        }
    }
    
    return true;
}

/**
 * @brief 행렬 전치
 */
bool matrix_transpose(const Matrix *m, Matrix *result) {
    *result = matrix_create(m->cols, m->rows);
    
    for (uint8_t i = 0; i < m->rows; i++) {
        for (uint8_t j = 0; j < m->cols; j++) {
            result->data[j][i] = m->data[i][j];
        }
    }
    
    return true;
}

/**
 * @brief 행렬 역행렬 계산 (가우스-조던 소거법 사용)
 */
bool matrix_inverse(const Matrix *m, Matrix *result) {
    // 정방행렬 확인
    if (m->rows != m->cols) {
        return false;
    }
    
    uint8_t n = m->rows;
    
    // 증강 행렬 생성 [A|I]
    Matrix augmented = matrix_create(n, 2 * n);
    
    // 원본 행렬 복사
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            augmented.data[i][j] = m->data[i][j];
        }
    }
    
    // 단위 행렬 추가
    for (uint8_t i = 0; i < n; i++) {
        augmented.data[i][i + n] = 1.0f;
    }
    
    // 가우스-조던 소거법
    for (uint8_t i = 0; i < n; i++) {
        // 피벗 찾기
        uint8_t pivot = i;
        float max_val = fabsf(augmented.data[i][i]);
        
        for (uint8_t j = i + 1; j < n; j++) {
            if (fabsf(augmented.data[j][i]) > max_val) {
                max_val = fabsf(augmented.data[j][i]);
                pivot = j;
            }
        }
        
        // 특이 행렬 확인
        if (max_val < 1e-6f) {
            return false; // 역행렬 없음
        }
        
        // 행 교환
        if (pivot != i) {
            for (uint8_t j = 0; j < 2 * n; j++) {
                float temp = augmented.data[i][j];
                augmented.data[i][j] = augmented.data[pivot][j];
                augmented.data[pivot][j] = temp;
            }
        }
        
        // 피벗 행 정규화
        float pivot_val = augmented.data[i][i];
        for (uint8_t j = 0; j < 2 * n; j++) {
            augmented.data[i][j] /= pivot_val;
        }
        
        // 다른 행 소거
        for (uint8_t j = 0; j < n; j++) {
            if (j != i) {
                float factor = augmented.data[j][i];
                for (uint8_t k = 0; k < 2 * n; k++) {
                    augmented.data[j][k] -= factor * augmented.data[i][k];
                }
            }
        }
    }
    
    // 결과 행렬 추출
    *result = matrix_create(n, n);
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            result->data[i][j] = augmented.data[i][j + n];
        }
    }
    
    return true;
}

/**
 * @brief 행렬 복사
 */
bool matrix_copy(const Matrix *src, Matrix *dst) {
    dst->rows = src->rows;
    dst->cols = src->cols;
    
    for (uint8_t i = 0; i < src->rows; i++) {
        for (uint8_t j = 0; j < src->cols; j++) {
            dst->data[i][j] = src->data[i][j];
        }
    }
    
    return true;
}

/**
 * @brief 행렬에 벡터 추가 (행 벡터로 추가)
 */
bool matrix_set_row(Matrix *m, uint8_t row, const float *vec, uint8_t vec_size) {
    if (row >= m->rows || vec == NULL || vec_size > m->cols) {
        return false;
    }
    
    for (uint8_t j = 0; j < vec_size; j++) {
        m->data[row][j] = vec[j];
    }
    
    return true;
}

/**
 * @brief 행렬에 벡터 추가 (열 벡터로 추가)
 */
bool matrix_set_column(Matrix *m, uint8_t col, const float *vec, uint8_t vec_size) {
    if (col >= m->cols || vec == NULL || vec_size > m->rows) {
        return false;
    }
    
    for (uint8_t i = 0; i < vec_size; i++) {
        m->data[i][col] = vec[i];
    }
    
    return true;
}

/**
 * @brief 행렬 행 가져오기
 */
bool matrix_get_row(const Matrix *m, uint8_t row, float *vec, uint8_t vec_size) {
    if (row >= m->rows || vec == NULL || vec_size > m->cols) {
        return false;
    }
    
    for (uint8_t j = 0; j < vec_size; j++) {
        vec[j] = m->data[row][j];
    }
    
    return true;
}

/**
 * @brief 행렬 열 가져오기
 */
bool matrix_get_column(const Matrix *m, uint8_t col, float *vec, uint8_t vec_size) {
    if (col >= m->cols || vec == NULL || vec_size > m->rows) {
        return false;
    }
    
    for (uint8_t i = 0; i < vec_size; i++) {
        vec[i] = m->data[i][col];
    }
    
    return true;
}

/**
 * @brief 행렬 초기화 (모든 요소를 0으로 설정)
 */
bool matrix_zero(Matrix *m) {
    for (uint8_t i = 0; i < m->rows; i++) {
        for (uint8_t j = 0; j < m->cols; j++) {
            m->data[i][j] = 0.0f;
        }
    }
    
    return true;
}

/**
 * @brief 행렬 초기화 (대각 요소를 지정된 값으로 설정)
 */
bool matrix_diagonal(Matrix *m, float value) {
    matrix_zero(m);
    
    uint8_t min_dim = (m->rows < m->cols) ? m->rows : m->cols;
    
    for (uint8_t i = 0; i < min_dim; i++) {
        m->data[i][i] = value;
    }
    
    return true;
}

/**
 * @brief 행렬 초기화 (대각 요소를 지정된 벡터로 설정)
 */
bool matrix_diagonal_vector(Matrix *m, const float *values, uint8_t size) {
    if (values == NULL) {
        return false;
    }
    
    matrix_zero(m);
    
    uint8_t min_dim = (m->rows < m->cols) ? m->rows : m->cols;
    if (size < min_dim) min_dim = size;
    
    for (uint8_t i = 0; i < min_dim; i++) {
        m->data[i][i] = values[i];
    }
    
    return true;
}

/**
 * @brief 행렬 출력 (디버깅용)
 */
void matrix_print(const Matrix *m, const char *name) {
    printf("Matrix %s (%dx%d):\n", name, m->rows, m->cols);
    
    for (uint8_t i = 0; i < m->rows; i++) {
        printf("  ");
        for (uint8_t j = 0; j < m->cols; j++) {
            printf("%8.4f ", m->data[i][j]);
        }
        printf("\n");
    }
}
