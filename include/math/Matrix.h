#pragma once

#include <cstring>
#include <cmath>
#include <cstdint>

// ============================================================
//  Matrix.h  —  Fixed-size matrix math, header-only.
//
//  Template parameters:
//    ROWS — number of rows
//    COLS — number of columns
//
//  All operations are stack-allocated — no heap use.
//  Intended for small matrices (≤ 8x8) as used in Kalman/LQR.
//
//  Usage:
//    Matrix<3,3> A;
//    Matrix<3,1> b;
//    auto c = A * b;          // Matrix<3,1>
//    auto At = A.transposed(); // Matrix<3,3>
// ============================================================

template<uint8_t ROWS, uint8_t COLS>
class Matrix {
public:

    // ----------------------------------------------------------
    //  Construction
    // ----------------------------------------------------------

    // Default: zero matrix
    Matrix() { zero(); }

    // Fill with a single scalar value
    explicit Matrix(float fill) {
        for (uint8_t i = 0; i < ROWS * COLS; i++) data_[i] = fill;
    }

    // Construct from a flat row-major array
    explicit Matrix(const float (&src)[ROWS * COLS]) {
        memcpy(data_, src, sizeof(data_));
    }

    // ----------------------------------------------------------
    //  Element access  (row, col) — zero-indexed
    // ----------------------------------------------------------
    float& operator()(uint8_t r, uint8_t c)       { return data_[r * COLS + c]; }
    float  operator()(uint8_t r, uint8_t c) const { return data_[r * COLS + c]; }

    // ----------------------------------------------------------
    //  Fill / identity helpers
    // ----------------------------------------------------------
    void zero() { memset(data_, 0, sizeof(data_)); }

    // Only valid for square matrices — asserts at compile time
    void setIdentity() {
        static_assert(ROWS == COLS, "setIdentity() requires a square matrix");
        zero();
        for (uint8_t i = 0; i < ROWS; i++) (*this)(i, i) = 1.0f;
    }

    // ----------------------------------------------------------
    //  Matrix addition:  C = A + B
    // ----------------------------------------------------------
    Matrix<ROWS, COLS> operator+(const Matrix<ROWS, COLS>& rhs) const {
        Matrix<ROWS, COLS> result;
        for (uint8_t i = 0; i < ROWS * COLS; i++)
            result.data_[i] = data_[i] + rhs.data_[i];
        return result;
    }

    Matrix<ROWS, COLS>& operator+=(const Matrix<ROWS, COLS>& rhs) {
        for (uint8_t i = 0; i < ROWS * COLS; i++) data_[i] += rhs.data_[i];
        return *this;
    }

    // ----------------------------------------------------------
    //  Matrix subtraction:  C = A - B
    // ----------------------------------------------------------
    Matrix<ROWS, COLS> operator-(const Matrix<ROWS, COLS>& rhs) const {
        Matrix<ROWS, COLS> result;
        for (uint8_t i = 0; i < ROWS * COLS; i++)
            result.data_[i] = data_[i] - rhs.data_[i];
        return result;
    }

    Matrix<ROWS, COLS>& operator-=(const Matrix<ROWS, COLS>& rhs) {
        for (uint8_t i = 0; i < ROWS * COLS; i++) data_[i] -= rhs.data_[i];
        return *this;
    }

    // ----------------------------------------------------------
    //  Scalar multiplication:  B = A * s
    // ----------------------------------------------------------
    Matrix<ROWS, COLS> operator*(float s) const {
        Matrix<ROWS, COLS> result;
        for (uint8_t i = 0; i < ROWS * COLS; i++) result.data_[i] = data_[i] * s;
        return result;
    }

    Matrix<ROWS, COLS>& operator*=(float s) {
        for (uint8_t i = 0; i < ROWS * COLS; i++) data_[i] *= s;
        return *this;
    }

    // ----------------------------------------------------------
    //  Matrix multiplication:  C(ROWS x OTHER_COLS) = A(ROWS x COLS) * B(COLS x OTHER_COLS)
    // ----------------------------------------------------------
    template<uint8_t OTHER_COLS>
    Matrix<ROWS, OTHER_COLS> operator*(const Matrix<COLS, OTHER_COLS>& rhs) const {
        Matrix<ROWS, OTHER_COLS> result;   // zero-initialized by default ctor
        for (uint8_t r = 0; r < ROWS; r++)
            for (uint8_t c = 0; c < OTHER_COLS; c++)
                for (uint8_t k = 0; k < COLS; k++)
                    result(r, c) += (*this)(r, k) * rhs(k, c);
        return result;
    }

    // ----------------------------------------------------------
    //  Transpose:  B(COLS x ROWS) = Aᵀ
    // ----------------------------------------------------------
    Matrix<COLS, ROWS> transposed() const {
        Matrix<COLS, ROWS> result;
        for (uint8_t r = 0; r < ROWS; r++)
            for (uint8_t c = 0; c < COLS; c++)
                result(c, r) = (*this)(r, c);
        return result;
    }

    // ----------------------------------------------------------
    //  In-place transpose — only valid for square matrices
    // ----------------------------------------------------------
    void transposeInPlace() {
        static_assert(ROWS == COLS, "transposeInPlace() requires a square matrix");
        for (uint8_t r = 0; r < ROWS; r++)
            for (uint8_t c = r + 1; c < COLS; c++) {
                float tmp       = (*this)(r, c);
                (*this)(r, c)   = (*this)(c, r);
                (*this)(c, r)   = tmp;
            }
    }

    // ----------------------------------------------------------
    //  Frobenius norm:  sqrt(Σ aᵢⱼ²)
    //  Useful for convergence checks in Kalman P matrix
    // ----------------------------------------------------------
    float norm() const {
        float sum = 0.0f;
        for (uint8_t i = 0; i < ROWS * COLS; i++) sum += data_[i] * data_[i];
        return sqrtf(sum);
    }

    // ----------------------------------------------------------
    //  Inversion — Gauss-Jordan, square matrices only (≤ 8x8)
    //  Returns false if matrix is singular (determinant ≈ 0).
    //  Result is written to `out` only if successful.
    //
    //  NOTE: For 2x2 Kalman innovation covariance inversion,
    //        prefer inverse2x2() below — it is faster and exact.
    // ----------------------------------------------------------
    bool inverse(Matrix<ROWS, COLS>& out) const {
        static_assert(ROWS == COLS, "inverse() requires a square matrix");

        // Augment [this | I]
        float aug[ROWS][COLS * 2];
        for (uint8_t r = 0; r < ROWS; r++) {
            for (uint8_t c = 0; c < COLS; c++) {
                aug[r][c]        = (*this)(r, c);
                aug[r][c + COLS] = (r == c) ? 1.0f : 0.0f;
            }
        }

        // Forward elimination with partial pivoting
        for (uint8_t col = 0; col < ROWS; col++) {
            // Find pivot
            uint8_t pivot = col;
            for (uint8_t r = col + 1; r < ROWS; r++)
                if (fabsf(aug[r][col]) > fabsf(aug[pivot][col])) pivot = r;

            // Swap rows
            if (pivot != col)
                for (uint8_t c = 0; c < COLS * 2; c++) {
                    float tmp       = aug[col][c];
                    aug[col][c]     = aug[pivot][c];
                    aug[pivot][c]   = tmp;
                }

            // Singular check
            if (fabsf(aug[col][col]) < 1e-9f) return false;

            // Scale pivot row
            float scale = aug[col][col];
            for (uint8_t c = 0; c < COLS * 2; c++) aug[col][c] /= scale;

            // Eliminate column
            for (uint8_t r = 0; r < ROWS; r++) {
                if (r == col) continue;
                float factor = aug[r][col];
                for (uint8_t c = 0; c < COLS * 2; c++)
                    aug[r][c] -= factor * aug[col][c];
            }
        }

        // Extract result
        for (uint8_t r = 0; r < ROWS; r++)
            for (uint8_t c = 0; c < COLS; c++)
                out(r, c) = aug[r][c + COLS];

        return true;
    }

    // ----------------------------------------------------------
    //  Fast exact 2x2 inverse — use for Kalman S matrix inversion
    //  Returns false if singular.
    // ----------------------------------------------------------
    bool inverse2x2(Matrix<2, 2>& out) const {
        static_assert(ROWS == 2 && COLS == 2, "inverse2x2() requires a 2x2 matrix");
        float det = (*this)(0,0) * (*this)(1,1) - (*this)(0,1) * (*this)(1,0);
        if (fabsf(det) < 1e-9f) return false;
        float invDet = 1.0f / det;
        out(0,0) =  (*this)(1,1) * invDet;
        out(0,1) = -(*this)(0,1) * invDet;
        out(1,0) = -(*this)(1,0) * invDet;
        out(1,1) =  (*this)(0,0) * invDet;
        return true;
    }

    // ----------------------------------------------------------
    //  Equality check with a tolerance (useful in tests)
    // ----------------------------------------------------------
    bool approxEqual(const Matrix<ROWS, COLS>& other, float tol = 1e-5f) const {
        for (uint8_t i = 0; i < ROWS * COLS; i++)
            if (fabsf(data_[i] - other.data_[i]) > tol) return false;
        return true;
    }

    // ----------------------------------------------------------
    //  Raw data access (row-major flat array)
    //  Useful for serial debug dumps
    // ----------------------------------------------------------
    const float* data() const { return data_; }
    float*       data()       { return data_; }

    // ----------------------------------------------------------
    //  Dimensions
    // ----------------------------------------------------------
    static constexpr uint8_t rows() { return ROWS; }
    static constexpr uint8_t cols() { return COLS; }

private:
    float data_[ROWS * COLS]{};
};

// ----------------------------------------------------------
//  Convenience aliases used throughout the control stack
// ----------------------------------------------------------
using Vec2   = Matrix<2, 1>;   // [x, y] or [τ_x, τ_y]
using Vec4   = Matrix<4, 1>;   // [x, ẋ, y, ẏ] — Kalman state
using Vec6   = Matrix<6, 1>;   // [e, ė, ∫e] per axis — LQR augmented state
using Mat2x2 = Matrix<2, 2>;
using Mat4x4 = Matrix<4, 4>;
using Mat2x6 = Matrix<2, 6>;   // LQR gain matrix K
