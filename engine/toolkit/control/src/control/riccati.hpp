#pragma once

// Private to the toolkit: the dense linear algebra the state-space design blocks need: solve,
// inverse, the matrix exponential, and the continuous and discrete algebraic Riccati equations.
// Sizes are a handful of states; plain O(n³) loops throughout.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>

#include "control/matrix.hpp"

namespace paperboat::toolkit::control {

inline Matrix identity(std::size_t n) {
  Matrix out = Matrix::zeros(n, n);
  for (std::size_t i = 0; i < n; ++i) out(i, i) = 1.0;
  return out;
}

inline Matrix transpose(const Matrix& a) {
  Matrix out = Matrix::zeros(a.cols, a.rows);
  for (std::size_t r = 0; r < a.rows; ++r)
    for (std::size_t c = 0; c < a.cols; ++c) out(c, r) = a(r, c);
  return out;
}

// a + s·b
inline Matrix add(const Matrix& a, const Matrix& b, double s = 1.0) {
  if (a.rows != b.rows || a.cols != b.cols) matrix_error("matrix", "can't add a " + shape_text(a) + " and a " + shape_text(b));
  Matrix out = a;
  for (std::size_t i = 0; i < out.values.size(); ++i) out.values[i] += s * b.values[i];
  return out;
}

inline Matrix scale(Matrix a, double s) {
  for (double& v : a.values) v *= s;
  return a;
}

inline double norm1(const Matrix& a) {
  double best = 0.0;
  for (std::size_t c = 0; c < a.cols; ++c) {
    double sum = 0.0;
    for (std::size_t r = 0; r < a.rows; ++r) sum += std::abs(a(r, c));
    best = std::max(best, sum);
  }
  return best;
}

// X with A X = B, by Gaussian elimination with partial pivoting. `what` names A in the error.
inline Matrix solve(Matrix a, Matrix b, std::string_view what = "matrix") {
  if (a.rows != a.cols || a.rows != b.rows) matrix_error(what, "can't solve a " + shape_text(a) + " system for a " + shape_text(b));
  const std::size_t n = a.rows;
  const double tolerance = 1e-12 * std::max(1.0, norm1(a));
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    for (std::size_t r = col + 1; r < n; ++r)
      if (std::abs(a(r, col)) > std::abs(a(pivot, col))) pivot = r;
    if (std::abs(a(pivot, col)) <= tolerance) matrix_error(what, "is singular");
    for (std::size_t c = 0; c < n; ++c) std::swap(a(col, c), a(pivot, c));
    for (std::size_t c = 0; c < b.cols; ++c) std::swap(b(col, c), b(pivot, c));
    for (std::size_t r = 0; r < n; ++r) {
      if (r == col) continue;
      const double f = a(r, col) / a(col, col);
      if (f == 0.0) continue;
      for (std::size_t c = col; c < n; ++c) a(r, c) -= f * a(col, c);
      for (std::size_t c = 0; c < b.cols; ++c) b(r, c) -= f * b(col, c);
    }
  }
  for (std::size_t r = 0; r < n; ++r)
    for (std::size_t c = 0; c < b.cols; ++c) b(r, c) /= a(r, r);
  return b;
}

inline Matrix inverse(const Matrix& a, std::string_view what = "matrix") { return solve(a, identity(a.rows), what); }

// e^A: Padé(6) with scaling and squaring.
inline Matrix expm(const Matrix& a) {
  const double norm = norm1(a);
  const int squarings = norm > 0.5 ? static_cast<int>(std::ceil(std::log2(norm / 0.5))) : 0;
  const Matrix x = scale(a, std::pow(2.0, -squarings));
  const double c[] = {1.0, 0.5, 5.0 / 44, 1.0 / 66, 1.0 / 792, 1.0 / 15840, 1.0 / 665280};
  Matrix power = identity(a.rows), num = identity(a.rows), den = identity(a.rows);
  for (int k = 1; k <= 6; ++k) {
    power = multiply(power, x);
    num = add(num, power, c[k]);
    den = add(den, power, (k % 2 ? -1.0 : 1.0) * c[k]);
  }
  Matrix out = solve(den, num);
  for (int i = 0; i < squarings; ++i) out = multiply(out, out);
  return out;
}

// P of the continuous ARE  Aᵀ P + P A − P B R⁻¹ Bᵀ P + Q = 0, by the matrix sign function of
// the Hamiltonian (no eigen-solver needed).
inline Matrix care(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R) {
  const std::size_t n = A.rows;
  const Matrix G = multiply(multiply(B, inverse(R, "R")), transpose(B));
  Matrix Z = Matrix::zeros(2 * n, 2 * n);
  for (std::size_t r = 0; r < n; ++r)
    for (std::size_t c = 0; c < n; ++c) {
      Z(r, c) = A(r, c);
      Z(r, c + n) = -G(r, c);
      Z(r + n, c) = -Q(r, c);
      Z(r + n, c + n) = -A(c, r);
    }
  for (int it = 0; it < 200; ++it) {
    const Matrix Zi = inverse(Z, "the Hamiltonian (is the system stabilizable?)");
    // Determinant-free scaling: |Z|/|Z⁻¹| in the 1-norm speeds the early iterations.
    const double g = std::sqrt(norm1(Zi) / norm1(Z));
    const Matrix next = scale(add(scale(Z, g), Zi, 1.0 / g), 0.5);
    const double change = norm1(add(next, Z, -1.0));
    Z = next;
    if (change <= 1e-13 * norm1(Z)) break;
  }
  // [Z12; Z22 + I] P = −[Z11 + I; Z21], solved by least squares (normal equations).
  Matrix M = Matrix::zeros(2 * n, n), N = Matrix::zeros(2 * n, n);
  for (std::size_t r = 0; r < n; ++r)
    for (std::size_t c = 0; c < n; ++c) {
      M(r, c) = Z(r, c + n);
      M(r + n, c) = Z(r + n, c + n) + (r == c);
      N(r, c) = -(Z(r, c) + (r == c));
      N(r + n, c) = -Z(r + n, c);
    }
  const Matrix Mt = transpose(M);
  const Matrix P = solve(multiply(Mt, M), multiply(Mt, N), "the Riccati solution (is the system stabilizable?)");
  return scale(add(P, transpose(P)), 0.5);
}

// P of the discrete ARE  P = Aᵀ P A − Aᵀ P B (R + Bᵀ P B)⁻¹ Bᵀ P A + Q, by iterating it.
inline Matrix dare(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R) {
  Matrix P = Q;
  const Matrix At = transpose(A), Bt = transpose(B);
  for (int it = 0; it < 100000; ++it) {
    const Matrix PA = multiply(P, A), PB = multiply(P, B);
    const Matrix gain = solve(add(R, multiply(Bt, PB)), multiply(Bt, PA), "R + BᵀPB");
    const Matrix next = add(add(Q, multiply(At, PA)), multiply(multiply(At, PB), gain), -1.0);
    const double change = norm1(add(next, P, -1.0));
    P = scale(add(next, transpose(next)), 0.5);
    if (change <= 1e-12 * std::max(1.0, norm1(P))) return P;
  }
  throw std::invalid_argument("the Riccati iteration did not converge: is the system stabilizable?");
}

// A square weight or covariance param of size n; a single number is that times the identity.
inline Matrix weight_param(const Values& p, const Config& c, const char* id, std::size_t n, const char* what) {
  Matrix m = parse_matrix_param(p, c, id, "1");
  if (m.rows == 1 && m.cols == 1 && n > 1) m = scale(identity(n), m(0, 0));
  if (m.rows != n || m.cols != n)
    matrix_error(id, "expected a " + std::to_string(n) + "x" + std::to_string(n) + " matrix (" + what + "), got " + shape_text(m));
  return m;
}

// A matrix as the text the matrix params parse: "1 2; 3 4".
inline std::string matrix_text(const Matrix& m) {
  std::string out;
  char buf[32];
  for (std::size_t r = 0; r < m.rows; ++r) {
    if (r) out += "; ";
    for (std::size_t c = 0; c < m.cols; ++c) {
      std::snprintf(buf, sizeof buf, "%.10g", m(r, c));
      out += (c ? " " : "") + std::string(buf);
    }
  }
  return out;
}

}  // namespace paperboat::toolkit::control
