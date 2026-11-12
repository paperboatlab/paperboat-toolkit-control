#pragma once

// Private to the toolkit: MATLAB-style text parsing for coefficient lists and matrices, a
// small dense matrix for the state-space blocks, and rank by Gaussian elimination.
//
// Coefficients and matrices are written as they are in MATLAB:
//
//   "1 2 1"              a coefficient list
//   "[1, 2, 1]"          the same list
//   "1 0; 0 1"           a 2x2 matrix, one row per line (newlines work too)
//   "[[1, 2], [3, 4]]"   the same matrix
//   "[]"                 an empty matrix (no rows)

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common.hpp"

namespace paperboat::toolkit::control {

using Coefficients = Vector;

// A dense row-major matrix.
struct Matrix {
  std::size_t rows = 0;
  std::size_t cols = 0;
  Vector values;

  double& operator()(std::size_t row, std::size_t col) { return values[row * cols + col]; }
  double operator()(std::size_t row, std::size_t col) const { return values[row * cols + col]; }

  static Matrix zeros(std::size_t rows, std::size_t cols) {
    Matrix out;
    out.rows = rows;
    out.cols = cols;
    out.values.assign(rows * cols, 0.0);
    return out;
  }
};

[[noreturn]] inline void matrix_error(std::string_view param, const std::string& message) {
  throw std::invalid_argument(std::string(param) + ": " + message);
}

inline std::string_view trim(std::string_view text) {
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) text.remove_prefix(1);
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.remove_suffix(1);
  return text;
}

// A text param: a direct call to Template::make puts text params in `params`; the catalog
// puts them in `config` and their enum index in `params`. Both paths work here.
inline std::string param_text(const Values& params, const Config& config, std::string_view id, std::string fallback) {
  if (const auto it = params.find(id); it != params.end())
    if (const auto* text = std::get_if<std::string>(&it->second)) return *text;
  if (const auto it = config.find(id); it != config.end()) return it->second;
  return fallback;
}

inline std::string enum_text(const Values& params, const Config& config, std::string_view id,
                             const std::vector<std::string>& options, std::string fallback) {
  if (const auto it = params.find(id); it != params.end()) {
    if (const auto* text = std::get_if<std::string>(&it->second)) return *text;
    if (const auto* index = std::get_if<double>(&it->second)) {
      const auto choice = static_cast<std::size_t>(std::lround(*index));
      if (choice < options.size()) return options[choice];
    }
  }
  if (const auto it = config.find(id); it != config.end()) return it->second;
  return fallback;
}

// Reads one number at `i` (leading whitespace is skipped).
inline bool read_number(std::string_view text, std::size_t& i, double& out) {
  while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
  if (i >= text.size()) return false;
  const std::string rest(text.substr(i));
  char* end = nullptr;
  errno = 0;
  const double value = std::strtod(rest.c_str(), &end);
  if (end == rest.c_str()) return false;
  i += static_cast<std::size_t>(end - rest.c_str());
  out = value;
  return true;
}

inline Coefficients parse_row(std::string_view text, std::string_view param) {
  Coefficients out;
  std::size_t i = 0;
  while (i < text.size()) {
    const char c = text[i];
    if (std::isspace(static_cast<unsigned char>(c)) || c == ',') {
      ++i;
      continue;
    }
    if (c == ';') break;
    if (c == '[' || c == ']') matrix_error(param, "unexpected '" + std::string(1, c) + "'");
    const std::size_t start = i;
    double value = 0.0;
    if (!read_number(text, i, value))
      matrix_error(param, "expected a number at \"" + std::string(text.substr(start)) + "\"");
    if (!std::isfinite(value))
      matrix_error(param, "expected a finite number, got \"" + std::string(text.substr(start, i - start)) + "\"");
    out.push_back(value);
  }
  return out;
}

inline Coefficients parse_coefficients(std::string_view text, std::string_view param);

inline Matrix parse_nested_rows(std::string_view text, std::string_view param) {
  Matrix out;
  std::size_t i = 0;
  while (i < text.size()) {
    const char c = text[i];
    if (std::isspace(static_cast<unsigned char>(c)) || c == ',' || c == ';') {
      ++i;
      continue;
    }
    if (c != '[') matrix_error(param, "expected a row in brackets, found \"" + std::string(1, c) + "\"");
    const std::size_t start = i;
    std::size_t depth = 0;
    for (; i < text.size(); ++i) {
      if (text[i] == '[') ++depth;
      if (text[i] == ']' && --depth == 0) {
        ++i;
        break;
      }
    }
    if (depth != 0) matrix_error(param, "unclosed '['");
    const Coefficients row = parse_coefficients(text.substr(start + 1, i - start - 2), param);
    if (out.rows == 0) {
      out.cols = row.size();
    } else if (row.size() != out.cols) {
      matrix_error(param, "row " + std::to_string(out.rows) + " has " + std::to_string(row.size()) +
                              " values, expected " + std::to_string(out.cols));
    }
    out.values.insert(out.values.end(), row.begin(), row.end());
    ++out.rows;
  }
  return out;
}

inline Matrix parse_flat_rows(std::string_view text, std::string_view param) {
  Matrix out;
  std::size_t start = 0;
  for (std::size_t i = 0; i <= text.size(); ++i) {
    if (i < text.size() && text[i] != ';' && text[i] != '\n') continue;
    const std::string_view row_text = trim(text.substr(start, i - start));
    start = i + 1;
    if (row_text.empty()) continue;
    const Coefficients row = parse_row(row_text, param);
    if (row.empty()) continue;
    if (out.rows == 0) {
      out.cols = row.size();
    } else if (row.size() != out.cols) {
      matrix_error(param, "row " + std::to_string(out.rows) + " has " + std::to_string(row.size()) +
                              " values, expected " + std::to_string(out.cols));
    }
    out.values.insert(out.values.end(), row.begin(), row.end());
    ++out.rows;
  }
  return out;
}

inline Matrix parse_matrix(std::string_view text, std::string_view param) {
  std::string_view s = trim(text);
  if (s.empty()) return {};
  if (s == "[]") return {};

  // Strip one matching pair of outer brackets.
  if (s.size() >= 2 && s.front() == '[' && s.back() == ']') {
    int depth = 0;
    for (std::size_t i = 0; i < s.size(); ++i) {
      if (s[i] == '[') ++depth;
      else if (s[i] == ']') {
        if (--depth < 0) matrix_error(param, "unexpected ']'");
        if (depth == 0) {
          if (i + 1 != s.size()) matrix_error(param, "unexpected text after ']'");
          s = trim(s.substr(1, s.size() - 2));
          break;
        }
      }
    }
    if (depth != 0) matrix_error(param, "unclosed '['");
  }

  if (s.empty()) return {};
  if (s.front() == '[') return parse_nested_rows(s, param);
  return parse_flat_rows(s, param);
}

inline Coefficients parse_coefficients(std::string_view text, std::string_view param) {
  const Matrix m = parse_matrix(text, param);
  if (m.rows == 0) matrix_error(param, "expected at least one coefficient");
  if (m.rows != 1)
    matrix_error(param, "expected a list of coefficients, got a " + std::to_string(m.rows) + "x" +
                            std::to_string(m.cols) + " matrix");
  return m.values;
}

inline std::string shape_text(const Matrix& matrix) {
  return std::to_string(matrix.rows) + "x" + std::to_string(matrix.cols);
}

inline Matrix require_square(Matrix matrix, std::string_view param) {
  if (matrix.rows == 0 || matrix.cols == 0) matrix_error(param, "expected a non-empty square matrix");
  if (matrix.rows != matrix.cols) matrix_error(param, "expected a square matrix, got " + shape_text(matrix));
  return matrix;
}

inline Matrix multiply(const Matrix& a, const Matrix& b) {
  if (a.cols != b.rows)
    matrix_error("matrix", "can't multiply a " + shape_text(a) + " by a " + shape_text(b));
  Matrix out = Matrix::zeros(a.rows, b.cols);
  for (std::size_t r = 0; r < a.rows; ++r)
    for (std::size_t k = 0; k < a.cols; ++k)
      for (std::size_t c = 0; c < b.cols; ++c) out(r, c) += a(r, k) * b(k, c);
  return out;
}

// The rank by Gaussian elimination with partial pivoting. A pivot is zero when it is below
// `relative_tolerance` times the largest entry of the matrix.
inline std::size_t matrix_rank(Matrix a, double relative_tolerance = 1e-9) {
  if (a.rows == 0 || a.cols == 0) return 0;
  double scale = 0.0;
  for (double value : a.values) scale = std::max(scale, std::abs(value));
  if (scale == 0.0) return 0;
  const double tolerance = relative_tolerance * scale;

  std::size_t rank = 0;
  for (std::size_t col = 0; col < a.cols && rank < a.rows; ++col) {
    std::size_t pivot = rank;
    double best = std::abs(a(rank, col));
    for (std::size_t row = rank + 1; row < a.rows; ++row) {
      const double value = std::abs(a(row, col));
      if (value > best) {
        best = value;
        pivot = row;
      }
    }
    if (best <= tolerance) continue;
    if (pivot != rank)
      for (std::size_t c = 0; c < a.cols; ++c) std::swap(a(rank, c), a(pivot, c));
    for (std::size_t row = rank + 1; row < a.rows; ++row) {
      const double factor = a(row, col) / a(rank, col);
      if (factor == 0.0) continue;
      for (std::size_t c = col; c < a.cols; ++c) a(row, c) -= factor * a(rank, c);
    }
    ++rank;
  }
  return rank;
}

inline Matrix controllability_matrix(const Matrix& a, const Matrix& b) {
  Matrix out = Matrix::zeros(a.rows, a.rows * b.cols);
  Matrix block = b;
  for (std::size_t k = 0; k < a.rows; ++k) {
    for (std::size_t r = 0; r < block.rows; ++r)
      for (std::size_t c = 0; c < block.cols; ++c) out(r, k * block.cols + c) = block(r, c);
    if (k + 1 < a.rows) block = multiply(a, block);
  }
  return out;
}

inline Matrix observability_matrix(const Matrix& a, const Matrix& c) {
  Matrix out = Matrix::zeros(a.rows * c.rows, a.rows);
  Matrix block = c;
  for (std::size_t k = 0; k < a.rows; ++k) {
    for (std::size_t r = 0; r < block.rows; ++r)
      for (std::size_t col = 0; col < block.cols; ++col) out(k * block.rows + r, col) = block(r, col);
    if (k + 1 < a.rows) block = multiply(block, a);
  }
  return out;
}

// A parsed state-space model. n states, m inputs, p outputs.
struct StateSpace {
  Matrix A;
  Matrix B;
  Matrix C;
  Matrix D;
  std::size_t n = 0;
  std::size_t m = 0;
  std::size_t p = 0;
};

struct StateInputs {
  Matrix A;
  Matrix B;
  std::size_t n = 0;
  std::size_t m = 0;
};

struct StateOutputs {
  Matrix A;
  Matrix C;
  std::size_t n = 0;
  std::size_t p = 0;
};

inline Matrix parse_matrix_param(const Values& params, const Config& config, std::string_view id,
                                 std::string fallback) {
  return parse_matrix(param_text(params, config, id, std::move(fallback)), id);
}

inline Matrix parse_A(const Values& params, const Config& config) {
  return require_square(parse_matrix_param(params, config, "A", "0"), "A");
}

inline StateInputs parse_state_inputs(const Values& params, const Config& config) {
  StateInputs out;
  out.A = parse_A(params, config);
  out.n = out.A.rows;
  out.B = parse_matrix_param(params, config, "B", "1");
  if (out.B.rows == 0 || out.B.cols == 0) matrix_error("B", "expected a non-empty matrix");
  if (out.B.rows != out.n)
    matrix_error("B", "expected " + std::to_string(out.n) + " rows (one per state), got " + shape_text(out.B) +
                          (out.B.rows == 1 && out.B.cols == 1 ? "; a single number is a 1x1 matrix" : ""));
  out.m = out.B.cols;
  return out;
}

inline StateOutputs parse_state_outputs(const Values& params, const Config& config) {
  StateOutputs out;
  out.A = parse_A(params, config);
  out.n = out.A.rows;
  out.C = parse_matrix_param(params, config, "C", "1");
  if (out.C.rows == 0 || out.C.cols == 0) matrix_error("C", "expected a non-empty matrix");
  if (out.C.cols != out.n)
    matrix_error("C", "expected " + std::to_string(out.n) + " columns (one per state), got " + shape_text(out.C) +
                          (out.C.rows == 1 && out.C.cols == 1 ? "; a single number is a 1x1 matrix" : ""));
  out.p = out.C.rows;
  return out;
}

inline StateSpace parse_state_space(const Values& params, const Config& config) {
  StateSpace out;
  out.A = parse_A(params, config);
  out.n = out.A.rows;

  out.B = parse_matrix_param(params, config, "B", "1");
  if (out.B.rows == 0 || out.B.cols == 0) matrix_error("B", "expected a non-empty matrix");
  if (out.B.rows != out.n)
    matrix_error("B", "expected " + std::to_string(out.n) + " rows (one per state), got " + shape_text(out.B) +
                          (out.B.rows == 1 && out.B.cols == 1 ? "; a single number is a 1x1 matrix" : ""));
  out.m = out.B.cols;

  out.C = parse_matrix_param(params, config, "C", "1");
  if (out.C.rows == 0 || out.C.cols == 0) matrix_error("C", "expected a non-empty matrix");
  if (out.C.cols != out.n)
    matrix_error("C", "expected " + std::to_string(out.n) + " columns (one per state), got " + shape_text(out.C) +
                          (out.C.rows == 1 && out.C.cols == 1 ? "; a single number is a 1x1 matrix" : ""));
  out.p = out.C.rows;

  out.D = parse_matrix_param(params, config, "D", "0");
  if (out.D.rows == 0 || out.D.cols == 0) {
    out.D = Matrix::zeros(out.p, out.m);  // [] or an empty text: no direct feedthrough
  } else if (out.D.rows == 1 && out.D.cols == 1 && (out.p != 1 || out.m != 1)) {
    if (out.D(0, 0) == 0.0) out.D = Matrix::zeros(out.p, out.m);  // "0" means the zero matrix
    else
      matrix_error("D", "expected a " + std::to_string(out.p) + "x" + std::to_string(out.m) +
                            " matrix, got 1x1; give the full matrix, or [] for zeros");
  } else if (out.D.rows != out.p || out.D.cols != out.m) {
    matrix_error("D", "expected a " + std::to_string(out.p) + "x" + std::to_string(out.m) + " matrix, got " +
                          shape_text(out.D));
  }
  return out;
}

}  // namespace paperboat::toolkit::control
