// control/transfer_fn: a transfer function H(s) or H(z) from a numerator and a denominator.
//
// The coefficients are in descending powers, as MATLAB's tf() writes them:
//
//   H(s) = (b0 s^m + ... + bm) / (a0 s^n + ... + an)
//   H(z) = (b0 z^m + ... + bm) / (a0 z^n + ... + an)
//
// For s it builds the controllable canonical realization and integrates it with a solver
// (default: the run's). For z it runs the direct-form II transposed difference equation.

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common.hpp"
#include "control/matrix.hpp"
#include "paperboat/toolkit/solve.hpp"

namespace paperboat::toolkit::control {
namespace {

struct ProperTransfer {
  Coefficients num;  // padded to order + 1, descending powers
  Coefficients den;  // padded to order + 1, descending powers, den[0] = 1
  std::size_t order = 0;
};

Coefficients trim_leading_zeros(const Coefficients& coefficients) {
  std::size_t first = 0;
  while (first + 1 < coefficients.size() && coefficients[first] == 0.0) ++first;
  return Coefficients(coefficients.begin() + static_cast<std::ptrdiff_t>(first), coefficients.end());
}

ProperTransfer proper_transfer(const Coefficients& num, const Coefficients& den) {
  if (den.empty()) throw std::invalid_argument("den: expected at least one coefficient");
  if (den[0] == 0.0) throw std::invalid_argument("den: the leading coefficient must be nonzero");
  Coefficients numerator = trim_leading_zeros(num);
  if (numerator.size() > den.size())
    throw std::invalid_argument(
        "num: the numerator's degree must not exceed the denominator's for a proper transfer function");

  const double leading = den[0];
  for (double& value : numerator) value /= leading;
  Coefficients denominator = den;
  for (double& value : denominator) value /= leading;

  numerator.insert(numerator.begin(), denominator.size() - numerator.size(), 0.0);
  const std::size_t order = denominator.size() - 1;
  return {std::move(numerator), std::move(denominator), order};
}

class TransferFn : public Block {
 public:
  TransferFn(const Coefficients& num, const Coefficients& den, std::string domain, std::string solver_name)
      : domain_(std::move(domain)), solver_name_(std::move(solver_name)) {
    const ProperTransfer transfer = proper_transfer(num, den);
    order_ = transfer.order;
    num_ = transfer.num;
    den_ = transfer.den;

    if (domain_ == "s") {
      // Controllable canonical form: A's first row is -a1 ... -an, B is e1, C is the
      // numerator's remainder after the direct term, D is its leading coefficient.
      A_ = Matrix::zeros(order_, order_);
      B_.assign(order_, 0.0);
      C_.assign(order_, 0.0);
      if (order_ > 0) {
        for (std::size_t i = 0; i < order_; ++i) A_(0, i) = -den_[i + 1];
        for (std::size_t i = 1; i < order_; ++i) A_(i, i - 1) = 1.0;
        B_[0] = 1.0;
        D_ = num_[0];
        for (std::size_t i = 0; i < order_; ++i) C_[i] = num_[i + 1] - D_ * den_[i + 1];
      } else {
        D_ = num_[0];
      }
      x_.assign(order_, 0.0);
      solver_ = solve::make(solver_name_);
    } else {
      state_.assign(order_, 0.0);
    }
  }

  Values step(const Context& ctx, const Values& in) override {
    const double u = get_or(in, "u");
    if (domain_ == "s") {
      if (order_ > 0) {
        solve::State state = x_;
        solver_->step(state, ctx.t, ctx.dt, [&](double, const solve::State& x) {
          solve::State dx(order_, 0.0);
          for (std::size_t i = 0; i < order_; ++i) {
            dx[i] = B_[i] * u;
            for (std::size_t j = 0; j < order_; ++j) dx[i] += A_(i, j) * x[j];
          }
          return dx;
        });
        x_ = std::move(state);
      }
      double y = D_ * u;
      for (std::size_t i = 0; i < order_; ++i) y += C_[i] * x_[i];
      return {{"y", y}};
    }

    // Direct form II transposed.
    double y = num_[0] * u;
    if (order_ > 0) y += state_[0];
    for (std::size_t i = 0; i + 1 < order_; ++i)
      state_[i] = num_[i + 1] * u + state_[i + 1] - den_[i + 1] * y;
    if (order_ > 0) state_[order_ - 1] = num_[order_] * u - den_[order_] * y;
    return {{"y", y}};
  }

 private:
  std::string domain_;
  std::string solver_name_;
  std::size_t order_ = 0;
  Coefficients num_;
  Coefficients den_;
  // s
  Matrix A_;
  Vector B_, C_;
  double D_ = 0.0;
  solve::State x_;
  std::unique_ptr<solve::Solver> solver_;
  // z
  solve::State state_;
};

}  // namespace

void add_transfer_fn(Registry& registry) {
  auto transfer = make_template(
      "control", "transfer_fn", "Transfer function", {"u"}, {"y"},
      {{.id = "num", .type = "text", .default_text = "1"},
       {.id = "den", .type = "text", .default_text = "1 1"},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}},
       solve::param()},
      [](const Values& p, const Config& c) {
        return std::make_unique<TransferFn>(parse_coefficients(param_text(p, c, "num", "1"), "num"),
                                            parse_coefficients(param_text(p, c, "den", "1 1"), "den"),
                                            enum_text(p, c, "domain", {"s", "z"}, "s"),
                                            param_text(p, c, "solver", "auto"));
      },
      "A transfer function H(s) or H(z) from its numerator and denominator coefficients "
      "(descending powers). s: a controllable canonical realization, integrated; z: direct form II transposed.");
  transfer.def.flows = {"signal"};
  transfer.def.feedthrough = true;  // D (or b0) may pass its input straight to its output
  registry.add(std::move(transfer));
}

}  // namespace paperboat::toolkit::control
