// control/kalman: a discrete Kalman filter on x[k+1] = A x + B u + w, y = C x + v, with
// process noise covariance Q and measurement noise covariance R. Each sample: predict from
// the last estimate and u, then update with y. Outputs the filtered x̂, the trace of its
// covariance P (how sure it is) and the innovation y − C x̂⁻.

#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/riccati.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class Kalman : public Block {
 public:
  Kalman(const Values& p, const Config& c) : sampler_{get_or(p, "sample_time")} {
    model_ = parse_state_space(p, c);
    Q_ = weight_param(p, c, "Q", model_.n, "one row per state");
    R_ = weight_param(p, c, "R", model_.p, "one row per output");
    const Matrix x0 = parse_matrix_param(p, c, "x0", "");
    if (x0.values.empty()) x_ = Matrix::zeros(model_.n, 1);
    else if (x0.values.size() == model_.n) x_ = Matrix{model_.n, 1, x0.values};
    else matrix_error("x0", "expected " + std::to_string(model_.n) + " values, got " + std::to_string(x0.values.size()));
    P_ = scale(identity(model_.n), get_or(p, "P0", 1.0));
    innovation_ = Vector(model_.p, 0.0);
  }

  Values step(const Context& ctx, const Values& in) override {
    if (sampler_.due(ctx.t)) {
      const Vector u = sized(as_vector(in, "u"), model_.m, "u"), y = sized(as_vector(in, "y"), model_.p, "y");
      const Matrix U{model_.m, 1, u}, Y{model_.p, 1, y};
      if (started_) {  // predict (the first sample updates x0 directly)
        x_ = add(multiply(model_.A, x_), multiply(model_.B, U));
        P_ = add(multiply(multiply(model_.A, P_), transpose(model_.A)), Q_);
      }
      started_ = true;
      const Matrix Ct = transpose(model_.C);
      const Matrix e = add(Y, multiply(model_.C, x_), -1.0);
      const Matrix S = add(multiply(multiply(model_.C, P_), Ct), R_);
      const Matrix K = transpose(solve(S, transpose(multiply(P_, Ct)), "C P Cᵀ + R"));
      x_ = add(x_, multiply(K, e));
      P_ = multiply(add(identity(model_.n), multiply(K, model_.C), -1.0), P_);
      P_ = scale(add(P_, transpose(P_)), 0.5);
      innovation_ = e.values;
    }
    double trace = 0.0;
    for (std::size_t i = 0; i < model_.n; ++i) trace += P_(i, i);
    return {{"x_hat", x_.values}, {"P_trace", trace}, {"innovation", innovation_}};
  }

 private:
  static Vector sized(Vector v, std::size_t n, const char* id) {
    if (v.empty()) v.assign(n, 0.0);
    if (v.size() != n) throw std::invalid_argument(std::string(id) + ": expected " + std::to_string(n) + " values, got " + std::to_string(v.size()));
    return v;
  }
  StateSpace model_;
  Matrix Q_, R_, x_, P_;
  Vector innovation_;
  Sampler sampler_;
  bool started_ = false;
};

}  // namespace

void add_kalman(Registry& registry) {
  auto t = make_template(
      "control", "kalman", "Kalman filter", {"u", "y"}, {"x_hat", "P_trace", "innovation"},
      {{.id = "A", .type = "text", .default_text = "1"}, {.id = "B", .type = "text", .default_text = "0"},
       {.id = "C", .type = "text", .default_text = "1"}, {.id = "Q", .type = "text", .default_text = "0.01"},
       {.id = "R", .type = "text", .default_text = "1"}, {.id = "x0", .type = "text", .default_text = ""},
       {.id = "P0", .default_value = 1.0}, {.id = "sample_time", .default_value = 0.0, .unit = "s"}},
      [](const Values& p, const Config& c) { return std::make_unique<Kalman>(p, c); },
      "A discrete Kalman filter: each sample (every step, or sample_time) it predicts with A, B and u, "
      "then corrects with y. Q: process noise, R: measurement noise (a number is that times I).");
  t.def.flows = {"signal"};
  t.def.inputs = {numbers_port("u"), numbers_port("y")};
  t.def.outputs = {numbers_port("x_hat"), PortDef{.id = "P_trace"}, numbers_port("innovation")};
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
