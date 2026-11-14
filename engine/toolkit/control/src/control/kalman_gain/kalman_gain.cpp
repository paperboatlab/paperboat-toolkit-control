// control/kalman_gain: the steady-state Kalman gain L for x' = A x + w, y = C x + v with
// process noise covariance Q and measurement noise covariance R: the dual of lqr.
// s: L = P Cᵀ R⁻¹ (Kalman–Bucy); z: L = P Cᵀ (C P Cᵀ + R)⁻¹, the update x̂ += L (y − C x̂).

#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/riccati.hpp"

namespace paperboat::toolkit::control {
namespace {

class KalmanGain : public Block {
 public:
  KalmanGain(const Values& p, const Config& c) {
    const StateOutputs m = parse_state_outputs(p, c);
    const Matrix Q = weight_param(p, c, "Q", m.n, "one row per state"), R = weight_param(p, c, "R", m.p, "one row per output");
    const bool discrete = enum_text(p, c, "domain", {"s", "z"}, "s") == "z";
    const Matrix At = transpose(m.A), Ct = transpose(m.C);
    const Matrix P = discrete ? dare(At, Ct, Q, R) : care(At, Ct, Q, R);
    const Matrix PCt = multiply(P, Ct);
    // L = P Cᵀ S⁻¹ = (S⁻¹ C P)ᵀ, S symmetric.
    const Matrix S = discrete ? add(R, multiply(m.C, PCt)) : R;
    L_ = matrix_text(transpose(solve(S, transpose(PCt), discrete ? "C P Cᵀ + R" : "R"))), P_ = matrix_text(P);
  }
  Values step(const Context&, const Values&) override { return {{"L", L_}, {"P", P_}}; }
  bool stateless() const override { return true; }

 private:
  std::string L_, P_;
};

}  // namespace

void add_kalman_gain(Registry& registry) {
  auto t = make_template(
      "control", "kalman_gain", "Kalman gain", {}, {"L", "P"},
      {{.id = "A", .type = "text", .default_text = "0"}, {.id = "C", .type = "text", .default_text = "1"},
       {.id = "Q", .type = "text", .default_text = "1"}, {.id = "R", .type = "text", .default_text = "1"},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}}},
      [](const Values& p, const Config& c) { return std::make_unique<KalmanGain>(p, c); },
      "The steady-state Kalman gain L for process noise Q and measurement noise R, and the error "
      "covariance P, as matrix text: the gain an observer takes.");
  t.def.flows = {"signal"};
  t.def.outputs = {{.id = "L", .type = "text"}, {.id = "P", .type = "text"}};
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
