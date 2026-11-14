// control/observer: a Luenberger observer, x̂' = A x̂ + B u + L (y − C x̂) (s) or
// x̂[k+1] = A x̂ + B u + L (y − C x̂) (z). Its estimate comes from its state alone (no
// feedthrough), so it can sit in a loop with state_feedback.

#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/riccati.hpp"
#include "paperboat/toolkit/solve.hpp"

namespace paperboat::toolkit::control {
namespace {

class Observer : public Block {
 public:
  Observer(const Values& p, const Config& c) : discrete_(enum_text(p, c, "domain", {"s", "z"}, "s") == "z") {
    model_ = parse_state_space(p, c);
    L_ = parse_matrix_param(p, c, "L", "1");
    if (L_.rows != model_.n || L_.cols != model_.p)
      matrix_error("L", "expected a " + std::to_string(model_.n) + "x" + std::to_string(model_.p) + " matrix (states x outputs), got " + shape_text(L_));
    x_.assign(model_.n, 0.0);
    solver_ = solve::make(param_text(p, c, "solver", "auto"));
  }

  Values outputs() const override {
    Vector y(model_.p, 0.0);
    for (std::size_t r = 0; r < model_.p; ++r)
      for (std::size_t k = 0; k < model_.n; ++k) y[r] += model_.C(r, k) * x_[k];
    return {{"x_hat", x_}, {"y_hat", y}};
  }

  Values step(const Context& ctx, const Values& in) override {
    const Vector u = sized(as_vector(in, "u"), model_.m, "u"), y = sized(as_vector(in, "y"), model_.p, "y");
    const auto dx = [&](const Vector& x) {
      Vector out(model_.n, 0.0), err = y;
      for (std::size_t r = 0; r < model_.p; ++r)
        for (std::size_t k = 0; k < model_.n; ++k) err[r] -= model_.C(r, k) * x[k];
      for (std::size_t r = 0; r < model_.n; ++r) {
        for (std::size_t k = 0; k < model_.n; ++k) out[r] += model_.A(r, k) * x[k];
        for (std::size_t k = 0; k < model_.m; ++k) out[r] += model_.B(r, k) * u[k];
        for (std::size_t k = 0; k < model_.p; ++k) out[r] += L_(r, k) * err[k];
      }
      return out;
    };
    if (discrete_) x_ = dx(x_);
    else {
      solve::State state = x_;
      solver_->step(state, ctx.t, ctx.dt, [&](double, const solve::State& x) { return dx(x); });
      x_ = std::move(state);
    }
    return outputs();
  }

 private:
  static Vector sized(Vector v, std::size_t n, const char* id) {
    if (v.empty()) v.assign(n, 0.0);  // unconnected: zeros
    if (v.size() != n) throw std::invalid_argument(std::string(id) + ": expected " + std::to_string(n) + " values, got " + std::to_string(v.size()));
    return v;
  }
  bool discrete_;
  StateSpace model_;
  Matrix L_;
  Vector x_;
  std::unique_ptr<solve::Solver> solver_;
};

}  // namespace

void add_observer(Registry& registry) {
  auto t = make_template(
      "control", "observer", "Observer", {"u", "y"}, {"x_hat", "y_hat"},
      {{.id = "A", .type = "text", .default_text = "0"}, {.id = "B", .type = "text", .default_text = "1"},
       {.id = "C", .type = "text", .default_text = "1"}, {.id = "L", .type = "text", .default_text = "1"},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}}, solve::param()},
      [](const Values& p, const Config& c) { return std::make_unique<Observer>(p, c); },
      "Estimates the state from u and y: x̂' = A x̂ + B u + L (y − C x̂). L from kalman_gain or by hand. "
      "No feedthrough, so it can close a loop with state_feedback.");
  t.def.flows = {"signal"};
  t.def.feedthrough = false;
  t.def.inputs = {numbers_port("u"), numbers_port("y")};
  t.def.outputs = {numbers_port("x_hat"), numbers_port("y_hat")};
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
