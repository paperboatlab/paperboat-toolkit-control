// control/state_space: x' = A x + B u, y = C x + D u (s) or
// x[k+1] = A x + B u, y = C x + D u (z).

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

class StateSpaceBlock : public Block {
 public:
  StateSpaceBlock(StateSpace model, std::string domain, std::string solver_name)
      : model_(std::move(model)), domain_(std::move(domain)), solver_name_(std::move(solver_name)) {
    x_.assign(model_.n, 0.0);
    if (domain_ == "s") solver_ = solve::make(solver_name_);
  }

  Values step(const Context& ctx, const Values& in) override {
    const Vector u = input(in);
    if (domain_ == "z") {
      Vector y = output(x_, u);
      Vector x = x_;
      Vector next(model_.n, 0.0);
      for (std::size_t r = 0; r < model_.n; ++r) {
        for (std::size_t c = 0; c < model_.n; ++c) next[r] += model_.A(r, c) * x_[c];
        for (std::size_t c = 0; c < model_.m; ++c) next[r] += model_.B(r, c) * u[c];
      }
      x_ = std::move(next);
      return {{"y", std::move(y)}, {"x", std::move(x)}};
    }

    if (model_.n > 0) {
      solve::State state = x_;
      solver_->step(state, ctx.t, ctx.dt, [&](double, const solve::State& x) {
        solve::State dx(model_.n, 0.0);
        for (std::size_t r = 0; r < model_.n; ++r) {
          for (std::size_t c = 0; c < model_.n; ++c) dx[r] += model_.A(r, c) * x[c];
          for (std::size_t c = 0; c < model_.m; ++c) dx[r] += model_.B(r, c) * u[c];
        }
        return dx;
      });
      x_ = std::move(state);
    }
    return {{"y", output(x_, u)}, {"x", x_}};
  }

 private:
  Vector input(const Values& in) const {
    Vector u = as_vector(in, "u");
    if (u.empty()) u.assign(model_.m, 0.0);  // an unconnected input reads as zeros
    if (u.size() != model_.m)
      throw std::invalid_argument("u: expected " + std::to_string(model_.m) + " values, got " +
                                  std::to_string(u.size()));
    return u;
  }

  Vector output(const Vector& x, const Vector& u) const {
    Vector y(model_.p, 0.0);
    for (std::size_t r = 0; r < model_.p; ++r) {
      for (std::size_t c = 0; c < model_.n; ++c) y[r] += model_.C(r, c) * x[c];
      for (std::size_t c = 0; c < model_.m; ++c) y[r] += model_.D(r, c) * u[c];
    }
    return y;
  }

  StateSpace model_;
  std::string domain_;
  std::string solver_name_;
  solve::State x_;
  std::unique_ptr<solve::Solver> solver_;
};

}  // namespace

void add_state_space(Registry& registry) {
  auto state_space = make_template(
      "control", "state_space", "State space", {"u"}, {"y", "x"},
      {{.id = "A", .type = "text", .default_text = "0"},
       {.id = "B", .type = "text", .default_text = "1"},
       {.id = "C", .type = "text", .default_text = "1"},
       {.id = "D", .type = "text", .default_text = "0"},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}},
       solve::param()},
      [](const Values& p, const Config& c) {
        return std::make_unique<StateSpaceBlock>(parse_state_space(p, c), enum_text(p, c, "domain", {"s", "z"}, "s"),
                                                 param_text(p, c, "solver", "auto"));
      },
      "A state-space model: x' = A x + B u and y = C x + D u for s, or x[k+1] = A x + B u and "
      "y = C x + D u for z. A is n x n, B n x m, C p x n and D p x m.");
  state_space.def.inputs = {numbers_port("u")};
  state_space.def.outputs = {numbers_port("y"), numbers_port("x")};
  state_space.def.flows = {"signal"};
  state_space.def.elementwise = false;
  state_space.def.feedthrough = true;
  registry.add(std::move(state_space));
}

}  // namespace paperboat::toolkit::control
