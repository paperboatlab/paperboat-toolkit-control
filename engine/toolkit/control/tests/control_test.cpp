// The control package's clock dynamics, moved here from the engine's built-in control package.

#include <cmath>
#include <memory>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "paperboat/toolkit/control.hpp"
#include "paperboat/toolkit/control/simulator.hpp"
#include "paperboat/toolkit/descriptor.hpp"
#include "paperboat/toolkit/registry.hpp"

using namespace paperboat::toolkit;
using Catch::Matchers::WithinAbs;

namespace {

const Registry& registry() {
  static const Registry r = [] {
    Registry out = builtins();
    control::register_control(out);
    return out;
  }();
  return r;
}

std::unique_ptr<Block> make(const std::string& name, const Values& params) {
  const Template* t = registry().find(name);
  REQUIRE(t != nullptr);
  return t->make(params, t->config);
}

double out(Block& b, double t, const Values& in = {}, const std::string& port = "y", double dt = 0.01) {
  return get_or(b.step(Context{t, dt}, in), port);
}

}  // namespace

TEST_CASE("control templates are named by group and typed by their last segment") {
  for (const char* name : {"control/pid", "control/integrator", "control/lag", "control/delay",
                           "control/mpc", "control/lp_step", "control/nmpc"}) {
    const Template* t = registry().find(name);
    REQUIRE(t != nullptr);
    CHECK(std::string(name).ends_with("/" + t->def.type));
  }
}

TEST_CASE("pid") {
  const double dt = 0.1;
  auto u = [&](Block& b, double t, double e) { return out(b, t, {{"e", e}}, "u", dt); };

  SECTION("proportional only") {
    auto p = make("control/pid", {{"kp", 4}, {"ki", 0}, {"kd", 0}});
    CHECK(u(*p, 0, 0.5) == 2.0);
  }
  SECTION("integral accumulates e·dt") {
    auto i = make("control/pid", {{"kp", 0}, {"ki", 1}, {"kd", 0}});
    u(*i, 0.0, 1);
    u(*i, 0.1, 1);
    CHECK_THAT(u(*i, 0.2, 1), WithinAbs(0.3, 1e-12));
  }
  SECTION("derivative is zero on the first step, then a backward difference") {
    auto d = make("control/pid", {{"kp", 0}, {"ki", 0}, {"kd", 1}});
    CHECK(u(*d, 0.0, 1) == 0.0);
    CHECK_THAT(u(*d, 0.1, 1.5), WithinAbs(5.0, 1e-12));
  }
  SECTION("instances don't share state") {
    auto a = make("control/pid", {{"kp", 0}, {"ki", 1}, {"kd", 0}});
    auto b = make("control/pid", {{"kp", 0}, {"ki", 1}, {"kd", 0}});
    u(*a, 0, 10);
    CHECK_THAT(u(*b, 0, 1), WithinAbs(0.1, 1e-12));
  }
}

TEST_CASE("integrator, lag and delay: no feedthrough, outputs() before step()") {
  const double dt = 0.1;
  // As a run calls one: read its outputs during the pass, then give it the step's input.
  auto run = [dt](Block& b, int steps, double u) {
    double y = 0;
    for (int k = 0; k < steps; ++k) {
      y = get_or(b.outputs(), "y");
      b.step({.t = k * dt, .dt = dt}, {{"u", u}});
    }
    return y;
  };
  auto integ = make("control/integrator", {{"initial", 1}});
  CHECK_THAT(run(*integ, 11, 2), WithinAbs(1 + 2 * 1.0, 1e-9));  // at t = 1: the state before that step's input

  auto lag = make("control/lag", {{"tau", 0.5}});
  CHECK_THAT(run(*lag, 100, 3), WithinAbs(3.0, 1e-6));

  auto none = make("control/lag", {{"tau", 0}});  // tau 0: a one-step delay
  CHECK(run(*none, 1, 4) == 0.0);
  CHECK(run(*none, 1, 4) == 4.0);

  const auto catalog = load_descriptor("domain = \"d\"\nimport = [\"control\"]\n", registry());
  for (const char* type : {"integrator", "lag", "delay"}) CHECK_FALSE(catalog.find(type)->def.feedthrough);
  CHECK(catalog.find("pid")->def.feedthrough);
}

TEST_CASE("the package works in Signal and Step flows; the dynamic blocks are Signal only") {
  const auto catalog = load_descriptor("domain = \"d\"\nimport = [\"control\"]\n", registry());
  for (const char* type : {"pid", "integrator", "lag", "delay"})
    CHECK(catalog.find(type)->def.flows == std::vector<std::string>{"signal", "step"});
  for (const char* type : {"mpc", "lp_step", "nmpc"})
    CHECK(catalog.find(type)->def.flows == std::vector<std::string>{"signal"});
}

TEST_CASE("rk4_step matches exp(-t)") {
  using control::State;
  State<1> x{1.0};
  const double dt = 0.01;
  for (int k = 0; k < 100; ++k) {
    x = control::rk4_step(x, k * dt, dt, [](double, const State<1>& s) { return State<1>{-s[0]}; });
  }
  CHECK_THAT(x[0], WithinAbs(std::exp(-1.0), 1e-8));
}

TEST_CASE("integrator and lag accept solver parameter and step accurately") {
  auto integ_rk4 = make("control/integrator", {{"initial", 0.0}, {"solver", "rk4"}});
  REQUIRE(integ_rk4 != nullptr);
  const double dt = 0.01;
  for (int k = 0; k < 100; ++k) {
    integ_rk4->step({.t = k * dt, .dt = dt}, {{"u", 5.0}});
  }
  CHECK_THAT(get_or(integ_rk4->outputs(), "y"), WithinAbs(5.0 * 1.0, 1e-9));

  auto lag_rk4 = make("control/lag", {{"tau", 1.0}, {"solver", "rk4"}});
  REQUIRE(lag_rk4 != nullptr);
  for (int k = 0; k < 200; ++k) {
    lag_rk4->step({.t = k * dt, .dt = dt}, {{"u", 2.0}});
  }
  CHECK_THAT(get_or(lag_rk4->outputs(), "y"), WithinAbs(2.0 * (1.0 - std::exp(-2.0)), 1e-3));
}
