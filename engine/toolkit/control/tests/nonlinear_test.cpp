// The PID upgrade (filter, limits, anti-windup) and the nonlinear and discrete blocks.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "paperboat/toolkit/control.hpp"
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

template <class U>
std::vector<double> run(Block& block, int steps, double dt, U u, const std::string& port = "y") {
  std::vector<double> ys;
  for (int k = 0; k < steps; ++k) {
    Values in = u(k * dt, k);
    ys.push_back(get_or(block.step(Context{.t = k * dt, .dt = dt}, in), port));
  }
  return ys;
}

Values u_is(double v) { return {{"u", v}}; }

void check_series(const std::vector<double>& got, const std::vector<double>& want, double tol = 1e-12) {
  REQUIRE(got.size() == want.size());
  for (std::size_t i = 0; i < want.size(); ++i) CHECK_THAT(got[i], WithinAbs(want[i], tol));
}

// A PI loop on an integrator plant x' = u, the output limited to ±1, chasing r = 10 for 20 s,
// then r = 0. Returns x's peak after r drops: wind-up makes it overshoot far below 0.
double windup_undershoot(const std::string& anti_windup) {
  auto pid = make("control/pid", {{"kp", 1.0}, {"ki", 1.0}, {"u_min", -1.0}, {"u_max", 1.0}, {"anti_windup", anti_windup}});
  const double dt = 0.01;
  double x = 0, low = 0;
  for (int k = 0; k < 6000; ++k) {
    const double t = k * dt, r = t < 20 ? 10.0 : 0.0;
    const double u = get_or(pid->step(Context{.t = t, .dt = dt}, {{"e", r - x}}), "u");
    x += u * dt;
    if (t > 20) low = std::min(low, x);
  }
  return low;
}

}  // namespace

TEST_CASE("pid limits its output and keeps its integral from winding up") {
  auto limited = make("control/pid", {{"kp", 10.0}, {"u_min", -2.0}, {"u_max", 2.0}});
  CHECK(get_or(limited->step(Context{.t = 0, .dt = 0.1}, {{"e", 1.0}}), "u") == 2.0);
  CHECK(windup_undershoot("none") < -5.0);
  CHECK(windup_undershoot("clamp") > -1.0);
  CHECK(windup_undershoot("back_calc") > -1.0);
}

TEST_CASE("pid n filters the derivative") {
  // kd s / (s / n + 1) on a unit step in e: the kick decays with time constant 1/n.
  auto pid = make("control/pid", {{"kp", 0.0}, {"kd", 1.0}, {"n", 10.0}});
  const auto us = run(*pid, 40, 0.01, [](double, int k) { return Values{{"e", k == 0 ? 0.0 : 1.0}}; }, "u");
  CHECK(us[1] < 10.0);                    // unfiltered it would be 1 / dt = 100
  CHECK_THAT(us[1], WithinAbs(1.0 / (0.1 + 0.01), 1e-12));  // backward Euler: kd / (1/n + dt)
  CHECK(us.back() < 0.1 * us[1]);         // decays
  CHECK_THROWS(make("control/pid", {{"n", -1.0}}));
}

TEST_CASE("rate_limiter, backlash, quantizer, relay, friction") {
  auto rate = make("control/rate_limiter", {{"rise", 1.0}, {"fall", 2.0}});
  check_series(run(*rate, 4, 0.5, [](double, int k) { return u_is(k == 0 ? 0.0 : k < 3 ? 10.0 : -10.0); }), {0, 0.5, 1.0, 0.0});

  auto gap = make("control/backlash", {{"width", 1.0}});
  check_series(run(*gap, 5, 0.1, [](double, int k) { return u_is(std::vector<double>{0.3, 1.0, 0.8, 0.2, -1.0}[k]); }),
               {0, 0.5, 0.5, 0.5, -0.5});

  auto adc = make("control/quantizer", {{"interval", 0.25}});
  check_series(run(*adc, 3, 0.1, [](double, int k) { return u_is(std::vector<double>{0.1, 0.13, -0.4}[k]); }), {0, 0.25, -0.5});

  auto relay = make("control/relay", {{"on_at", 1.0}, {"off_at", -1.0}, {"on_value", 5.0}, {"off_value", -5.0}});
  check_series(run(*relay, 5, 0.1, [](double, int k) { return u_is(std::vector<double>{0, 1, 0, -1, 0}[k]); }), {-5, 5, 5, -5, -5});
  CHECK_THROWS(make("control/relay", {{"on_at", -1.0}, {"off_at", 1.0}}));

  auto f = make("control/friction", {{"coulomb", 1.0}, {"viscous", 0.5}, {"static", 2.0}, {"stribeck_v", 0.1}});
  check_series(run(*f, 3, 0.1, [](double, int k) { return u_is(std::vector<double>{0, 1e-9, -10}[k]); }), {0, 2.0, -6.0}, 1e-6);
}

TEST_CASE("discrete_integrator: methods, limits and reset") {
  auto fwd = make("control/discrete_integrator", {{"method", "forward"}});
  check_series(run(*fwd, 3, 0.5, [](double, int) { return u_is(2.0); }), {0, 1, 2});
  auto back = make("control/discrete_integrator", {{"method", "backward"}});
  check_series(run(*back, 3, 0.5, [](double, int) { return u_is(2.0); }), {1, 2, 3});
  auto trap = make("control/discrete_integrator", {{"method", "trapezoid"}});
  check_series(run(*trap, 3, 1.0, [](double, int k) { return u_is(k); }), {0, 0.5, 2});
  auto held = make("control/discrete_integrator", {{"method", "backward"}, {"upper", 1.5}, {"lower", -1.0}});
  check_series(run(*held, 4, 0.5, [](double, int k) { return Values{{"u", 2.0}, {"reset", k == 3 ? 1.0 : 0.0}}; }), {1, 1.5, 1.5, 0});
}

TEST_CASE("zoh, foh, discrete_derivative and tapped_delay sample every sample_time") {
  const auto ramp = [](double t, int) { return u_is(t); };
  auto zoh = make("control/zoh", {{"sample_time", 0.3}});
  check_series(run(*zoh, 7, 0.1, ramp), {0, 0, 0, 0.3, 0.3, 0.3, 0.6}, 1e-9);
  auto foh = make("control/foh", {{"sample_time", 0.3}});
  check_series(run(*foh, 7, 0.1, ramp), {0, 0, 0, 0.3, 0.4, 0.5, 0.6}, 1e-9);
  auto d = make("control/discrete_derivative", {});
  check_series(run(*d, 3, 0.1, [](double t, int) { return u_is(3 * t); }), {0, 3, 3}, 1e-9);
  auto taps = make("control/tapped_delay", {{"taps", 3.0}});
  run(*taps, 3, 0.1, [](double, int k) { return u_is(k + 1); });
  const auto last = taps->step(Context{.t = 0.3, .dt = 0.1}, u_is(4));
  CHECK(std::get<std::vector<double>>(last.at("y")) == std::vector<double>{4, 3, 2});
}
