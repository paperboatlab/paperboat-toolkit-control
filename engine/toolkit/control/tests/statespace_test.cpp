// The state-space loop: lqr, place, c2d, kalman_gain (design) and state_feedback, observer,
// kalman (runtime), against closed-form answers.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "control/riccati.hpp"
#include "paperboat/toolkit/control.hpp"
#include "paperboat/toolkit/registry.hpp"

using namespace paperboat::toolkit;
using namespace paperboat::toolkit::control;
using Catch::Matchers::WithinAbs;

namespace {

const Registry& registry() {
  static const Registry r = [] {
    Registry out = builtins();
    register_control(out);
    return out;
  }();
  return r;
}

std::unique_ptr<Block> make(const std::string& name, const Values& params) {
  const Template* t = registry().find(name);
  REQUIRE(t != nullptr);
  return t->make(params, t->config);
}

// A design block's text output, parsed back.
Matrix design(const std::string& name, const Values& params, const std::string& port) {
  auto block = make(name, params);
  return parse_matrix(std::get<std::string>(block->step(Context{}, {}).at(port)), port);
}

void check_matrix(const Matrix& got, std::size_t rows, std::size_t cols, const std::vector<double>& want, double tol = 1e-8) {
  REQUIRE(got.rows == rows);
  REQUIRE(got.cols == cols);
  for (std::size_t i = 0; i < want.size(); ++i) CHECK_THAT(got.values[i], WithinAbs(want[i], tol));
}

const double phi = (1 + std::sqrt(5.0)) / 2;

}  // namespace

TEST_CASE("lqr solves the continuous and discrete Riccati equations") {
  // Double integrator, Q = I, R = 1: K = [1, √3], P = [[√3, 1], [1, √3]].
  const Values di = {{"A", "0 1; 0 0"}, {"B", "0; 1"}, {"Q", "1"}, {"R", "1"}};
  check_matrix(design("control/lqr", di, "K"), 1, 2, {1, std::sqrt(3.0)});
  check_matrix(design("control/lqr", di, "P"), 2, 2, {std::sqrt(3.0), 1, 1, std::sqrt(3.0)});
  // Scalar x[k+1] = x + u, Q = R = 1: P = φ (golden ratio), K = P / (1 + P).
  const Values scalar = {{"A", "1"}, {"B", "1"}, {"Q", "1"}, {"R", "1"}, {"domain", "z"}};
  check_matrix(design("control/lqr", scalar, "P"), 1, 1, {phi});
  check_matrix(design("control/lqr", scalar, "K"), 1, 1, {phi / (1 + phi)});
  // An unstable plant: x' = x + u, Q = R = 1: P = 1 + √2.
  check_matrix(design("control/lqr", {{"A", "1"}, {"B", "1"}}, "P"), 1, 1, {1 + std::sqrt(2.0)});
  CHECK_THROWS(make("control/lqr", {{"A", "0 1; 0 0"}, {"B", "0; 1"}, {"Q", "1 0"}}));
}

TEST_CASE("place puts the poles by Ackermann") {
  // Double integrator, poles -1, -2: s² + 3s + 2, so K = [2, 3].
  check_matrix(design("control/place", {{"A", "0 1; 0 0"}, {"B", "0; 1"}, {"poles", "-1 -2"}}, "K"), 1, 2, {2, 3});
  check_matrix(design("control/place", {{"A", "0 1; 0 0"}, {"B", "0; 1"}, {"poles", "-1+1j -1-1j"}}, "K"), 1, 2, {2, 2});
  CHECK_THROWS(make("control/place", {{"A", "0 1; 0 0"}, {"B", "0; 1"}, {"poles", "-1"}}));
  CHECK_THROWS(make("control/place", {{"A", "1 0; 0 1"}, {"B", "1; 1"}, {"poles", "-1 -2"}}));  // not controllable
}

TEST_CASE("c2d by zoh and tustin; expm") {
  const Values di = {{"A", "0 1; 0 0"}, {"B", "0; 1"}, {"dt", 0.1}};
  check_matrix(design("control/c2d", di, "Ad"), 2, 2, {1, 0.1, 0, 1}, 1e-12);
  check_matrix(design("control/c2d", di, "Bd"), 2, 1, {0.005, 0.1}, 1e-12);
  const Values lag = {{"A", "-1"}, {"B", "1"}, {"dt", 0.1}, {"method", "tustin"}};
  check_matrix(design("control/c2d", lag, "Ad"), 1, 1, {0.95 / 1.05}, 1e-9);
  check_matrix(design("control/c2d", lag, "Bd"), 1, 1, {0.1 / 1.05}, 1e-9);
  // e^[[0 3], [-3 0]] is a rotation by 3 rad (needs scaling and squaring).
  check_matrix(expm(parse_matrix("0 3; -3 0", "A")), 2, 2, {std::cos(3.0), std::sin(3.0), -std::sin(3.0), std::cos(3.0)}, 1e-12);
}

TEST_CASE("kalman_gain is the dual of lqr") {
  // Scalar random walk, Q = R = 1: prior P = φ, filter gain L = P / (P + 1).
  check_matrix(design("control/kalman_gain", {{"A", "1"}, {"C", "1"}, {"domain", "z"}}, "L"), 1, 1, {phi / (1 + phi)});
  // Continuous: x' = w, y = x + v, Q = R = 1: P = 1, L = 1.
  check_matrix(design("control/kalman_gain", {{"A", "0"}, {"C", "1"}}, "L"), 1, 1, {1});
}

TEST_CASE("state_feedback takes K from its param or its K input") {
  auto fb = make("control/state_feedback", {{"K", "1 2"}});
  const auto u = [&](const Values& in) { return std::get<std::vector<double>>(fb->step(Context{}, in).at("u")); };
  CHECK(u({{"x", std::vector<double>{1, 1}}}) == std::vector<double>{-3});
  CHECK(u({{"x", std::vector<double>{1, 1}}, {"r", std::vector<double>{3}}}) == std::vector<double>{0});
  CHECK(u({{"x", std::vector<double>{1, 1}}, {"r", std::vector<double>{3}}, {"K", std::string("2 0")}}) == std::vector<double>{1});
  CHECK_THROWS(u({{"x", std::vector<double>{1, 1, 1}}}));
}

TEST_CASE("observer converges at the rate of A - L C") {
  // Plant x[k+1] = 0.9 x + 0.1 u from x = 5; the observer starts at 0 with L = 0.5: error × 0.4 a step.
  auto obs = make("control/observer", {{"A", "0.9"}, {"B", "0.1"}, {"C", "1"}, {"L", "0.5"}, {"domain", "z"}});
  double x = 5;
  for (int k = 0; k < 40; ++k) {
    CHECK(std::get<std::vector<double>>(obs->outputs().at("x_hat")).size() == 1);
    obs->step(Context{.t = k * 0.1, .dt = 0.1}, {{"u", std::vector<double>{1}}, {"y", std::vector<double>{x}}});
    x = 0.9 * x + 0.1;
  }
  CHECK_THAT(std::get<std::vector<double>>(obs->outputs().at("x_hat"))[0], WithinAbs(x, 1e-12));
  CHECK_FALSE(registry().find("control/observer")->def.feedthrough);
}

TEST_CASE("kalman with Q = 0 averages its measurements") {
  auto kf = make("control/kalman", {{"A", "1"}, {"B", "0"}, {"C", "1"}, {"Q", "0"}, {"R", "1"}, {"P0", 1e9}});
  Values out;
  for (int k = 0; k < 100; ++k)
    out = kf->step(Context{.t = k * 0.1, .dt = 0.1}, {{"y", std::vector<double>{3.0 + (k % 2 ? 1.0 : -1.0)}}});
  CHECK_THAT(std::get<std::vector<double>>(out.at("x_hat"))[0], WithinAbs(3.0, 1e-6));
  CHECK_THAT(get_or(out, "P_trace"), WithinAbs(0.01, 1e-6));
}
