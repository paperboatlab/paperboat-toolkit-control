// The shared LTI core (control/lti.hpp): Tustin against SciPy, sample-and-hold, and a
// steady start, through transfer_fn.

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "control/lti.hpp"
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

void check_close(const Coefficients& got, const std::vector<double>& want) {
  REQUIRE(got.size() == want.size());
  for (std::size_t i = 0; i < want.size(); ++i) CHECK_THAT(got[i], WithinAbs(want[i], 1e-12));
}

}  // namespace

TEST_CASE("bilinear matches scipy.signal.bilinear") {
  // scipy.signal.bilinear([1], [1, 1, 1], fs=10)
  Polynomials z = bilinear({1}, {1, 1, 1}, 0.1);
  check_close(z.num, {0.002375296912114, 0.004750593824228, 0.002375296912114});
  check_close(z.den, {1.0, -1.895486935866983, 0.904988123515439});

  // A 2nd-order Butterworth at 5 Hz, prewarped, 100 Hz: scipy.signal.butter(2, 5, fs=100)
  const double w = warp(2 * std::numbers::pi * 5, 0.01);
  z = bilinear({w * w}, {1, std::sqrt(2.0) * w, w * w}, 0.01);
  check_close(z.num, {0.020083365564211, 0.040166731128422, 0.020083365564211});
  check_close(z.den, {1.0, -1.561018075800718, 0.641351538057563});

  CHECK_THROWS(warp(2 * std::numbers::pi * 60, 0.01));  // 60 Hz is above a 100 Hz step's Nyquist
}

TEST_CASE("poly_from_roots expands conjugate pairs and refuses an unpaired one") {
  check_close(poly_from_roots({{-1, 2}, {-1, -2}}, "poles"), {1, 2, 5});
  CHECK_THROWS(poly_from_roots({{-1, 2}}, "poles"));
}

TEST_CASE("transfer_fn tustin reaches the dc gain") {
  auto lag = make("control/transfer_fn", {{"num", "2"}, {"den", "1 1"}, {"realize", "tustin"}});
  double y = 0;
  for (int k = 0; k < 2000; ++k) y = get_or(lag->step(Context{.t = k * 0.01, .dt = 0.01}, {{"u", 1.0}}), "y");
  CHECK_THAT(y, WithinAbs(2.0, 1e-6));
}

TEST_CASE("sample_time holds the output between samples") {
  // y[k] = u[k-1] at 0.1 s samples, in a 0.01 s run.
  auto delay = make("control/transfer_fn", {{"num", "1"}, {"den", "1 0"}, {"domain", "z"}, {"sample_time", 0.1}});
  std::vector<double> ys;
  for (int k = 0; k < 25; ++k) ys.push_back(get_or(delay->step(Context{.t = k * 0.01, .dt = 0.01}, {{"u", double(k)}}), "y"));
  CHECK(ys[0] == 0.0);
  CHECK(ys[9] == 0.0);   // held
  CHECK(ys[10] == 0.0);  // second sample: the input at t = 0
  CHECK(ys[19] == 0.0);
  CHECK(ys[20] == 10.0);  // third sample: the input at t = 0.1
  CHECK(ys[24] == 10.0);
}

TEST_CASE("start = steady begins at the steady state for the first input") {
  for (const char* realize : {"solver", "tustin"}) {
    auto lag = make("control/transfer_fn", {{"num", "2"}, {"den", "1 1"}, {"realize", realize}, {"start", "steady"}});
    for (int k = 0; k < 5; ++k)
      CHECK_THAT(get_or(lag->step(Context{.t = k * 0.01, .dt = 0.01}, {{"u", 3.0}}), "y"), WithinAbs(6.0, 1e-9));
  }
  auto z = make("control/transfer_fn", {{"num", "0.5"}, {"den", "1 -0.5"}, {"domain", "z"}, {"start", "steady"}});
  CHECK_THAT(get_or(z->step(Context{.t = 0, .dt = 0.01}, {{"u", 4.0}}), "y"), WithinAbs(4.0, 1e-12));
}
