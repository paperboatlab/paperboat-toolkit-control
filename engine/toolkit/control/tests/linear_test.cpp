// Linear and filter blocks: filter against SciPy, the cutoff at three steps, and each small
// block's defining response.

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "paperboat/toolkit/control.hpp"
#include "paperboat/toolkit/registry.hpp"

using namespace paperboat::toolkit;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using std::numbers::pi;

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

// The block's output for u(k) at steps of dt.
template <class U>
std::vector<double> run(Block& block, int steps, double dt, U u) {
  std::vector<double> ys;
  for (int k = 0; k < steps; ++k) ys.push_back(get_or(block.step(Context{.t = k * dt, .dt = dt}, {{"u", u(k * dt, k)}}), "y"));
  return ys;
}

std::vector<double> impulse(Block& block, int steps, double dt) {
  return run(block, steps, dt, [](double, int k) { return k == 0 ? 1.0 : 0.0; });
}

void check_series(const std::vector<double>& got, const std::vector<double>& want, double tol) {
  REQUIRE(got.size() >= want.size());
  for (std::size_t i = 0; i < want.size(); ++i) CHECK_THAT(got[i], WithinAbs(want[i], tol));
}

// The amplitude of a sine at f through the block: RMS · √2 over its last 5 of 20 periods
// (whole periods, so it is exact even at a few samples a period).
double gain_at(Block& block, double f, double dt) {
  const int steps = static_cast<int>(std::round(20.0 / f / dt));
  const auto ys = run(block, steps, dt, [&](double t, int) { return std::sin(2 * pi * f * t); });
  double sum = 0;
  const std::size_t from = ys.size() * 3 / 4;
  for (std::size_t i = from; i < ys.size(); ++i) sum += ys[i] * ys[i];
  return std::sqrt(2.0 * sum / static_cast<double>(ys.size() - from));
}

}  // namespace

TEST_CASE("filter matches scipy.signal at fs = 1000") {
  const double dt = 0.001;
  SECTION("butter(4, 10)") {
    auto f = make("control/filter", {{"family", "butterworth"}, {"order", 4.0}, {"cutoff", 10.0}});
    check_series(impulse(*f, 12, dt),
                 {8.984861463970648e-07, 7.0403806938559324e-06, 2.7436209448736596e-05, 7.314062150635031e-05,
                  0.00015409781734870002, 0.00027826885576823473, 0.0004518084071333337, 0.000679232355184714,
                  0.0009635764908146834, 0.0013065465412015176, 0.0017086597775482137, 0.002169378444495709},
                 1e-12);
  }
  SECTION("cheby1(3, 1, 50, highpass)") {
    auto f = make("control/filter", {{"family", "chebyshev1"}, {"kind", "highpass"}, {"order", 3.0}, {"cutoff", 50.0}, {"ripple", 1.0}});
    check_series(impulse(*f, 12, dt),
                 {0.6859738865629318, -0.4935414510121874, -0.2791000740062166, -0.14819323076006297, -0.0627850353826912,
                  -0.004594671692541102, 0.0350253755838878, 0.06029121953415503, 0.07361071484647322, 0.076844140189894,
                  0.07184150026802641, 0.06059866580998152},
                 1e-9);
  }
  SECTION("cheby2(4, 40, [50, 150], bandpass)") {
    auto f = make("control/filter", {{"family", "chebyshev2"}, {"kind", "bandpass"}, {"order", 4.0}, {"cutoff", 50.0},
                                     {"cutoff2", 150.0}, {"attenuation", 40.0}});
    check_series(impulse(*f, 12, dt),
                 {0.012483820797182145, 0.013238972601890012, 0.019696962991252676, 0.009827091967690077,
                  -0.009556388722601089, -0.03190188329818238, -0.04873610975231504, -0.050456724977525624,
                  -0.030773406996337904, 0.00814691906991194, 0.05391618866139313, 0.08777764208084646},
                 1e-9);
  }
  SECTION("bessel(3, 20, norm = mag)") {
    auto f = make("control/filter", {{"family", "bessel"}, {"order", 3.0}, {"cutoff", 20.0}});
    check_series(impulse(*f, 12, dt),
                 {0.0005589366373071465, 0.0031222485276475613, 0.008498415418161495, 0.01590237851437746,
                  0.024109294729911804, 0.03223718925695375, 0.03968059141488618, 0.04605342754454306,
                  0.05114039144121517, 0.054855992607330095, 0.057210483229225575, 0.05828188249886168},
                 1e-9);
  }
  SECTION("butter(2, [40, 60], bandstop)") {
    auto f = make("control/filter", {{"kind", "bandstop"}, {"order", 2.0}, {"cutoff", 40.0}, {"cutoff2", 60.0}});
    check_series(impulse(*f, 12, dt),
                 {0.9149691441130827, -0.15459369148569557, -0.11832379689900507, -0.07397913545354973,
                  -0.026779752898448916, 0.0182693827219802, 0.05688993463406315, 0.08591134164723128,
                  0.10349223974014193, 0.10917384817442344, 0.10377685766253769, 0.08916987602183263},
                 1e-9);
  }
}

TEST_CASE("filter is -3 dB at its cutoff whatever the step") {
  for (double dt : {0.0005, 0.002, 0.01}) {
    auto f = make("control/filter", {{"order", 4.0}, {"cutoff", 20.0}});
    CHECK_THAT(gain_at(*f, 20.0, dt), WithinAbs(std::sqrt(0.5), 0.01));
  }
  auto f = make("control/filter", {{"cutoff", 80.0}});
  CHECK_THROWS(run(*f, 1, 0.01, [](double, int) { return 0.0; }));  // 80 Hz is above 50 Hz Nyquist
  CHECK_THROWS(make("control/filter", {{"order", 11.0}}));
  CHECK_THROWS(make("control/filter", {{"kind", "bandpass"}, {"cutoff", 20.0}, {"cutoff2", 10.0}}));
}

TEST_CASE("second_order overshoots by exp(-pi zeta / sqrt(1 - zeta^2))") {
  auto plant = make("control/second_order", {{"wn", 2.0}, {"zeta", 0.5}, {"gain", 3.0}, {"solver", "rk4"}});
  const auto ys = run(*plant, 10000, 0.001, [](double, int) { return 1.0; });
  const double peak = *std::max_element(ys.begin(), ys.end());
  CHECK_THAT(ys.back(), WithinAbs(3.0, 1e-4));
  CHECK_THAT(peak / 3.0 - 1.0, WithinAbs(std::exp(-pi * 0.5 / std::sqrt(0.75)), 1e-3));
}

TEST_CASE("zpk builds k (s - z) / (s - p) from real and complex roots") {
  auto real = make("control/zpk", {{"poles", "-1 -2"}, {"gain", 2.0}, {"solver", "rk4"}});
  CHECK_THAT(run(*real, 3000, 0.01, [](double, int) { return 1.0; }).back(), WithinAbs(1.0, 1e-6));
  auto pair = make("control/zpk", {{"poles", "[-1+2j, -1-2j]"}, {"gain", 5.0}, {"solver", "rk4"}});
  CHECK_THAT(run(*pair, 3000, 0.01, [](double, int) { return 1.0; }).back(), WithinAbs(1.0, 1e-6));
  CHECK_THROWS(make("control/zpk", {{"poles", "-1+2j"}}));
  CHECK_THROWS(make("control/zpk", {{"poles", "-1+x"}}));
}

TEST_CASE("lead_lag, derivative and washout have their dc behaviour") {
  auto lead = make("control/lead_lag", {{"gain", 4.0}, {"zero", 1.0}, {"pole", 10.0}});
  CHECK_THAT(run(*lead, 2000, 0.01, [](double, int) { return 1.0; }).back(), WithinAbs(4.0, 1e-6));
  auto d = make("control/derivative", {{"tau", 0.01}});
  CHECK_THAT(run(*d, 1000, 0.001, [](double t, int) { return 3.0 * t; }).back(), WithinAbs(3.0, 1e-6));
  auto w = make("control/washout", {{"tau", 0.1}});
  CHECK_THAT(run(*w, 1000, 0.01, [](double, int) { return 7.0; }).back(), WithinAbs(0.0, 1e-6));
  CHECK_THROWS(make("control/derivative", {{"tau", 0.0}}));
}

TEST_CASE("notch cuts depth dB at its frequency and passes the rest") {
  auto n = make("control/notch", {{"freq", 50.0}, {"depth", 40.0}});
  CHECK_THAT(gain_at(*n, 50.0, 0.0005), WithinAbs(0.01, 0.003));
  auto pass = make("control/notch", {{"freq", 50.0}, {"depth", 40.0}});
  CHECK_THAT(gain_at(*pass, 5.0, 0.0005), WithinAbs(1.0, 0.02));
}

TEST_CASE("biquad lowpass gives the cookbook coefficients") {
  const double f = 100, q = 0.9, fs = 1000;
  const double w0 = 2 * pi * f / fs, alpha = std::sin(w0) / (2 * q), cw = std::cos(w0);
  const double a0 = 1 + alpha;
  const std::vector<double> b = {(1 - cw) / 2 / a0, (1 - cw) / a0, (1 - cw) / 2 / a0}, a = {1, -2 * cw / a0, (1 - alpha) / a0};
  // Its impulse response, from the difference equation.
  std::vector<double> want;
  double y1 = 0, y2 = 0;
  for (int k = 0; k < 8; ++k) {
    const double x0 = k == 0, x1 = k == 1, x2 = k == 2;
    const double y = b[0] * x0 + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2;
    want.push_back(y);
    y2 = y1, y1 = y;
  }
  auto bq = make("control/biquad", {{"kind", "lowpass"}, {"freq", f}, {"q", q}});
  check_series(impulse(*bq, 8, 1 / fs), want, 1e-12);
  auto peak = make("control/biquad", {{"kind", "peak"}, {"freq", 100.0}, {"q", 2.0}, {"gain_db", 6.0}});
  CHECK_THAT(gain_at(*peak, 100.0, 0.0002), WithinRel(std::pow(10.0, 6.0 / 20.0), 0.01));
}

TEST_CASE("moving_average, ema and median") {
  auto avg = make("control/moving_average", {{"window", 4.0}});
  check_series(run(*avg, 6, 0.01, [](double, int) { return 1.0; }), {0.25, 0.5, 0.75, 1, 1, 1}, 1e-12);
  auto ema = make("control/ema", {{"alpha", 0.5}});
  check_series(run(*ema, 3, 0.01, [](double, int) { return 1.0; }), {0.5, 0.75, 0.875}, 1e-12);
  auto steady = make("control/ema", {{"alpha", 0.5}, {"start", "steady"}});
  check_series(run(*steady, 2, 0.01, [](double, int) { return 4.0; }), {4.0, 4.0}, 1e-12);
  auto med = make("control/median", {{"window", 3.0}});
  const std::vector<double> spiky = {0, 1, 2, 50, 4, 5, 6};
  check_series(run(*med, 7, 0.01, [&](double, int k) { return spiky[k]; }), {0, 1, 1, 2, 4, 5, 5}, 1e-12);
  CHECK_THROWS(make("control/median", {{"window", 4.0}}));
}
