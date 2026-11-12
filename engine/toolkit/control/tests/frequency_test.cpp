// Frequency-domain and state-space blocks: transfer_fn, state_space, bode, controllability
// and observability, plus the coefficient and matrix text parsers they share.

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "control/matrix.hpp"
#include "paperboat/toolkit/control.hpp"
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

double out(Block& block, double t, const Values& in = {}, const std::string& port = "y", double dt = 0.01) {
  return get_or(block.step(Context{.t = t, .dt = dt}, in), port);
}

template <class Fn>
std::string error_of(Fn fn) {
  try {
    fn();
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return {};
}

}  // namespace

TEST_CASE("frequency and state-space blocks are registered as Signal blocks") {
  for (const char* name : {"control/transfer_fn", "control/state_space", "control/bode", "control/controllability",
                           "control/observability"}) {
    const Template* t = registry().find(name);
    REQUIRE(t != nullptr);
    CHECK(std::string(name).ends_with("/" + t->def.type));
    CHECK(t->def.flows == std::vector<std::string>{"signal"});
  }

  const auto catalog = load_descriptor("domain = \"d\"\nimport = [\"control\"]\n", registry());
  const auto* state_space = catalog.find("state_space");
  REQUIRE(state_space != nullptr);
  REQUIRE(state_space->def.inputs.size() == 1);
  CHECK(state_space->def.inputs[0].id == "u");
  CHECK(state_space->def.inputs[0].type == "numbers");
  CHECK(state_space->def.outputs[0].type == "numbers");
  CHECK(catalog.find("transfer_fn")->def.feedthrough);
  CHECK(catalog.find("state_space")->def.feedthrough);
  CHECK(catalog.find("controllability")->def.inputs.empty());
  CHECK(catalog.find("observability")->def.inputs.empty());
}

TEST_CASE("transfer_fn s step response reaches the dc gain") {
  auto transfer = make("control/transfer_fn",
                       {{"num", "1"}, {"den", "1 1"}, {"domain", "s"}, {"solver", "rk4"}});
  REQUIRE(transfer != nullptr);
  const double dt = 0.01;
  double y = 0.0;
  for (int k = 0; k < 1000; ++k) y = out(*transfer, k * dt, {{"u", 1.0}}, "y", dt);
  CHECK_THAT(y, WithinAbs(1.0, 1e-3));

  auto gain = make("control/transfer_fn", {{"num", "[5]"}, {"den", "1"}, {"domain", "s"}});
  REQUIRE(gain != nullptr);
  CHECK_THAT(out(*gain, 0.0, {{"u", 2.0}}), WithinAbs(10.0, 1e-12));
}

TEST_CASE("transfer_fn z runs direct form II transposed") {
  // H(z) = 1/(z - 0.5): y[0] = 0, y[1] = 1, y[2] = 1.5, ...
  auto transfer = make("control/transfer_fn", {{"num", "1"}, {"den", "1 -0.5"}, {"domain", "z"}});
  REQUIRE(transfer != nullptr);
  CHECK_THAT(out(*transfer, 0.0, {{"u", 1.0}}), WithinAbs(0.0, 1e-12));
  CHECK_THAT(out(*transfer, 1.0, {{"u", 1.0}}), WithinAbs(1.0, 1e-12));
  CHECK_THAT(out(*transfer, 2.0, {{"u", 1.0}}), WithinAbs(1.5, 1e-12));
  CHECK_THAT(out(*transfer, 3.0, {{"u", 1.0}}), WithinAbs(1.75, 1e-12));
}

TEST_CASE("state_space s integrates x' = A x + B u") {
  auto model = make("control/state_space",
                    {{"A", "0"}, {"B", "1"}, {"C", "1"}, {"D", "0"}, {"domain", "s"}, {"solver", "rk4"}});
  REQUIRE(model != nullptr);
  const double dt = 0.01;
  Values last;
  for (int k = 0; k < 100; ++k) last = model->step(Context{.t = k * dt, .dt = dt}, {{"u", Vector{1.0}}});

  const Vector* y = get_vector(last, "y");
  const Vector* x = get_vector(last, "x");
  REQUIRE(y != nullptr);
  REQUIRE(x != nullptr);
  REQUIRE(y->size() == 1);
  REQUIRE(x->size() == 1);
  CHECK_THAT((*y)[0], WithinAbs(1.0, 1e-3));
  CHECK_THAT((*x)[0], WithinAbs(1.0, 1e-3));
}

TEST_CASE("state_space z updates x[k+1] = A x[k] + B u[k]") {
  auto model = make("control/state_space", {{"A", "1"}, {"B", "1"}, {"C", "1"}, {"D", "0"}, {"domain", "z"}});
  REQUIRE(model != nullptr);
  for (int k = 0; k < 4; ++k) {
    const Values values = model->step(Context{.t = 0.0, .dt = 1.0}, {{"u", Vector{1.0}}});
    const Vector* y = get_vector(values, "y");
    const Vector* x = get_vector(values, "x");
    REQUIRE(y != nullptr);
    REQUIRE(x != nullptr);
    CHECK_THAT((*y)[0], WithinAbs(k, 1e-12));
    CHECK_THAT((*x)[0], WithinAbs(k, 1e-12));
  }
}

TEST_CASE("state_space takes a 2x2 model and a zero D") {
  auto model = make("control/state_space", {{"A", "[[0, 1], [0, 0]]"},
                                            {"B", "[[0], [1]]"},
                                            {"C", "1 0"},
                                            {"D", "0"},
                                            {"domain", "s"},
                                            {"solver", "rk4"}});
  REQUIRE(model != nullptr);
  const Values values = model->step(Context{.t = 0.0, .dt = 0.1}, {{"u", Vector{1.0}}});
  const Vector* y = get_vector(values, "y");
  const Vector* x = get_vector(values, "x");
  REQUIRE(y != nullptr);
  REQUIRE(x != nullptr);
  REQUIRE(y->size() == 1);
  REQUIRE(x->size() == 2);
  CHECK_THAT((*x)[0], WithinAbs(0.005, 1e-9));
  CHECK_THAT((*x)[1], WithinAbs(0.1, 1e-9));
  CHECK_THAT((*y)[0], WithinAbs(0.005, 1e-9));
}

TEST_CASE("bode gives -3.0103 dB and -45 degrees at the corner") {
  auto response = make("control/bode", {{"num", "1"}, {"den", "1 1"}, {"domain", "s"}});
  REQUIRE(response != nullptr);
  const Values values = response->step(Context{}, {{"w", Vector{1.0}}});
  const Vector* magnitude = get_vector(values, "magnitude");
  const Vector* phase = get_vector(values, "phase");
  REQUIRE(magnitude != nullptr);
  REQUIRE(phase != nullptr);
  REQUIRE(magnitude->size() == 1);
  REQUIRE(phase->size() == 1);
  CHECK_THAT((*magnitude)[0], WithinAbs(-3.010299956639812, 1e-9));
  CHECK_THAT((*phase)[0], WithinAbs(-45.0, 1e-9));
}

TEST_CASE("bode z uses z = exp(j w sample_time)") {
  auto response = make("control/bode", {{"num", "1 0"}, {"den", "1"}, {"domain", "z"}, {"sample_time", 0.5}});
  REQUIRE(response != nullptr);
  const Values values = response->step(Context{}, {{"w", Vector{1.0}}});
  const Vector* magnitude = get_vector(values, "magnitude");
  const Vector* phase = get_vector(values, "phase");
  REQUIRE(magnitude != nullptr);
  REQUIRE(phase != nullptr);
  CHECK_THAT((*magnitude)[0], WithinAbs(0.0, 1e-12));
  CHECK_THAT((*phase)[0], WithinAbs(0.5 * 180.0 / std::numbers::pi, 1e-9));
}

TEST_CASE("controllability and observability rank a 2-state system") {
  auto controllable = make("control/controllability", {{"A", "0 1; 0 0"}, {"B", "0; 1"}});
  REQUIRE(controllable != nullptr);
  Values values = controllable->step(Context{}, {});
  CHECK(get_or(values, "rank") == 2.0);
  CHECK(get_or(values, "order") == 2.0);
  CHECK(get_or(values, "controllable") == 1.0);

  auto not_controllable = make("control/controllability", {{"A", "0 1; 0 0"}, {"B", "1; 0"}});
  values = not_controllable->step(Context{}, {});
  CHECK(get_or(values, "rank") == 1.0);
  CHECK(get_or(values, "order") == 2.0);
  CHECK(get_or(values, "controllable") == 0.0);

  auto observable = make("control/observability", {{"A", "0 1; 0 0"}, {"C", "1 0"}});
  REQUIRE(observable != nullptr);
  values = observable->step(Context{}, {});
  CHECK(get_or(values, "rank") == 2.0);
  CHECK(get_or(values, "order") == 2.0);
  CHECK(get_or(values, "observable") == 1.0);

  auto not_observable = make("control/observability", {{"A", "0 1; 0 0"}, {"C", "0 1"}});
  values = not_observable->step(Context{}, {});
  CHECK(get_or(values, "rank") == 1.0);
  CHECK(get_or(values, "observable") == 0.0);
}

TEST_CASE("coefficient and matrix text parse in MATLAB style") {
  CHECK(control::parse_coefficients("1 2 1", "num") == control::Coefficients{1.0, 2.0, 1.0});
  CHECK(control::parse_coefficients("[1, 2, 1]", "num") == control::Coefficients{1.0, 2.0, 1.0});
  CHECK(control::parse_coefficients("[1 2 1]", "num") == control::Coefficients{1.0, 2.0, 1.0});

  const control::Matrix matrix = control::parse_matrix("[[1, 2], [3, 4]]", "A");
  CHECK(matrix.rows == 2);
  CHECK(matrix.cols == 2);
  CHECK(matrix(0, 0) == 1.0);
  CHECK(matrix(0, 1) == 2.0);
  CHECK(matrix(1, 0) == 3.0);
  CHECK(matrix(1, 1) == 4.0);
  CHECK(control::parse_matrix("1 2; 3 4", "A").values == matrix.values);
}

TEST_CASE("bad coefficient, matrix and dimension text names its parameter") {
  CHECK(error_of([] { make("control/transfer_fn", {{"num", "1 x 2"}}); }).find("num") != std::string::npos);
  CHECK(error_of([] { make("control/transfer_fn", {{"den", "[1, 2"}}); }).find("den") != std::string::npos);
  CHECK(error_of([] { make("control/transfer_fn", {{"den", "1 2; 3 4"}}); }).find("den") != std::string::npos);
  CHECK(error_of([] { make("control/transfer_fn", {{"den", "0 1"}}); }).find("den") != std::string::npos);
  CHECK(error_of([] { make("control/transfer_fn", {{"num", "1 0 0"}, {"den", "1 1"}}); }).find("num") !=
        std::string::npos);

  CHECK(error_of([] {
          make("control/state_space", {{"A", "1 0; 0 1"}, {"B", "1 2"}, {"C", "1 0"}});
        }).find("B") != std::string::npos);
  CHECK(error_of([] {
          make("control/state_space", {{"A", "1 0; 0 1"}, {"B", "1; 0"}, {"C", "1 2 3"}});
        }).find("C") != std::string::npos);
  CHECK(error_of([] {
          make("control/state_space", {{"A", "1 0; 0 1"}, {"B", "1; 0"}, {"C", "1 0"}, {"D", "1 2"}});
        }).find("D") != std::string::npos);
  CHECK(error_of([] {
          make("control/state_space", {{"A", "1 2 3; 4 5 6"}, {"B", "1"}, {"C", "1"}});
        }).find("A") != std::string::npos);

  CHECK(error_of([] {
          make("control/controllability", {{"A", "1"}, {"B", "1; 0"}});
        }).find("B") != std::string::npos);

  CHECK(error_of([] { control::parse_matrix("1 2; 3", "A"); }).find("row 1") != std::string::npos);
  CHECK(error_of([] { control::parse_coefficients("[1, 2; 3, 4]", "num"); }).find("num") != std::string::npos);
}
