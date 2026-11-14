// PID controller. Each instance keeps its own integral and previous error.

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "common.hpp"
#include "control/matrix.hpp"

namespace paperboat::toolkit::control {
namespace {

// u = kp e + I + D, with I = Σ ki e dt (forward, so a ki change is bumpless) and D the
// derivative through a first-order filter of bandwidth n rad/s (n = 0: none, a plain backward
// difference). With limits (u_min < u_max), u is clamped and the integral is kept from winding
// up: `clamp` stops integrating while saturated the same way; `back_calc` bleeds I by kt (u − u_raw).
class Pid : public Block {
 public:
  struct Options {
    double n = 0.0, u_min = 0.0, u_max = 0.0, kt = 1.0;
    std::string anti_windup = "clamp";
  };
  Pid(double kp, double ki, double kd, Options o) : kp_(kp), ki_(ki), kd_(kd), o_(std::move(o)) {}

  Values step(const Context& ctx, const Values& in) override {
    const double e = get_or(in, "e");
    const double i_before = i_;
    i_ += ki_ * e * ctx.dt;  // forward Euler
    // No previous error on the first step, so no derivative kick.
    if (prev_ && ctx.dt > 0) {
      const double tf = o_.n > 0 ? 1.0 / o_.n : 0.0;  // backward Euler of kd s / (tf s + 1)
      d_ = (tf * d_ + kd_ * (e - *prev_)) / (tf + ctx.dt);
    }
    prev_ = e;
    const double raw = kp_ * e + i_ + d_;
    if (o_.u_min >= o_.u_max) return {{"u", raw}};  // no limits
    const double u = std::clamp(raw, o_.u_min, o_.u_max);
    if (u != raw) {
      if (o_.anti_windup == "clamp" && (raw - u) * e > 0) i_ = i_before;  // e drives it further in
      else if (o_.anti_windup == "back_calc") i_ += o_.kt * (u - raw) * ctx.dt;
    }
    return {{"u", u}};
  }

  // kp + ki/s + kd n s / (s + n), unsaturated: x1 = ∫e, x2' = −n x2 + e (the derivative filter).
  std::optional<LinearModel> linearize(const Values&) const override {
    if (kd_ != 0 && o_.n <= 0) throw std::invalid_argument("kd needs a derivative filter (n > 0) to be analysed: kd·s alone has no finite gain at high frequency");
    LinearModel m{.inputs = {"e"}, .outputs = {"u"}};
    if (kd_ == 0) {
      m.states = 1, m.A = {0}, m.B = {1}, m.C = {ki_}, m.D = {kp_};
    } else {
      m.states = 2, m.A = {0, 0, 0, -o_.n}, m.B = {1, 1}, m.C = {ki_, -kd_ * o_.n * o_.n}, m.D = {kp_ + kd_ * o_.n};
    }
    return m;
  }

  // Tunable gains (#75): the integral and the previous error carry on.
  bool set_param(std::string_view id, double value) override {
    double* gain = id == "kp" ? &kp_ : id == "ki" ? &ki_ : id == "kd" ? &kd_ : nullptr;
    if (gain) *gain = value;
    return gain != nullptr;
  }

 private:
  double kp_, ki_, kd_;
  Options o_;
  double i_ = 0.0, d_ = 0.0;
  std::optional<double> prev_;
};

}  // namespace

void add_pid(Registry& registry) {
  auto pid = make_template(
      "control", "pid", "PID", {"e"}, {"u"},
      {{.id = "kp", .default_value = 1.0},
       {.id = "ki", .default_value = 0.0},
       {.id = "kd", .default_value = 0.0},
       {.id = "n", .default_value = 0.0, .unit = "rad/s"},
       {.id = "u_min", .default_value = 0.0},
       {.id = "u_max", .default_value = 0.0},
       {.id = "anti_windup", .type = "enum", .default_text = "clamp", .options = {"none", "clamp", "back_calc"}},
       {.id = "kt", .default_value = 1.0}},
      [](const Values& p, const Config& c) {
        Pid::Options o{.n = get_or(p, "n"), .u_min = get_or(p, "u_min"), .u_max = get_or(p, "u_max"), .kt = get_or(p, "kt", 1.0),
                       .anti_windup = enum_text(p, c, "anti_windup", {"none", "clamp", "back_calc"}, "clamp")};
        if (o.n < 0) throw std::invalid_argument("n: expected 0 (no filter) or more");
        return std::make_unique<Pid>(get_or(p, "kp"), get_or(p, "ki"), get_or(p, "kd"), std::move(o));
      },
      "Drives an error to zero: proportional, integral and derivative terms. n filters the derivative; "
      "u_min < u_max limits the output, with anti-windup (clamp or back_calc with kt). Equal limits: none.");
  pid.def.elementwise = true;  // a vector error: one controller per element
  registry.add(std::move(pid));
}

}  // namespace paperboat::toolkit::control
