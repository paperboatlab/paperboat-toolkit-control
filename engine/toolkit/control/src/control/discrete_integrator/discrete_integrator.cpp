// control/discrete_integrator: an accumulator updated every step (or sample_time), forward,
// backward or trapezoidal, with optional limits and a reset input that returns it to initial.

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/matrix.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class DiscreteIntegrator : public Block {
 public:
  DiscreteIntegrator(std::string method, double period, double lower, double upper, double initial)
      : method_(std::move(method)), sampler_{period}, lower_(lower), upper_(upper), initial_(initial), y_(initial), out_(initial) {}

  Values step(const Context& ctx, const Values& in) override {
    const double u = get_or(in, "u");
    if (get_or(in, "reset") > 0.5) {
      y_ = out_ = initial_, prev_u_ = u;
      return {{"y", out_}};
    }
    if (!sampler_.due(ctx.t)) return {{"y", out_}};  // hold between samples
    const double T = sampler_.period > 0 ? sampler_.period : ctx.dt;
    if (method_ == "forward") out_ = y_, y_ += T * u;  // gives the sum before this sample's input
    else if (method_ == "backward") y_ += T * u, out_ = y_;
    else y_ += T * (u + prev_u_) / 2, out_ = y_;
    prev_u_ = u;
    if (lower_ < upper_) y_ = std::clamp(y_, lower_, upper_), out_ = std::clamp(out_, lower_, upper_);
    return {{"y", out_}};
  }

 private:
  std::string method_;
  Sampler sampler_;
  double lower_, upper_, initial_, y_, out_, prev_u_ = 0.0;
};

}  // namespace

void add_discrete_integrator(Registry& registry) {
  add_signal(registry, make_template(
      "control", "discrete_integrator", "Discrete integrator", {"u", "reset"}, {"y"},
      {{.id = "method", .type = "enum", .default_text = "forward", .options = {"forward", "backward", "trapezoid"}},
       {.id = "sample_time", .default_value = 0.0, .unit = "s"},
       {.id = "lower", .default_value = 0.0}, {.id = "upper", .default_value = 0.0}, {.id = "initial", .default_value = 0.0}},
      [](const Values& p, const Config& c) -> std::unique_ptr<Block> {
        if (get_or(p, "sample_time") < 0) throw std::invalid_argument("sample_time: expected 0 or more");
        return std::make_unique<DiscreteIntegrator>(
            enum_text(p, c, "method", {"forward", "backward", "trapezoid"}, "forward"), get_or(p, "sample_time"),
            get_or(p, "lower"), get_or(p, "upper"), get_or(p, "initial"));
      },
      "Sums its input times the step (or sample_time): forward, backward or trapezoid. lower < upper "
      "limits it (equal: none); reset > 0.5 returns it to initial."));
}

}  // namespace paperboat::toolkit::control
