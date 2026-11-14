// control/discrete_derivative: (u[n] − u[n−1]) / T every step (or sample_time); 0 at first.

#include <memory>
#include <optional>
#include <stdexcept>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class DiscreteDerivative : public Block {
 public:
  explicit DiscreteDerivative(double period) : sampler_{period} {}
  Values step(const Context& ctx, const Values& in) override {
    if (!sampler_.due(ctx.t)) return {{"y", y_}};
    const double u = get_or(in, "u"), T = sampler_.period > 0 ? sampler_.period : ctx.dt;
    y_ = prev_ && T > 0 ? (u - *prev_) / T : 0.0;
    prev_ = u;
    return {{"y", y_}};
  }

 private:
  Sampler sampler_;
  std::optional<double> prev_;
  double y_ = 0.0;
};

}  // namespace

void add_discrete_derivative(Registry& registry) {
  add_signal(registry, make_template(
      "control", "discrete_derivative", "Discrete derivative", {"u"}, {"y"},
      {{.id = "sample_time", .default_value = 0.0, .unit = "s"}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        if (get_or(p, "sample_time") < 0) throw std::invalid_argument("sample_time: expected 0 or more");
        return std::make_unique<DiscreteDerivative>(get_or(p, "sample_time"));
      },
      "The difference of its last two samples over the step (or sample_time); unfiltered, see derivative."));
}

}  // namespace paperboat::toolkit::control
