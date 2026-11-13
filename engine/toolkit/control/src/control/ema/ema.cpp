// control/ema: an exponential moving average, y += alpha (u − y), each step or sample.

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_ema(Registry& registry) {
  add_designed(
      registry, "ema", "Exponential average",
      {{.id = "alpha", .default_value = 0.1}},
      "",
      [](const Values& p, const Config&) -> DesignFn {
        const double alpha = get_or(p, "alpha", 0.1);
        if (alpha <= 0 || alpha > 1) throw std::invalid_argument("alpha: expected more than 0 and at most 1");
        return [=](double) { return Design{{alpha, 0.0}, {1.0, alpha - 1.0}, "z"}; };
      },
      "An exponential moving average: y += alpha (u − y) every step (or sample_time). Smaller alpha, smoother.");
}

}  // namespace paperboat::toolkit::control
