// control/derivative: a filtered derivative, s / (tau s + 1).

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_derivative(Registry& registry) {
  add_designed(
      registry, "derivative", "Derivative",
      {{.id = "tau", .default_value = 0.01, .unit = "s"}},
      "tustin",
      [](const Values& p, const Config&) -> DesignFn {
        const double tau = get_or(p, "tau", 0.01);
        if (tau <= 0) throw std::invalid_argument("tau: expected more than 0 (a derivative needs a filter)");
        return [=](double) { return Design{{1.0, 0.0}, {tau, 1.0}}; };
      },
      "The rate of change of its input, filtered: s / (tau s + 1).");
}

}  // namespace paperboat::toolkit::control
