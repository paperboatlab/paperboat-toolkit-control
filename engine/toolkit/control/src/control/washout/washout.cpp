// control/washout: a first-order highpass, tau s / (tau s + 1); it removes the dc part.

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_washout(Registry& registry) {
  add_designed(
      registry, "washout", "Washout",
      {{.id = "tau", .default_value = 1.0, .unit = "s"}},
      "tustin",
      [](const Values& p, const Config&) -> DesignFn {
        const double tau = get_or(p, "tau", 1.0);
        if (tau <= 0) throw std::invalid_argument("tau: expected more than 0");
        return [=](double) { return Design{{tau, 0.0}, {tau, 1.0}}; };
      },
      "Removes the steady part of its input, passing changes: tau s / (tau s + 1).");
}

}  // namespace paperboat::toolkit::control
