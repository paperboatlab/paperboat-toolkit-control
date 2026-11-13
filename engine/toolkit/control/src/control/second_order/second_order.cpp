// control/second_order: k wn² / (s² + 2 zeta wn s + wn²), the standard second-order system.

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_second_order(Registry& registry) {
  add_designed(
      registry, "second_order", "Second order",
      {{.id = "wn", .default_value = 1.0, .unit = "rad/s"}, {.id = "zeta", .default_value = 0.7}, {.id = "gain", .default_value = 1.0}},
      "solver",
      [](const Values& p, const Config&) -> DesignFn {
        const double wn = get_or(p, "wn", 1.0), zeta = get_or(p, "zeta", 0.7), k = get_or(p, "gain", 1.0);
        if (wn <= 0) throw std::invalid_argument("wn: expected more than 0");
        if (zeta < 0) throw std::invalid_argument("zeta: expected 0 or more");
        return [=](double) { return Design{{k * wn * wn}, {1.0, 2.0 * zeta * wn, wn * wn}}; };
      },
      "A second-order system k wn² / (s² + 2 zeta wn s + wn²): natural frequency wn, damping zeta, dc gain k.");
}

}  // namespace paperboat::toolkit::control
