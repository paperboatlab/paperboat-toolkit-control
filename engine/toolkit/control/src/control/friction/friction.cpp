// control/friction: the friction force against a velocity u: Coulomb, viscous and a Stribeck
// peak at rest, f = sign(u) (coulomb + (static − coulomb) e^-(u/stribeck_v)²) + viscous u.

#include <cmath>
#include <memory>
#include <stdexcept>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {

void add_friction(Registry& registry) {
  add_signal(registry, make_template(
      "control", "friction", "Friction", {"u"}, {"y"},
      {{.id = "coulomb", .default_value = 1.0}, {.id = "viscous", .default_value = 0.1},
       {.id = "static", .default_value = 1.0}, {.id = "stribeck_v", .default_value = 0.01, .unit = "m/s"}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        const double fc = get_or(p, "coulomb", 1.0), b = get_or(p, "viscous", 0.1), fs = get_or(p, "static", 1.0),
                     vs = get_or(p, "stribeck_v", 0.01);
        if (vs <= 0) throw std::invalid_argument("stribeck_v: expected more than 0");
        return std::make_unique<Stateless>([=](const Context&, const Values& in) {
          const double v = get_or(in, "u");
          const double sign = (v > 0) - (v < 0);
          return Values{{"y", sign * (fc + (fs - fc) * std::exp(-(v / vs) * (v / vs))) + b * v}};
        });
      },
      "The friction force at velocity u: Coulomb, viscous, and a Stribeck peak (static) near rest."));
}

}  // namespace paperboat::toolkit::control
