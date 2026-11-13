// control/lead_lag: k (s/z + 1) / (s/p + 1). Lead when zero < pole, lag when zero > pole.

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_lead_lag(Registry& registry) {
  add_designed(
      registry, "lead_lag", "Lead-lag",
      {{.id = "gain", .default_value = 1.0}, {.id = "zero", .default_value = 1.0, .unit = "rad/s"}, {.id = "pole", .default_value = 10.0, .unit = "rad/s"}},
      "tustin",
      [](const Values& p, const Config&) -> DesignFn {
        const double k = get_or(p, "gain", 1.0), z = get_or(p, "zero", 1.0), pole = get_or(p, "pole", 10.0);
        if (z <= 0) throw std::invalid_argument("zero: expected more than 0 rad/s");
        if (pole <= 0) throw std::invalid_argument("pole: expected more than 0 rad/s");
        return [=](double) { return Design{{k / z, k}, {1.0 / pole, 1.0}}; };
      },
      "A lead-lag compensator k (s/zero + 1) / (s/pole + 1): lead when zero < pole, lag when zero > pole.");
}

}  // namespace paperboat::toolkit::control
