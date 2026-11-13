// control/moving_average: the mean of its last `window` samples, an FIR of equal taps.

#include <cmath>

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_moving_average(Registry& registry) {
  add_designed(
      registry, "moving_average", "Moving average",
      {{.id = "window", .default_value = 10.0}},
      "",
      [](const Values& p, const Config&) -> DesignFn {
        const double n = get_or(p, "window", 10.0);
        if (n < 1 || n > 10000 || n != std::round(n)) throw std::invalid_argument("window: expected a whole number from 1 to 10000");
        // ponytail: O(window) per step as a DF-II section; a running sum if long windows get slow.
        Design d{Coefficients(static_cast<std::size_t>(n), 1.0 / n), Coefficients(static_cast<std::size_t>(n), 0.0), "z"};
        d.den[0] = 1.0;
        return [d](double) { return d; };
      },
      "The mean of its last window samples (every step, or every sample_time).");
}

}  // namespace paperboat::toolkit::control
