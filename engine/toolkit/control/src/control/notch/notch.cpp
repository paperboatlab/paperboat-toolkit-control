// control/notch: (s² + 2 zz w0 s + w0²) / (s² + 2 zp w0 s + w0²), zz = zp 10^(-depth/20).
// It cuts `depth` dB at `freq`; `width` (zp) sets how wide.

#include <cmath>
#include <numbers>

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_notch(Registry& registry) {
  add_designed(
      registry, "notch", "Notch",
      {{.id = "freq", .default_value = 50.0, .unit = "Hz"}, {.id = "depth", .default_value = 40.0, .unit = "dB"}, {.id = "width", .default_value = 0.5}},
      "tustin",
      [](const Values& p, const Config&) -> DesignFn {
        const double f = get_or(p, "freq", 50.0), depth = get_or(p, "depth", 40.0), zp = get_or(p, "width", 0.5);
        if (f <= 0) throw std::invalid_argument("freq: expected more than 0 Hz");
        if (zp <= 0) throw std::invalid_argument("width: expected more than 0");
        const double zz = zp * std::pow(10.0, -depth / 20.0);
        return [=](double T) {
          const double w = warp(2 * std::numbers::pi * f, T, "freq");
          return Design{{1.0, 2 * zz * w, w * w}, {1.0, 2 * zp * w, w * w}};
        };
      },
      "Cuts one frequency: depth dB at freq, width sets how wide. For a resonance or mains hum.");
}

}  // namespace paperboat::toolkit::control
