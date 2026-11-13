// control/biquad: one second-order section, the Audio EQ Cookbook (R. Bristow-Johnson) forms.
// Each is its analog prototype at 1 rad/s, scaled to the prewarped freq, then Tustin: the
// cookbook's own derivation, so the digital coefficients are the cookbook's.

#include <cmath>
#include <numbers>
#include <string>

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_biquad(Registry& registry) {
  add_designed(
      registry, "biquad", "Biquad",
      {{.id = "kind", .type = "enum", .default_text = "lowpass",
        .options = {"lowpass", "highpass", "bandpass", "notch", "allpass", "peak", "lowshelf", "highshelf"}},
       {.id = "freq", .default_value = 10.0, .unit = "Hz"},
       {.id = "q", .default_value = 0.7071},
       {.id = "gain_db", .default_value = 0.0, .unit = "dB"}},
      "tustin",
      [](const Values& p, const Config& c) -> DesignFn {
        const std::string kind = enum_text(
            p, c, "kind", {"lowpass", "highpass", "bandpass", "notch", "allpass", "peak", "lowshelf", "highshelf"}, "lowpass");
        const double f = get_or(p, "freq", 10.0), q = get_or(p, "q", 0.7071), A = std::pow(10.0, get_or(p, "gain_db", 0.0) / 40.0);
        if (f <= 0) throw std::invalid_argument("freq: expected more than 0 Hz");
        if (q <= 0) throw std::invalid_argument("q: expected more than 0");
        Coefficients b, a{1.0, 1.0 / q, 1.0};
        const double r = std::sqrt(A) / q;
        if (kind == "lowpass") b = {0, 0, 1};
        else if (kind == "highpass") b = {1, 0, 0};
        else if (kind == "bandpass") b = {0, 1.0 / q, 0};
        else if (kind == "notch") b = {1, 0, 1};
        else if (kind == "allpass") b = {1, -1.0 / q, 1};
        else if (kind == "peak") b = {1, A / q, 1}, a = {1, 1.0 / (A * q), 1};
        else if (kind == "lowshelf") b = {A, A * r, A * A}, a = {A, r, 1};
        else b = {A * A, A * r, A}, a = {1, r, A};  // highshelf
        return [=](double T) {
          const double w = warp(2 * std::numbers::pi * f, T, "freq");
          const Polynomials s = substitute(b, a, {1.0, 0.0}, {w});  // s → s / w
          return Design{s.num, s.den};
        };
      },
      "One second-order section from the Audio EQ Cookbook: lowpass, highpass, bandpass, notch, allpass, "
      "peak, lowshelf or highshelf at freq with q (and gain_db for peak and shelves).");
}

}  // namespace paperboat::toolkit::control
