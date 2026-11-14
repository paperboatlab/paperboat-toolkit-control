// control/filter: an IIR filter designer, as MATLAB's butter / cheby1 / cheby2 / besself.
//
// 1. An analog lowpass prototype at 1 rad/s (Butterworth and Chebyshev from their poles,
//    Bessel from its reverse Bessel polynomial, normalized to -3 dB at 1 rad/s).
// 2. The edges, prewarped for the step it runs at, and a frequency transform (lowpass,
//    highpass, bandpass, bandstop; a band kind doubles the order, as in MATLAB).
// 3. Tustin (realize = tustin, the default) or the analog filter integrated.
// The chebyshev2 cutoff is the stopband edge, as in SciPy and MATLAB; the others' the passband.

#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {
namespace {

using std::numbers::pi;

Design prototype(const std::string& family, int n, double ripple, double attenuation) {
  std::vector<Complex> poles, zeros;
  double gain = 1.0;
  if (family == "butterworth") {
    for (int k = 0; k < n; ++k) poles.push_back(std::polar(1.0, pi * (2.0 * k + n + 1) / (2.0 * n)));
  } else if (family == "chebyshev1") {
    if (ripple <= 0) throw std::invalid_argument("ripple: expected more than 0 dB");
    const double eps = std::sqrt(std::pow(10.0, ripple / 10.0) - 1.0);
    const double mu = std::asinh(1.0 / eps) / n;
    for (int k = 0; k < n; ++k) {
      const double theta = pi * (2.0 * k + 1) / (2.0 * n);
      poles.push_back({-std::sinh(mu) * std::sin(theta), std::cosh(mu) * std::cos(theta)});
    }
    Complex g = 1.0;
    for (const Complex& p : poles) g *= -p;
    gain = g.real() / (n % 2 == 0 ? std::sqrt(1.0 + eps * eps) : 1.0);
  } else if (family == "chebyshev2") {
    if (attenuation <= 0) throw std::invalid_argument("attenuation: expected more than 0 dB");
    const double de = 1.0 / std::sqrt(std::pow(10.0, attenuation / 10.0) - 1.0);
    const double mu = std::asinh(1.0 / de) / n;
    for (int m = -n + 1; m < n; m += 2) {
      if (m != 0) zeros.push_back({0.0, 1.0 / std::sin(m * pi / (2.0 * n))});
      const Complex p = -std::polar(1.0, pi * m / (2.0 * n));
      poles.push_back(1.0 / Complex(std::sinh(mu) * p.real(), std::cosh(mu) * p.imag()));
    }
    Complex pz = 1.0, pp = 1.0;
    for (const Complex& p : poles) pp *= -p;
    for (const Complex& z : zeros) pz *= -z;
    gain = (pp / pz).real();
  } else {  // bessel: θn(s) = Σ (2n−k)! / (2^(n−k) k! (n−k)!) s^k, then -3 dB at 1 rad/s
    Coefficients den(n + 1);
    for (int k = 0; k <= n; ++k)
      den[n - k] = std::tgamma(2.0 * n - k + 1) / (std::pow(2.0, n - k) * std::tgamma(k + 1.0) * std::tgamma(n - k + 1.0));
    const auto mag2 = [&](double w) {
      Complex d = 0.0;
      for (double c : den) d = d * Complex(0.0, w) + c;
      return std::norm(den[n] / d);
    };
    double lo = 0.0, hi = 1.0;
    while (mag2(hi) > 0.5) hi *= 2.0;
    for (int i = 0; i < 200; ++i) (mag2((lo + hi) / 2) > 0.5 ? lo : hi) = (lo + hi) / 2;
    const double w3 = (lo + hi) / 2;  // H(w3 s) is -3 dB at 1 rad/s
    for (int k = 0; k <= n; ++k) den[n - k] *= std::pow(w3, k);
    return {{den[n]}, den};
  }
  Coefficients num = poly_from_roots(zeros, "zeros");
  for (double& c : num) c *= gain;
  return {num, poly_from_roots(poles, "poles")};
}

}  // namespace

void add_filter(Registry& registry) {
  add_designed(
      registry, "filter", "Filter",
      {{.id = "family", .type = "enum", .default_text = "butterworth",
        .options = {"butterworth", "chebyshev1", "chebyshev2", "bessel"}},
       {.id = "kind", .type = "enum", .default_text = "lowpass", .options = {"lowpass", "highpass", "bandpass", "bandstop"}},
       {.id = "order", .default_value = 2.0},
       {.id = "cutoff", .default_value = 10.0, .unit = "Hz"},
       {.id = "cutoff2", .default_value = 20.0, .unit = "Hz"},
       {.id = "ripple", .default_value = 1.0, .unit = "dB"},
       {.id = "attenuation", .default_value = 40.0, .unit = "dB"}},
      "tustin",
      [](const Values& p, const Config& c) -> DesignFn {
        const std::string family = enum_text(p, c, "family", {"butterworth", "chebyshev1", "chebyshev2", "bessel"}, "butterworth");
        const std::string kind = enum_text(p, c, "kind", {"lowpass", "highpass", "bandpass", "bandstop"}, "lowpass");
        const double order = get_or(p, "order", 2.0);
        if (order < 1 || order > 10 || order != std::round(order))
          throw std::invalid_argument("order: expected a whole number from 1 to 10");
        const double f1 = get_or(p, "cutoff", 10.0), f2 = get_or(p, "cutoff2", 20.0);
        const bool band = kind == "bandpass" || kind == "bandstop";
        if (f1 <= 0) throw std::invalid_argument("cutoff: expected more than 0 Hz");
        if (band && f2 <= f1) throw std::invalid_argument("cutoff2: expected more than cutoff for a band");
        const Design proto = prototype(family, static_cast<int>(order), get_or(p, "ripple", 1.0), get_or(p, "attenuation", 40.0));
        return [=](double T) {
          const double w1 = warp(2 * pi * f1, T, "cutoff");
          Polynomials out;
          if (kind == "lowpass") out = substitute(proto.num, proto.den, {1.0, 0.0}, {w1});
          else if (kind == "highpass") out = substitute(proto.num, proto.den, {w1}, {1.0, 0.0});
          else {
            const double w2 = warp(2 * pi * f2, T, "cutoff2");
            const double w0sq = w1 * w2, bw = w2 - w1;
            out = kind == "bandpass" ? substitute(proto.num, proto.den, {1.0, 0.0, w0sq}, {bw, 0.0})
                                     : substitute(proto.num, proto.den, {bw, 0.0}, {1.0, 0.0, w0sq});
          }
          return Design{out.num, out.den, "s"};
        };
      },
      "An IIR filter: Butterworth, Chebyshev I/II or Bessel; lowpass, highpass, bandpass or bandstop. "
      "Tustin with prewarped edges by default, so the cutoff is exact at any step.");
}

}  // namespace paperboat::toolkit::control
