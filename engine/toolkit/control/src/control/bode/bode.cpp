// control/bode: the frequency response of H(s) or H(z), from the coefficients of its
// numerator and denominator (descending powers).

#include <algorithm>
#include <cmath>
#include <complex>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common.hpp"
#include "control/matrix.hpp"

namespace paperboat::toolkit::control {
namespace {

class Bode : public Block {
 public:
  Bode(Coefficients num, Coefficients den, std::string domain, double sample_time)
      : num_(std::move(num)), den_(std::move(den)), domain_(std::move(domain)), sample_time_(sample_time) {
    if (std::all_of(den_.begin(), den_.end(), [](double value) { return value == 0.0; }))
      throw std::invalid_argument("den: the denominator must not be zero");
  }

  Values step(const Context&, const Values& in) override {
    const Vector frequencies = as_vector(in, "w");
    Vector magnitude;
    Vector phase;
    magnitude.reserve(frequencies.size());
    phase.reserve(frequencies.size());
    for (const double omega : frequencies) {
      const std::complex<double> at =
          domain_ == "z" ? std::polar(1.0, omega * sample_time_) : std::complex<double>(0.0, omega);
      const std::complex<double> response = evaluate(num_, at) / evaluate(den_, at);
      magnitude.push_back(20.0 * std::log10(std::abs(response)));
      phase.push_back(std::arg(response) * 180.0 / std::numbers::pi);
    }
    return {{"magnitude", std::move(magnitude)}, {"phase", std::move(phase)}};
  }

  bool stateless() const override { return true; }

 private:
  static std::complex<double> evaluate(const Coefficients& coefficients, const std::complex<double>& at) {
    std::complex<double> value = 0.0;
    for (const double coefficient : coefficients) value = value * at + coefficient;
    return value;
  }

  Coefficients num_;
  Coefficients den_;
  std::string domain_;
  double sample_time_;
};

}  // namespace

void add_bode(Registry& registry) {
  auto bode = make_template(
      "control", "bode", "Bode", {"w"}, {"magnitude", "phase"},
      {{.id = "num", .type = "text", .default_text = "1"},
       {.id = "den", .type = "text", .default_text = "1 1"},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}},
       {.id = "sample_time", .default_value = 1.0, .type = "number"}},
      [](const Values& p, const Config& c) {
        return std::make_unique<Bode>(parse_coefficients(param_text(p, c, "num", "1"), "num"),
                                      parse_coefficients(param_text(p, c, "den", "1 1"), "den"),
                                      enum_text(p, c, "domain", {"s", "z"}, "s"), get_or(p, "sample_time", 1.0));
      },
      "The frequency response of num/den: magnitude in dB (20 log10 |H|) and phase in degrees. "
      "For z, sample_time sets z = exp(j w sample_time).");
  bode.def.inputs[0] = numbers_port("w");
  bode.def.outputs = {numbers_port("magnitude"), numbers_port("phase")};
  bode.def.flows = {"signal"};
  registry.add(std::move(bode));
}

}  // namespace paperboat::toolkit::control
