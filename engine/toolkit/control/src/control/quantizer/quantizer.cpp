// control/quantizer: the input in steps of `interval`, as an ADC or an encoder reads it.

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/matrix.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {

void add_quantizer(Registry& registry) {
  add_signal(registry, make_template(
      "control", "quantizer", "Quantizer", {"u"}, {"y"},
      {{.id = "interval", .default_value = 0.1}, {.id = "mode", .type = "enum", .default_text = "round", .options = {"round", "floor"}}},
      [](const Values& p, const Config& c) -> std::unique_ptr<Block> {
        const double q = get_or(p, "interval", 0.1);
        if (q <= 0) throw std::invalid_argument("interval: expected more than 0");
        const bool floor = enum_text(p, c, "mode", {"round", "floor"}, "round") == "floor";
        return std::make_unique<Stateless>([q, floor](const Context&, const Values& in) {
          const double x = get_or(in, "u") / q;
          return Values{{"y", (floor ? std::floor(x) : std::round(x)) * q}};
        });
      },
      "The input in steps of interval, rounded or floored: what an ADC or an encoder reads."));
}

}  // namespace paperboat::toolkit::control
