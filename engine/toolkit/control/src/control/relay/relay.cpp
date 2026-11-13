// control/relay: a switch with hysteresis. On once the input reaches on_at, off once it falls
// to off_at; it starts off. Also bang-bang control.

#include <memory>
#include <stdexcept>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class Relay : public Block {
 public:
  Relay(double on_at, double off_at, double on, double off) : on_at_(on_at), off_at_(off_at), on_(on), off_(off) {}
  Values step(const Context&, const Values& in) override {
    const double u = get_or(in, "u");
    if (u >= on_at_) state_ = true;
    else if (u <= off_at_) state_ = false;
    return {{"y", state_ ? on_ : off_}};
  }

 private:
  double on_at_, off_at_, on_, off_;
  bool state_ = false;
};

}  // namespace

void add_relay(Registry& registry) {
  add_signal(registry, make_template(
      "control", "relay", "Relay", {"u"}, {"y"},
      {{.id = "on_at", .default_value = 0.1}, {.id = "off_at", .default_value = -0.1},
       {.id = "on_value", .default_value = 1.0}, {.id = "off_value", .default_value = 0.0}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        const double on_at = get_or(p, "on_at", 0.1), off_at = get_or(p, "off_at", -0.1);
        if (off_at > on_at) throw std::invalid_argument("off_at: expected at most on_at");
        return std::make_unique<Relay>(on_at, off_at, get_or(p, "on_value", 1.0), get_or(p, "off_value"));
      },
      "A switch with hysteresis: on_value once the input reaches on_at, off_value once it falls to off_at."));
}

}  // namespace paperboat::toolkit::control
