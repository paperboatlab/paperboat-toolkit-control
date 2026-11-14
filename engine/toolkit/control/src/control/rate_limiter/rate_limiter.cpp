// control/rate_limiter: the output follows the input no faster than `rise` up and `fall`
// down (units/s). It starts at its first input.

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class RateLimiter : public Block {
 public:
  RateLimiter(double rise, double fall) : rise_(rise), fall_(fall) {}
  Values step(const Context& ctx, const Values& in) override {
    const double u = get_or(in, "u");
    y_ = y_ ? std::clamp(u, *y_ - fall_ * ctx.dt, *y_ + rise_ * ctx.dt) : u;
    return {{"y", *y_}};
  }

 private:
  double rise_, fall_;
  std::optional<double> y_;
};

}  // namespace

void add_rate_limiter(Registry& registry) {
  add_signal(registry, make_template(
      "control", "rate_limiter", "Rate limiter", {"u"}, {"y"},
      {{.id = "rise", .default_value = 1.0, .unit = "/s"}, {.id = "fall", .default_value = 1.0, .unit = "/s"}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        const double rise = get_or(p, "rise", 1.0), fall = get_or(p, "fall", 1.0);
        if (rise < 0 || fall < 0) throw std::invalid_argument("rise, fall: expected 0 or more (fall is a positive rate)");
        return std::make_unique<RateLimiter>(rise, fall);
      },
      "Follows its input no faster than rise up and fall down, per second: an actuator's slew limit."));
}

}  // namespace paperboat::toolkit::control
