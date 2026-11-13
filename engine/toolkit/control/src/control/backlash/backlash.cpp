// control/backlash: gear play. The output moves only once the input has crossed a gap of
// `width` around it, then follows at its edge.

#include <memory>
#include <stdexcept>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class Backlash : public Block {
 public:
  Backlash(double width, double initial) : half_(width / 2), y_(initial) {}
  Values step(const Context&, const Values& in) override {
    const double u = get_or(in, "u");
    if (u > y_ + half_) y_ = u - half_;
    else if (u < y_ - half_) y_ = u + half_;
    return {{"y", y_}};
  }

 private:
  double half_, y_;
};

}  // namespace

void add_backlash(Registry& registry) {
  add_signal(registry, make_template(
      "control", "backlash", "Backlash", {"u"}, {"y"},
      {{.id = "width", .default_value = 0.1}, {.id = "initial", .default_value = 0.0}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        if (get_or(p, "width", 0.1) < 0) throw std::invalid_argument("width: expected 0 or more");
        return std::make_unique<Backlash>(get_or(p, "width", 0.1), get_or(p, "initial"));
      },
      "Gear play: the output moves only once the input crosses a gap of width, then follows at its edge."));
}

}  // namespace paperboat::toolkit::control
