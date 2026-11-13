// control/tapped_delay: its last `taps` samples as a numbers list, newest first.

#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <vector>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class TappedDelay : public Block {
 public:
  TappedDelay(std::size_t taps, double initial, double period) : last_(taps, initial), sampler_{period} {}
  Values step(const Context& ctx, const Values& in) override {
    if (sampler_.due(ctx.t)) {
      last_.push_front(get_or(in, "u"));
      last_.pop_back();
    }
    return {{"y", std::vector<double>(last_.begin(), last_.end())}};
  }

 private:
  std::deque<double> last_;
  Sampler sampler_;
};

}  // namespace

void add_tapped_delay(Registry& registry) {
  auto t = make_template(
      "control", "tapped_delay", "Tapped delay", {"u"}, {"y"},
      {{.id = "taps", .default_value = 4.0}, {.id = "initial", .default_value = 0.0},
       {.id = "sample_time", .default_value = 0.0, .unit = "s"}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        const double taps = get_or(p, "taps", 4.0);
        if (taps < 1 || taps > 10000 || taps != std::round(taps)) throw std::invalid_argument("taps: expected a whole number from 1 to 10000");
        if (get_or(p, "sample_time") < 0) throw std::invalid_argument("sample_time: expected 0 or more");
        return std::make_unique<TappedDelay>(static_cast<std::size_t>(taps), get_or(p, "initial"), get_or(p, "sample_time"));
      },
      "Its last taps samples as a numbers list, newest first (this step's input included).");
  t.def.outputs = {numbers_port("y")};
  add_signal(registry, std::move(t), false);
}

}  // namespace paperboat::toolkit::control
