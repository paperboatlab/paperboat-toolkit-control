// control/zoh: sample and hold. It takes its input every sample_time s and holds it.

#include <memory>
#include <stdexcept>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class Zoh : public Block {
 public:
  explicit Zoh(double period) : sampler_{period} {}
  Values step(const Context& ctx, const Values& in) override {
    if (sampler_.due(ctx.t)) y_ = get_or(in, "u");
    return {{"y", y_}};
  }

 private:
  Sampler sampler_;
  double y_ = 0.0;
};

}  // namespace

void add_zoh(Registry& registry) {
  add_signal(registry, make_template(
      "control", "zoh", "Zero-order hold", {"u"}, {"y"}, {{.id = "sample_time", .default_value = 0.1, .unit = "s"}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        if (get_or(p, "sample_time", 0.1) <= 0) throw std::invalid_argument("sample_time: expected more than 0");
        return std::make_unique<Zoh>(get_or(p, "sample_time", 0.1));
      },
      "Samples its input every sample_time and holds it: what a digital controller sees."));
}

}  // namespace paperboat::toolkit::control
