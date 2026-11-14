// control/foh: a first-order hold. Between samples it extends the line through the last two.

#include <memory>
#include <stdexcept>

#include "common.hpp"
#include "control/sampler.hpp"

namespace paperboat::toolkit::control {
namespace {

class Foh : public Block {
 public:
  explicit Foh(double period) : sampler_{period} {}
  Values step(const Context& ctx, const Values& in) override {
    if (sampler_.due(ctx.t)) {
      slope_ = started_ ? (get_or(in, "u") - last_) / sampler_.period : 0.0;
      last_ = get_or(in, "u"), at_ = ctx.t, started_ = true;
    }
    return {{"y", last_ + slope_ * (ctx.t - at_)}};
  }

 private:
  Sampler sampler_;
  bool started_ = false;
  double last_ = 0.0, slope_ = 0.0, at_ = 0.0;
};

}  // namespace

void add_foh(Registry& registry) {
  add_signal(registry, make_template(
      "control", "foh", "First-order hold", {"u"}, {"y"}, {{.id = "sample_time", .default_value = 0.1, .unit = "s"}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        if (get_or(p, "sample_time", 0.1) <= 0) throw std::invalid_argument("sample_time: expected more than 0");
        return std::make_unique<Foh>(get_or(p, "sample_time", 0.1));
      },
      "Samples its input every sample_time and extends the line through the last two samples between."));
}

}  // namespace paperboat::toolkit::control
