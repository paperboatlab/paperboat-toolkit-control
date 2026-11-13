// control/median: the median of its last `window` samples; removes spikes, keeps edges.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <vector>

#include "common.hpp"

namespace paperboat::toolkit::control {
namespace {

class Median : public Block {
 public:
  explicit Median(std::size_t window) : window_(window) {}
  Values step(const Context&, const Values& in) override {
    last_.push_back(get_or(in, "u"));
    if (last_.size() > window_) last_.pop_front();
    std::vector<double> sorted(last_.begin(), last_.end());
    const auto mid = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2);
    std::nth_element(sorted.begin(), mid, sorted.end());
    return {{"y", *mid}};
  }

 private:
  std::size_t window_;
  std::deque<double> last_;
};

}  // namespace

void add_median(Registry& registry) {
  auto t = make_template(
      "control", "median", "Median", {"u"}, {"y"}, {{.id = "window", .default_value = 5.0}},
      [](const Values& p, const Config&) -> std::unique_ptr<Block> {
        const double n = get_or(p, "window", 5.0);
        if (n < 1 || n > 101 || n != std::round(n) || static_cast<int>(n) % 2 == 0)
          throw std::invalid_argument("window: expected an odd whole number from 1 to 101");
        return std::make_unique<Median>(static_cast<std::size_t>(n));
      },
      "The median of its last window samples: removes spikes and keeps edges.");
  t.def.flows = {"signal"};
  t.def.elementwise = true;
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
