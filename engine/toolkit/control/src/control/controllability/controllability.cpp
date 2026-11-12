// control/controllability: whether [B AB ... A^(n-1) B] has full row rank.

#include <memory>
#include <string>
#include <utility>

#include "common.hpp"
#include "control/matrix.hpp"

namespace paperboat::toolkit::control {
namespace {

class Controllability : public Block {
 public:
  explicit Controllability(const Values& params, const Config& config) {
    const StateInputs model = parse_state_inputs(params, config);
    order_ = model.n;
    rank_ = matrix_rank(controllability_matrix(model.A, model.B));
  }

  Values step(const Context&, const Values&) override {
    return {{"rank", static_cast<double>(rank_)},
            {"order", static_cast<double>(order_)},
            {"controllable", rank_ == order_ ? 1.0 : 0.0}};
  }

  bool stateless() const override { return true; }

 private:
  std::size_t order_ = 0;
  std::size_t rank_ = 0;
};

}  // namespace

void add_controllability(Registry& registry) {
  auto controllability = make_template(
      "control", "controllability", "Controllability", {}, {"rank", "order", "controllable"},
      {{.id = "A", .type = "text", .default_text = "0"}, {.id = "B", .type = "text", .default_text = "1"}},
      [](const Values& p, const Config& c) { return std::make_unique<Controllability>(p, c); },
      "The rank of the controllability matrix [B AB ... A^(n-1) B], the system order and whether "
      "the system is controllable. A and B are matrix text.");
  controllability.def.flows = {"signal"};
  registry.add(std::move(controllability));
}

}  // namespace paperboat::toolkit::control
