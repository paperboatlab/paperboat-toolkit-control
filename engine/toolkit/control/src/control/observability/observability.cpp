// control/observability: whether [C; CA; ...; CA^(n-1)] has full column rank.

#include <memory>
#include <string>
#include <utility>

#include "common.hpp"
#include "control/matrix.hpp"

namespace paperboat::toolkit::control {
namespace {

class Observability : public Block {
 public:
  Observability(const Values& params, const Config& config) {
    const StateOutputs model = parse_state_outputs(params, config);
    order_ = model.n;
    rank_ = matrix_rank(observability_matrix(model.A, model.C));
  }

  Values step(const Context&, const Values&) override {
    return {{"rank", static_cast<double>(rank_)},
            {"order", static_cast<double>(order_)},
            {"observable", rank_ == order_ ? 1.0 : 0.0}};
  }

  bool stateless() const override { return true; }

 private:
  std::size_t order_ = 0;
  std::size_t rank_ = 0;
};

}  // namespace

void add_observability(Registry& registry) {
  auto observability = make_template(
      "control", "observability", "Observability", {}, {"rank", "order", "observable"},
      {{.id = "A", .type = "text", .default_text = "0"}, {.id = "C", .type = "text", .default_text = "1"}},
      [](const Values& p, const Config& c) { return std::make_unique<Observability>(p, c); },
      "The rank of the observability matrix [C; CA; ...; CA^(n-1)], the system order and whether "
      "the system is observable. A and C are matrix text.");
  observability.def.flows = {"signal"};
  registry.add(std::move(observability));
}

}  // namespace paperboat::toolkit::control
