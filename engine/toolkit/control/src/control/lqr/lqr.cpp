// control/lqr: the state-feedback gain K (u = −K x) that minimizes ∫ xᵀQx + uᵀRu (s) or
// Σ xᵀQx + uᵀRu (z), from the algebraic Riccati equation.

#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/riccati.hpp"

namespace paperboat::toolkit::control {
namespace {

class Lqr : public Block {
 public:
  Lqr(const Values& p, const Config& c) {
    const StateInputs m = parse_state_inputs(p, c);
    const Matrix Q = weight_param(p, c, "Q", m.n, "one row per state"), R = weight_param(p, c, "R", m.m, "one row per input");
    const bool discrete = enum_text(p, c, "domain", {"s", "z"}, "s") == "z";
    const Matrix P = discrete ? dare(m.A, m.B, Q, R) : care(m.A, m.B, Q, R);
    const Matrix Bt = transpose(m.B);
    const Matrix K = discrete ? solve(add(R, multiply(multiply(Bt, P), m.B)), multiply(multiply(Bt, P), m.A), "R + BᵀPB")
                              : solve(R, multiply(Bt, P), "R");
    K_ = matrix_text(K), P_ = matrix_text(P);
  }
  Values step(const Context&, const Values&) override { return {{"K", K_}, {"P", P_}}; }
  bool stateless() const override { return true; }

 private:
  std::string K_, P_;
};

}  // namespace

void add_lqr(Registry& registry) {
  auto t = make_template(
      "control", "lqr", "LQR", {}, {"K", "P"},
      {{.id = "A", .type = "text", .default_text = "0"}, {.id = "B", .type = "text", .default_text = "1"},
       {.id = "Q", .type = "text", .default_text = "1"}, {.id = "R", .type = "text", .default_text = "1"},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}}},
      [](const Values& p, const Config& c) { return std::make_unique<Lqr>(p, c); },
      "The optimal state-feedback gain K (u = −K x) for weights Q (states) and R (inputs), and the "
      "Riccati solution P, as matrix text. A number for Q or R is that times the identity.");
  t.def.flows = {"signal"};
  t.def.outputs = {{.id = "K", .type = "text"}, {.id = "P", .type = "text"}};
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
