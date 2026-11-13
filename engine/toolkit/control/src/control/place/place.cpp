// control/place: the state-feedback gain K that puts the closed-loop poles of A − B K where
// asked, by Ackermann's formula: K = [0 … 0 1] 𝒞⁻¹ φ(A). One input (B a column).

#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/lti.hpp"
#include "control/riccati.hpp"

namespace paperboat::toolkit::control {
namespace {

class Place : public Block {
 public:
  Place(const Values& p, const Config& c) {
    const StateInputs m = parse_state_inputs(p, c);
    if (m.m != 1) matrix_error("B", "expected one column (place takes one input), got " + shape_text(m.B));
    const auto poles = parse_roots(param_text(p, c, "poles", "-1"), "poles");
    if (poles.size() != m.n)
      throw std::invalid_argument("poles: expected " + std::to_string(m.n) + " (one per state), got " + std::to_string(poles.size()));
    const Coefficients phi = poly_from_roots(poles, "poles");  // descending, phi[0] = 1
    Matrix phiA = Matrix::zeros(m.n, m.n);                     // Horner: φ(A)
    for (double coefficient : phi) phiA = add(multiply(phiA, m.A), identity(m.n), coefficient);
    Matrix last = Matrix::zeros(1, m.n);
    last(0, m.n - 1) = 1.0;
    // K = eₙᵀ 𝒞⁻¹ φ(A): solve 𝒞ᵀ v = eₙ, then K = vᵀ φ(A).
    const Matrix v = solve(transpose(controllability_matrix(m.A, m.B)), transpose(last), "the controllability matrix (not controllable)");
    K_ = matrix_text(multiply(transpose(v), phiA));
  }
  Values step(const Context&, const Values&) override { return {{"K", K_}}; }
  bool stateless() const override { return true; }

 private:
  std::string K_;
};

}  // namespace

void add_place(Registry& registry) {
  auto t = make_template(
      "control", "place", "Pole placement", {}, {"K"},
      {{.id = "A", .type = "text", .default_text = "0"}, {.id = "B", .type = "text", .default_text = "1"},
       {.id = "poles", .type = "text", .default_text = "-1"}},
      [](const Values& p, const Config& c) { return std::make_unique<Place>(p, c); },
      "The state-feedback gain K that puts the poles of A − B K at poles (like -1 -2+3j -2-3j), as matrix text. One input.");
  t.def.flows = {"signal"};
  t.def.outputs = {{.id = "K", .type = "text"}};
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
