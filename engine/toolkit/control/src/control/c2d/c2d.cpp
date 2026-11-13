// control/c2d: a continuous model x' = A x + B u as the discrete x[k+1] = Ad x + Bd u at a
// sample time dt: zoh (exact for a held input: e^[[A B]; [0 0]] dt) or tustin.

#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/riccati.hpp"

namespace paperboat::toolkit::control {
namespace {

class C2d : public Block {
 public:
  C2d(const Values& p, const Config& c) {
    const StateInputs m = parse_state_inputs(p, c);
    const double dt = get_or(p, "dt", 0.01);
    if (dt <= 0) throw std::invalid_argument("dt: expected more than 0 s");
    Matrix Ad, Bd;
    if (enum_text(p, c, "method", {"zoh", "tustin"}, "zoh") == "zoh") {
      Matrix M = Matrix::zeros(m.n + m.m, m.n + m.m);
      for (std::size_t r = 0; r < m.n; ++r) {
        for (std::size_t k = 0; k < m.n; ++k) M(r, k) = m.A(r, k) * dt;
        for (std::size_t k = 0; k < m.m; ++k) M(r, m.n + k) = m.B(r, k) * dt;
      }
      const Matrix E = expm(M);
      Ad = Matrix::zeros(m.n, m.n), Bd = Matrix::zeros(m.n, m.m);
      for (std::size_t r = 0; r < m.n; ++r) {
        for (std::size_t k = 0; k < m.n; ++k) Ad(r, k) = E(r, k);
        for (std::size_t k = 0; k < m.m; ++k) Bd(r, k) = E(r, m.n + k);
      }
    } else {
      const Matrix half = scale(m.A, dt / 2);
      const Matrix left = add(identity(m.n), half, -1.0);
      Ad = solve(left, add(identity(m.n), half), "I - A dt/2");
      Bd = solve(left, scale(m.B, dt), "I - A dt/2");
    }
    Ad_ = matrix_text(Ad), Bd_ = matrix_text(Bd);
  }
  Values step(const Context&, const Values&) override { return {{"Ad", Ad_}, {"Bd", Bd_}}; }
  bool stateless() const override { return true; }

 private:
  std::string Ad_, Bd_;
};

}  // namespace

void add_c2d(Registry& registry) {
  auto t = make_template(
      "control", "c2d", "Continuous to discrete", {}, {"Ad", "Bd"},
      {{.id = "A", .type = "text", .default_text = "0"}, {.id = "B", .type = "text", .default_text = "1"},
       {.id = "dt", .default_value = 0.01, .unit = "s"},
       {.id = "method", .type = "enum", .default_text = "zoh", .options = {"zoh", "tustin"}}},
      [](const Values& p, const Config& c) { return std::make_unique<C2d>(p, c); },
      "Discretizes x' = A x + B u at dt: Ad and Bd as matrix text, by zoh (exact for a held input) or tustin.");
  t.def.flows = {"signal"};
  t.def.outputs = {{.id = "Ad", .type = "text"}, {.id = "Bd", .type = "text"}};
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
