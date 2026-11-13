// control/state_feedback: u = −K x + N r. K comes from the param, or from the K input when
// it is wired (an lqr or place block's K); N defaults to the identity, so r is a feedforward u.

#include <memory>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "control/riccati.hpp"

namespace paperboat::toolkit::control {
namespace {

class StateFeedback : public Block {
 public:
  StateFeedback(std::string K, std::string N) : K_text_(std::move(K)), N_text_(std::move(N)) { parse(); }

  Values step(const Context&, const Values& in) override {
    if (const auto it = in.find("K"); it != in.end())
      if (const auto* text = std::get_if<std::string>(&it->second); text && *text != K_text_) K_text_ = *text, parse();
    const Vector x = as_vector(in, "x");
    if (x.size() != K_.cols)
      throw std::invalid_argument("x: expected " + std::to_string(K_.cols) + " values (K's columns), got " + std::to_string(x.size()));
    Vector r = as_vector(in, "r");
    const std::size_t m = K_.rows;
    Vector u(m, 0.0);
    for (std::size_t i = 0; i < m; ++i)
      for (std::size_t j = 0; j < K_.cols; ++j) u[i] -= K_(i, j) * x[j];
    if (!r.empty()) {
      const Matrix N = N_.rows ? N_ : identity(m);
      if (r.size() != N.cols) throw std::invalid_argument("r: expected " + std::to_string(N.cols) + " values, got " + std::to_string(r.size()));
      if (N.rows != m) matrix_error("N", "expected " + std::to_string(m) + " rows (K's), got " + shape_text(N));
      for (std::size_t i = 0; i < m; ++i)
        for (std::size_t j = 0; j < N.cols; ++j) u[i] += N(i, j) * r[j];
    }
    return {{"u", u}};
  }

 private:
  void parse() {
    K_ = parse_matrix(K_text_, "K");
    if (K_.rows == 0) matrix_error("K", "expected a matrix, one row per input");
    N_ = parse_matrix(N_text_, "N");
  }
  std::string K_text_, N_text_;
  Matrix K_, N_;
};

}  // namespace

void add_state_feedback(Registry& registry) {
  auto t = make_template(
      "control", "state_feedback", "State feedback", {"x", "r", "K"}, {"u"},
      {{.id = "K", .type = "text", .default_text = "1"}, {.id = "N", .type = "text", .default_text = ""}},
      [](const Values& p, const Config& c) {
        return std::make_unique<StateFeedback>(param_text(p, c, "K", "1"), param_text(p, c, "N", ""));
      },
      "u = −K x + N r. Wire an lqr or place block's K into the K input, or type K. N empty: the identity.");
  t.def.flows = {"signal"};
  t.def.inputs = {numbers_port("x"), numbers_port("r"), {.id = "K", .type = "text"}};
  t.def.outputs = {numbers_port("u")};
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
