// control/rhs: x' = f(t, x, u), integrated with the run's solver.
// Equations in `f` are separated by `;` or newlines, one per state component.

#include <cctype>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common.hpp"
#include "control/matrix.hpp"
#include "paperboat/toolkit/expression.hpp"
#include "paperboat/toolkit/solve.hpp"

namespace paperboat::toolkit::control {
namespace {

enum class VarKind { Time, State, Input };

struct VarBinding {
  VarKind kind = VarKind::Time;
  std::size_t index = 0;
};

struct CompiledEquation {
  Expression expr;
  std::vector<VarBinding> bindings;
};

std::optional<std::size_t> parse_positive_index(std::string_view suffix) {
  if (suffix.empty()) return std::nullopt;
  std::size_t val = 0;
  for (char c : suffix) {
    if (!std::isdigit(static_cast<unsigned char>(c))) return std::nullopt;
    val = val * 10 + static_cast<std::size_t>(c - '0');
    if (val > 10000) return std::nullopt;
  }
  return val > 0 ? std::optional<std::size_t>(val - 1) : std::nullopt;
}

std::vector<std::string> split_equations(std::string_view text) {
  std::vector<std::string> out;
  std::size_t start = 0;
  for (std::size_t i = 0; i <= text.size(); ++i) {
    if (i < text.size() && text[i] != ';' && text[i] != '\n') continue;
    const std::string_view part = trim(text.substr(start, i - start));
    start = i + 1;
    if (!part.empty()) out.emplace_back(part);
  }
  if (out.empty()) throw ExpressionError("f: expected at least one ODE equation");
  return out;
}

VarBinding bind_name(std::string_view name, std::size_t n) {
  if (name == "t") return {VarKind::Time, 0};
  if (name == "x") {
    if (n != 1)
      throw ExpressionError("f: use x1..x" + std::to_string(n) + " for a " + std::to_string(n) + "-state system");
    return {VarKind::State, 0};
  }
  if (name.size() > 1 && name.front() == 'x') {
    if (auto idx = parse_positive_index(name.substr(1)); idx && *idx < n) return {VarKind::State, *idx};
    throw ExpressionError("f: \"" + std::string(name) + "\" is out of range for " + std::to_string(n) + " state(s)");
  }
  if (name == "u") return {VarKind::Input, 0};
  if (name.size() > 1 && name.front() == 'u') {
    if (auto idx = parse_positive_index(name.substr(1))) return {VarKind::Input, *idx};
  }
  throw ExpressionError("f: unknown variable \"" + std::string(name) + "\" (allowed: t, " +
                        (n == 1 ? "x, " : "x1..x" + std::to_string(n) + ", ") + "u, u1..)");
}

std::vector<CompiledEquation> compile_equations(std::string_view text, std::size_t& state_count) {
  const std::vector<std::string> parts = split_equations(text);
  state_count = parts.size();
  std::vector<CompiledEquation> compiled;
  compiled.reserve(state_count);
  for (const std::string& eq : parts) {
    std::vector<std::string> names = Expression::free_names(eq);
    std::vector<VarBinding> bindings;
    bindings.reserve(names.size());
    for (const std::string& name : names) bindings.push_back(bind_name(name, state_count));
    compiled.push_back({Expression(eq, std::move(names)), std::move(bindings)});
  }
  return compiled;
}

class RhsBlock final : public Block {
 public:
  RhsBlock(std::string f_text, Vector x0, std::string solver_name)
      : solver_name_(std::move(solver_name)), solver_(solve::make(solver_name_)) {
    std::size_t n = 0;
    equations_ = compile_equations(f_text, n);
    if (x0.size() != n)
      matrix_error("x0", "expected " + std::to_string(n) + " values, got " + std::to_string(x0.size()));
    x_ = std::move(x0);
    std::size_t max_vars = 0;
    for (const auto& eq : equations_) max_vars = std::max(max_vars, eq.bindings.size());
    eval_buf_.assign(max_vars, 0.0);
  }

  Values outputs() const override { return {{"x", x_}}; }

  Values step(const Context& ctx, const Values& in) override {
    const Vector u = as_vector(in, "u");
    solve::State state = x_;
    solver_->step(state, ctx.t, ctx.dt, [&](double t, const solve::State& x) {
      solve::State dx(equations_.size(), 0.0);
      for (std::size_t k = 0; k < equations_.size(); ++k) {
        const auto& eq = equations_[k];
        for (std::size_t j = 0; j < eq.bindings.size(); ++j) {
          const auto& b = eq.bindings[j];
          if (b.kind == VarKind::Time) eval_buf_[j] = t;
          else if (b.kind == VarKind::State) eval_buf_[j] = x[b.index];
          else eval_buf_[j] = b.index < u.size() ? u[b.index] : 0.0;
        }
        dx[k] = eq.expr(std::span<const double>(eval_buf_.data(), eq.bindings.size()));
      }
      return dx;
    });
    x_ = std::move(state);
    return outputs();
  }

 private:
  std::vector<CompiledEquation> equations_;
  std::string solver_name_;
  std::unique_ptr<solve::Solver> solver_;
  solve::State x_;
  std::vector<double> eval_buf_;
};

}  // namespace

void add_rhs(Registry& registry) {
  auto rhs = make_template(
      "control", "rhs", "ODE RHS", {"u"}, {"x"},
      {{.id = "f", .type = "text", .default_text = "-x"},
       {.id = "x0", .type = "matrix", .default_text = "0"},
       solve::param()},
      [](const Values& p, const Config& c) {
        const std::string f_text = param_text(p, c, "f", "-x");
        const auto parts = split_equations(f_text);
        Vector x0 = parse_initial_state(p, c, parts.size());
        return std::make_unique<RhsBlock>(f_text, std::move(x0), param_text(p, c, "solver", "auto"));
      },
      "User-supplied ODE right-hand side x' = f(t, x, u), separated by ';' for multi-state systems "
      "(x1..xn, u1..um, t). Integrated by the run's solver.");
  rhs.def.inputs = {numbers_port("u")};
  rhs.def.outputs = {numbers_port("x")};
  rhs.def.flows = {"signal"};
  rhs.def.feedthrough = false;
  rhs.def.elementwise = false;
  registry.add(std::move(rhs));
}

}  // namespace paperboat::toolkit::control
