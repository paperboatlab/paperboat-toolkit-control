// NMPC: a nonlinear program solved every clock step, from a JSON file (needs Ipopt). A Signal
// block: feedthrough = false, outputs() returns the last control and step() solves for the next.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "paperboat/toolkit/block.hpp"
#include "paperboat/toolkit/optimization.hpp"

namespace paperboat::toolkit::control {
namespace {

using optimization::Vector;

class NmpcBlock final : public Block {
 public:
  NmpcBlock(const Values& p, const Config& c)
      : problem_(optimization::read_nlp_problem(c.contains("file") ? c.at("file") : std::string{})),
        u_(problem_.variables.size(), 0.0),
        max_time_(get_or(p, "max_time", 0.0)),
        options_{.eps_abs = get_or(p, "tol", 1e-7),
                 .max_iter = static_cast<int>(get_or(p, "max_iter", 500.0))},
        workspace_(options_) {}

  Values step(const Context&, const Values& in) override {
    Vector x0 = as_vector(in, "y");
    const auto result = workspace_.solve(problem_, x0);
    if (result.status.find("solved") != std::string::npos) u_ = result.x;
    status_ = result.status;
    if (max_time_ > 0 && result.solve_time > max_time_ && status_.find("solved") != std::string::npos) status_ = "late";
    iterations_ = result.iterations;
    solve_time_ = result.solve_time;
    return outputs();
  }

  Values outputs() const override {
    return {{"u", u_}, {"status", status_}, {"iterations", iterations_}, {"solve_time", solve_time_}};
  }

 private:
  optimization::NlpProblem problem_;
  Vector u_;
  double max_time_;
  optimization::Options options_;
  optimization::NlpWorkspace workspace_;
  std::string status_ = "idle";
  double iterations_ = 0.0;
  double solve_time_ = 0.0;
};

Template nmpc_template() {
  Template t;
  t.name = "control/nmpc";
  t.def.type = "nmpc";
  t.def.flows = {"signal"};
  t.def.label = "NMPC";
  t.def.kind = Kind::Function;
  t.def.inputs = {numbers_port("y"), numbers_port("ref"), numbers_port("x0")};
  t.def.outputs = {numbers_port("u"), PortDef{.id = "status", .type = "text"}, PortDef{.id = "iterations"},
                   PortDef{.id = "solve_time"}};
  t.def.params = {{.id = "file", .type = "file"},
                  {.id = "max_time", .default_value = 0.0, .type = "number"},
                  {.id = "tol", .default_value = 1e-7, .type = "number"},
                  {.id = "max_iter", .default_value = 500.0, .type = "number"}};
  t.def.feedthrough = false;
  t.def.description = "Solves a nonlinear program every clock step from a JSON file (needs Ipopt).";
  t.def.view = {.compact = false, .value = "{file}"};
  t.make = [](const Values& p, const Config& c) { return std::make_unique<NmpcBlock>(p, c); };
  return t;
}

}  // namespace

void add_nmpc(Registry& registry) { registry.add(nmpc_template()); }

}  // namespace paperboat::toolkit::control
