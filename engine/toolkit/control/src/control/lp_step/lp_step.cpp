// A linear program solved every clock step, from a JSON file. A Signal block: feedthrough =
// false, outputs() returns the last solution and step() solves for the next one.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "paperboat/toolkit/block.hpp"
#include "paperboat/toolkit/optimization.hpp"

namespace paperboat::toolkit::control {
namespace {

using optimization::Vector;

class LpStepBlock final : public Block {
 public:
  LpStepBlock(const Values& p, const Config& c)
      : problem_(optimization::read_problem(c.contains("file") ? c.at("file") : std::string{}, false)),
        u_(problem_.variables, 0.0),
        max_time_(get_or(p, "max_time", 0.0)),
        options_{.max_iter = static_cast<int>(get_or(p, "max_iter", 4000.0))},
        workspace_(options_) {}

  Values step(const Context&, const Values&) override {
    const auto result = workspace_.solve(problem_);
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
  optimization::Problem problem_;
  Vector u_;
  double max_time_;
  optimization::Options options_;
  optimization::Workspace workspace_;
  std::string status_ = "idle";
  double iterations_ = 0.0;
  double solve_time_ = 0.0;
};

Template lp_step_template() {
  Template t;
  t.name = "control/lp_step";
  t.def.type = "lp_step";
  t.def.flows = {"signal"};
  t.def.label = "Dynamic LP";
  t.def.kind = Kind::Function;
  t.def.inputs = {numbers_port("y"), numbers_port("ref")};
  t.def.outputs = {numbers_port("u"), PortDef{.id = "status", .type = "text"}, PortDef{.id = "iterations"},
                   PortDef{.id = "solve_time"}};
  t.def.params = {{.id = "file", .type = "file"},
                  {.id = "max_time", .default_value = 0.0, .type = "number"},
                  {.id = "max_iter", .default_value = 4000.0, .type = "number"}};
  t.def.feedthrough = false;
  t.def.description = "Solves a linear program every clock step from a JSON file.";
  t.def.view = {.compact = false, .value = "{file}"};
  t.make = [](const Values& p, const Config& c) { return std::make_unique<LpStepBlock>(p, c); };
  return t;
}

}  // namespace

void add_lp_step(Registry& registry) { registry.add(lp_step_template()); }

}  // namespace paperboat::toolkit::control
