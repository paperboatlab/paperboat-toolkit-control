// The control package's blocks: MPC (dynamic QP), a per-step LP and NMPC (dynamic NLP). They are
// Signal blocks: no step ports, feedthrough = false, outputs() returns the last control, step()
// solves for the next one.

#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "paperboat/toolkit/block.hpp"
#include "paperboat/toolkit/mpc.hpp"
#include "paperboat/toolkit/optimization.hpp"

namespace paperboat::toolkit::mpc {
namespace {

using optimization::Matrix;
using optimization::Vector;

Matrix zeros(std::size_t rows, std::size_t cols) { return Matrix(rows, Vector(cols, 0.0)); }

Matrix identity(std::size_t n) {
  Matrix out = zeros(n, n);
  for (std::size_t i = 0; i < n; ++i) out[i][i] = 1.0;
  return out;
}

Matrix mul(const Matrix& a, const Matrix& b) {
  if (a.empty() || b.empty() || a.front().size() != b.size())
    throw std::invalid_argument("MPC: matrix dimensions do not fit");
  Matrix out = zeros(a.size(), b.front().size());
  for (std::size_t i = 0; i < a.size(); ++i)
    for (std::size_t k = 0; k < b.size(); ++k) {
      if (a[i][k] == 0.0) continue;
      for (std::size_t j = 0; j < b[k].size(); ++j) out[i][j] += a[i][k] * b[k][j];
    }
  return out;
}

Vector mul(const Matrix& a, const Vector& x) {
  if (a.empty() || a.front().size() != x.size()) throw std::invalid_argument("MPC: matrix/vector dimensions do not fit");
  Vector out(a.size(), 0.0);
  for (std::size_t i = 0; i < a.size(); ++i)
    for (std::size_t j = 0; j < x.size(); ++j) out[i] += a[i][j] * x[j];
  return out;
}

Matrix transpose(const Matrix& a) {
  Matrix out = zeros(a.empty() ? 0 : a.front().size(), a.size());
  for (std::size_t i = 0; i < a.size(); ++i)
    for (std::size_t j = 0; j < a[i].size(); ++j) out[j][i] = a[i][j];
  return out;
}

Matrix add(const Matrix& a, const Matrix& b) {
  if (a.size() != b.size() || (!a.empty() && a.front().size() != b.front().size()))
    throw std::invalid_argument("MPC: matrix dimensions do not fit");
  Matrix out = a;
  for (std::size_t i = 0; i < out.size(); ++i)
    for (std::size_t j = 0; j < out[i].size(); ++j) out[i][j] += b[i][j];
  return out;
}

Matrix block_diag(const std::vector<Matrix>& blocks, std::size_t count) {
  const std::size_t n = blocks.front().size();
  Matrix out = zeros(n * count, n * count);
  for (std::size_t k = 0; k < count; ++k)
    for (std::size_t i = 0; i < n; ++i)
      for (std::size_t j = 0; j < n; ++j) out[k * n + i][k * n + j] = blocks.front()[i][j];
  return out;
}

Matrix power(const Matrix& a, std::size_t exponent) {
  Matrix out = identity(a.size());
  for (std::size_t k = 0; k < exponent; ++k) out = mul(out, a);
  return out;
}

Matrix rows_of(const nlohmann::json& doc, const char* key, const std::string& at) {
  if (!doc.contains(key)) throw std::runtime_error(at + ": needs \"" + key + "\"");
  if (!doc[key].is_array()) throw std::runtime_error(at + ": \"" + key + "\" must be an array of rows");
  for (const auto& row : doc[key])
    if (!row.is_array()) throw std::runtime_error(at + ": \"" + key + "\" must be an array of rows");
  return doc[key].get<Matrix>();
}

Vector vector_of(const nlohmann::json& doc, const char* key, const std::string& at, bool required) {
  if (!doc.contains(key)) {
    if (required) throw std::runtime_error(at + ": needs \"" + key + "\"");
    return {};
  }
  if (!doc[key].is_array()) throw std::runtime_error(at + ": \"" + key + "\" must be an array");
  return doc[key].get<Vector>();
}

struct MpcConfig {
  Matrix A, B, Q, R;
  std::size_t n = 0, m = 0, horizon = 0;
  Vector u_min, u_max;
};

MpcConfig read_mpc(const std::string& file) {
  if (file.empty()) throw std::runtime_error("MPC needs a file: A, B, Q, R and N");
  std::ifstream in(file);
  if (!in) throw std::runtime_error("cannot open " + file);
  nlohmann::json doc;
  try {
    in >> doc;
  } catch (const nlohmann::json::exception& err) {
    throw std::runtime_error(file + ": " + err.what());
  }
  if (!doc.is_object()) throw std::runtime_error(file + ": the model must be a JSON object");

  MpcConfig config;
  config.A = rows_of(doc, "A", file);
  config.B = rows_of(doc, "B", file);
  config.Q = rows_of(doc, "Q", file);
  config.R = rows_of(doc, "R", file);
  config.n = config.A.size();
  config.m = config.B.empty() ? 0 : config.B.front().size();
  config.horizon = doc.value("N", 0);
  if (config.n == 0 || config.m == 0 || config.horizon == 0)
    throw std::runtime_error(file + ": needs A (n×n), B (n×m) and N > 0");
  for (const auto& row : config.A)
    if (row.size() != config.n) throw std::runtime_error(file + ": A must be n×n");
  for (const auto& row : config.B)
    if (row.size() != config.m) throw std::runtime_error(file + ": B must be n×m");
  for (const auto& row : config.Q)
    if (row.size() != config.n) throw std::runtime_error(file + ": Q must be n×n");
  for (const auto& row : config.R)
    if (row.size() != config.m) throw std::runtime_error(file + ": R must be m×m");
  config.u_min = vector_of(doc, "u_min", file, false);
  config.u_max = vector_of(doc, "u_max", file, false);
  if (!config.u_min.empty() && config.u_min.size() != config.m) throw std::runtime_error(file + ": u_min needs m values");
  if (!config.u_max.empty() && config.u_max.size() != config.m) throw std::runtime_error(file + ": u_max needs m values");
  return config;
}

// The condensed QP over U = [u_0 … u_{N-1}]:  min ½U'HU + f'U, u_min <= u_k <= u_max.
optimization::Problem condensed(const MpcConfig& config, const Vector& x0, const Vector& ref) {
  const std::size_t n = config.n, m = config.m, N = config.horizon;
  const std::size_t nx = n * N, nu = m * N;

  Matrix Phi = zeros(nx, n);
  Matrix Gamma = zeros(nx, nu);
  for (std::size_t k = 0; k < N; ++k) {
    const Matrix Ak = power(config.A, k);
    for (std::size_t i = 0; i < n; ++i)
      for (std::size_t j = 0; j < n; ++j) Phi[k * n + i][j] = Ak[i][j];
    for (std::size_t j = 0; j < k; ++j) {
      const Matrix G = mul(power(config.A, k - 1 - j), config.B);
      for (std::size_t i = 0; i < n; ++i)
        for (std::size_t c = 0; c < m; ++c) Gamma[k * n + i][j * m + c] = G[i][c];
    }
  }

  const Matrix Qbar = block_diag({config.Q}, N);
  const Matrix Rbar = block_diag({config.R}, N);
  const Matrix H = add(Rbar, mul(mul(transpose(Gamma), Qbar), Gamma));

  Vector refbar(nx, 0.0);
  for (std::size_t k = 0; k < N; ++k)
    for (std::size_t i = 0; i < n; ++i) refbar[k * n + i] = ref.size() == n ? ref[i] : 0.0;
  Vector target = mul(Phi, x0);
  for (std::size_t i = 0; i < nx; ++i) target[i] -= refbar[i];
  const Vector f = mul(mul(transpose(Gamma), Qbar), target);

  optimization::Problem problem;
  problem.P = H;
  problem.q = f;
  problem.variables = nu;
  if (!config.u_min.empty() || !config.u_max.empty()) {
    problem.A = identity(nu);
    problem.l = config.u_min.empty() ? Vector(nu, -1e20) : Vector(nu, 0.0);
    problem.u = config.u_max.empty() ? Vector(nu, 1e20) : Vector(nu, 0.0);
    for (std::size_t k = 0; k < N; ++k)
      for (std::size_t c = 0; c < m; ++c) {
        problem.l[k * m + c] = config.u_min.empty() ? -1e20 : config.u_min[c];
        problem.u[k * m + c] = config.u_max.empty() ? 1e20 : config.u_max[c];
      }
    problem.constraints = nu;
  }
  return problem;
}

class MpcBlock final : public Block {
 public:
  MpcBlock(const Values& p, const Config& c)
      : file_(c.contains("file") ? c.at("file") : std::string{}),
        config_(read_mpc(file_)),
        u_(config_.m, 0.0),
        max_time_(get_or(p, "max_time", 0.0)),
        options_{.max_iter = static_cast<int>(get_or(p, "max_iter", 4000.0))},
        workspace_(options_) {}

  Values step(const Context&, const Values& in) override {
    Vector x0 = as_vector(in, "y");
    if (x0.size() != config_.n) x0.assign(config_.n, 0.0);
    Vector ref = as_vector(in, "ref");
    if (ref.size() != config_.n) ref.assign(config_.n, 0.0);
    const auto result = workspace_.solve(condensed(config_, x0, ref));
    if (result.status.find("solved") != std::string::npos) {
      for (std::size_t i = 0; i < config_.m && i < result.x.size(); ++i) u_[i] = result.x[i];
    }
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
  std::string file_;
  MpcConfig config_;
  Vector u_;
  double max_time_;
  optimization::Options options_;
  optimization::Workspace workspace_;
  std::string status_ = "idle";
  double iterations_ = 0.0;
  double solve_time_ = 0.0;
};

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

Template mpc_template() {
  Template t;
  t.name = "mpc/mpc";
  t.def.type = "mpc";
  t.def.label = "MPC";
  t.def.kind = Kind::Function;
  t.def.inputs = {numbers_port("y"), numbers_port("ref"), numbers_port("x0")};
  t.def.outputs = {numbers_port("u"), PortDef{.id = "status", .type = "text"}, PortDef{.id = "iterations"},
                   PortDef{.id = "solve_time"}};
  t.def.params = {{.id = "file", .type = "file"},
                  {.id = "max_time", .default_value = 0.0, .type = "number"},
                  {.id = "max_iter", .default_value = 4000.0, .type = "number"}};
  t.def.feedthrough = false;
  t.def.description = "Solves a condensed QP every clock step (A, B, Q, R, N, u bounds from a JSON file).";
  t.def.view = {.compact = false, .value = "{file}"};
  t.make = [](const Values& p, const Config& c) { return std::make_unique<MpcBlock>(p, c); };
  return t;
}

Template lp_step_template() {
  Template t;
  t.name = "mpc/lp_step";
  t.def.type = "lp_step";
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

Template nmpc_template() {
  Template t;
  t.name = "mpc/nmpc";
  t.def.type = "nmpc";
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

void register_mpc(Registry& registry) {
  registry.add(mpc_template());
  registry.add(lp_step_template());
  registry.add(nmpc_template());
}

}  // namespace paperboat::toolkit::mpc
