// control/linear: the control toolkit's analysis (paperboat#370, control#14). A subsystem or a
// selection, linearized: each block's (A, B, C, D) (Block::linearize, or differences of step()
// for a stateless one) joined along the wires into one model from the part's inputs to its
// outputs; then, for one input → output pair, its poles, zeros, Bode plot, margins and step.

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "common.hpp"
#include "control/lti.hpp"
#include "control/riccati.hpp"
#include "paperboat/toolkit/analysis.hpp"

namespace paperboat::toolkit::control {
namespace {

using json = nlohmann::json;
using std::numbers::pi;

// A channel: one scalar of a port; "u" is (u, 0), "u#2" is (u, 2).
std::pair<std::string, std::size_t> channel(const std::string& name) {
  const auto at = name.find('#');
  if (at == std::string::npos) return {name, 0};
  return {name.substr(0, at), std::stoul(name.substr(at + 1))};
}

// A stateless block by differences of step() around zero inputs.
LinearModel by_differences(const AnalysisBlock& b, double dt) {
  LinearModel m;
  for (const auto& p : b.def.inputs)
    if (p.type != "step") m.inputs.push_back(p.id);
  Values zero;
  for (const auto& id : m.inputs) zero[id] = 0.0;
  const Context ctx{.t = 0, .dt = dt};
  const Values y0 = b.block->step(ctx, zero);
  for (const auto& p : b.def.outputs)
    if (const auto it = y0.find(p.id); it != y0.end() && std::holds_alternative<double>(it->second)) m.outputs.push_back(p.id);
  const double h = 1e-6;
  m.D.assign(m.outputs.size() * m.inputs.size(), 0.0);
  for (std::size_t j = 0; j < m.inputs.size(); ++j) {
    Values in = zero;
    in[m.inputs[j]] = h;
    const Values y = b.block->step(ctx, in);
    for (std::size_t i = 0; i < m.outputs.size(); ++i)
      m.D[i * m.inputs.size() + j] = (get_or(y, m.outputs[i]) - get_or(y0, m.outputs[i])) / h;
  }
  return m;
}

struct Joined {
  Matrix A, B, C, D;
  bool discrete = false;
  double T = 0.0;
  std::vector<std::string> inputs, outputs;  // "node.port" or "node.port[k]"
};

// Every block's model in one: x' = A x + B v, w = C x + D v over all block channels, then the
// wires v = M w + E u and the outputs y = P w close it: w = F (C x + D E u), F = (I − D M)⁻¹.
Joined join(const AnalysisRequest& req) {
  struct Part { const AnalysisBlock* b; LinearModel m; std::size_t x0, v0, w0; };
  std::vector<Part> parts;
  std::size_t nx = 0, nv = 0, nw = 0;
  std::optional<bool> discrete;
  double T = 0.0;
  for (const auto& b : req.blocks) {
    std::optional<LinearModel> m = b.block->linearize({});
    if (!m && b.block->stateless()) m = by_differences(b, req.dt);
    if (!m) throw std::invalid_argument(b.id + " (" + b.def.type + ") can't be linearized: take it out of the part, or give it a linear model");
    if (!m->inputs.empty() || !m->outputs.empty() || m->states > 0) {
      if (m->states > 0) {
        if (discrete && *discrete != m->discrete)
          throw std::invalid_argument("the part mixes continuous (s) and discrete (z) blocks; " + b.id + " is " + (m->discrete ? "discrete" : "continuous"));
        discrete = m->discrete;
        if (m->discrete) {
          const double t = m->sample_time > 0 ? m->sample_time : req.dt;
          if (T > 0 && std::abs(t - T) > 1e-12 * T) throw std::invalid_argument("the part's discrete blocks run at different sample times");
          T = t;
        }
      }
    }
    parts.push_back({&b, std::move(*m), nx, nv, nw});
    nx += parts.back().m.states, nv += parts.back().m.inputs.size(), nw += parts.back().m.outputs.size();
  }
  Joined out;
  out.discrete = discrete.value_or(false);
  out.T = T;
  Matrix A = Matrix::zeros(nx, nx), B = Matrix::zeros(nx, nv), C = Matrix::zeros(nw, nx), D = Matrix::zeros(nw, nv);
  for (const auto& p : parts) {
    const auto& m = p.m;
    const std::size_t n = m.states, mi = m.inputs.size(), mo = m.outputs.size();
    for (std::size_t r = 0; r < n; ++r) {
      for (std::size_t c = 0; c < n; ++c) A(p.x0 + r, p.x0 + c) = m.A[r * n + c];
      for (std::size_t c = 0; c < mi; ++c) B(p.x0 + r, p.v0 + c) = m.B[r * mi + c];
    }
    for (std::size_t r = 0; r < mo; ++r) {
      for (std::size_t c = 0; c < n; ++c) C(p.w0 + r, p.x0 + c) = m.C[r * n + c];
      for (std::size_t c = 0; c < mi; ++c) D(p.w0 + r, p.v0 + c) = m.D[r * mi + c];
    }
  }
  const auto find_part = [&](const std::string& id) -> const Part* {
    for (const auto& p : parts)
      if (p.b->id == id) return &p;
    return nullptr;
  };
  const auto name = [](const std::string& node, const std::string& ch) {
    const auto [port, k] = channel(ch);
    return node + "." + port + (ch.find('#') == std::string::npos ? "" : "[" + std::to_string(k) + "]");
  };
  // Inputs: each boundary input's channels.
  std::vector<std::pair<std::size_t, std::size_t>> u_of_v;  // (v index, u index)
  for (const auto& [node, port] : req.inputs) {
    const Part* p = find_part(node);
    if (!p) continue;  // a monitor or a commented-out block
    for (std::size_t c = 0; c < p->m.inputs.size(); ++c)
      if (channel(p->m.inputs[c]).first == port) {
        u_of_v.emplace_back(p->v0 + c, out.inputs.size());
        out.inputs.push_back(name(node, p->m.inputs[c]));
      }
  }
  std::vector<std::size_t> w_of_y;
  for (const auto& [node, port] : req.outputs) {
    const Part* p = find_part(node);
    if (!p) continue;
    for (std::size_t c = 0; c < p->m.outputs.size(); ++c)
      if (channel(p->m.outputs[c]).first == port) {
        w_of_y.push_back(p->w0 + c);
        out.outputs.push_back(name(node, p->m.outputs[c]));
      }
  }
  if (out.inputs.empty()) throw std::invalid_argument("the part has no inputs: nothing comes in from outside it");
  if (out.outputs.empty()) throw std::invalid_argument("the part has no outputs: nothing it computes goes out");
  Matrix M = Matrix::zeros(nv, nw), E = Matrix::zeros(nv, out.inputs.size()), P = Matrix::zeros(out.outputs.size(), nw);
  for (const auto& p : parts)
    for (std::size_t c = 0; c < p.m.inputs.size(); ++c) {
      const auto [port, k] = channel(p.m.inputs[c]);
      const auto wire = req.wires.find({p.b->id, port});
      if (wire == req.wires.end()) continue;
      const Part* src = find_part(wire->second.first);
      if (!src) continue;
      for (std::size_t o = 0; o < src->m.outputs.size(); ++o) {
        const auto [sport, sk] = channel(src->m.outputs[o]);
        if (sport == wire->second.second && sk == k) M(p.v0 + c, src->w0 + o) = 1.0;
      }
    }
  for (const auto& [v, u] : u_of_v) E(v, u) = 1.0;
  for (std::size_t i = 0; i < w_of_y.size(); ++i) P(i, w_of_y[i]) = 1.0;
  const Matrix F = inverse(add(identity(nw), multiply(D, M), -1.0), "the part (an algebraic loop: a loop with no lag, integrator or delay in it)");
  const Matrix FC = multiply(F, C), FDE = multiply(multiply(F, D), E);
  out.A = add(A, multiply(multiply(B, M), FC));
  out.B = multiply(B, add(multiply(M, FDE), E));
  out.C = multiply(P, FC);
  out.D = multiply(P, FDE);
  return out;
}

// Roots of a polynomial (descending powers): roots at 0 exactly, the rest by Aberth–Ehrlich.
// ponytail: fine to order ~15; a Hessenberg QR on the companion matrix if higher orders lose digits.
std::vector<Complex> roots(Coefficients p) {
  p = trim_leading_zeros(p);
  std::vector<Complex> out;
  if (p.size() <= 1 || p[0] == 0.0) return out;
  while (p.size() > 1 && p.back() == 0.0) p.pop_back(), out.push_back(0.0);
  const std::size_t n = p.size() - 1;
  if (n == 0) return out;
  double bound = 0;
  for (std::size_t i = 1; i <= n; ++i) bound = std::max(bound, std::abs(p[i] / p[0]));
  std::vector<Complex> z(n);
  for (std::size_t i = 0; i < n; ++i) z[i] = std::polar(0.5 * (1 + bound), 2 * pi * static_cast<double>(i) / static_cast<double>(n) + 0.4);
  const auto eval = [&](Complex x) {
    Complex v = 0, d = 0;
    for (double c : p) d = d * x + v, v = v * x + c;
    return std::pair{v, d};
  };
  for (int it = 0; it < 2000; ++it) {
    double moved = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const auto [v, d] = eval(z[i]);
      if (v == 0.0) continue;
      const Complex ratio = v / d;
      Complex sum = 0;
      for (std::size_t j = 0; j < n; ++j)
        if (j != i) sum += 1.0 / (z[i] - z[j]);
      const Complex w = ratio / (1.0 - ratio * sum);
      z[i] -= w;
      moved = std::max(moved, std::abs(w) / (1 + std::abs(z[i])));
    }
    if (moved < 1e-15) break;
  }
  for (Complex& r : z) {
    if (std::abs(r.imag()) < 1e-7 * (1 + std::abs(r))) r = r.real();
    out.push_back(r);
  }
  std::ranges::sort(out, [](Complex a, Complex b) { return a.real() != b.real() ? a.real() > b.real() : a.imag() > b.imag(); });
  return out;
}

// The SISO transfer function of (A, b, c, d), by Faddeev–LeVerrier: den = det(sI − A),
// num = c adj(sI − A) b + d den.
std::pair<Coefficients, Coefficients> transfer(const Matrix& A, const Matrix& b, const Matrix& c, double d) {
  const std::size_t n = A.rows;
  Coefficients den(n + 1, 0.0), num(n + 1, 0.0);
  den[0] = 1.0;
  Matrix N = identity(n);
  for (std::size_t k = 0; k < n; ++k) {
    num[k + 1] = multiply(multiply(c, N), b)(0, 0);  // c N_k b multiplies s^(n−1−k)
    const Matrix AN = multiply(A, N);
    double tr = 0;
    for (std::size_t i = 0; i < n; ++i) tr += AN(i, i);
    den[k + 1] = -tr / static_cast<double>(k + 1);
    N = add(AN, identity(n), den[k + 1]);
  }
  for (std::size_t i = 0; i <= n; ++i) num[i] += d * den[i];
  return {num, den};
}

Complex at(const Coefficients& p, Complex x) {
  Complex v = 0;
  for (double c : p) v = v * x + c;
  return v;
}

json complex_list(const std::vector<Complex>& z) {
  json out = json::array();
  for (const Complex& r : z) out.push_back({r.real(), r.imag()});
  return out;
}

json matrix_json(const Matrix& m) {
  json out = json::array();
  for (std::size_t r = 0; r < m.rows; ++r) {
    json row = json::array();
    for (std::size_t c = 0; c < m.cols; ++c) row.push_back(m(r, c));
    out.push_back(std::move(row));
  }
  return out;
}

json analyse(const AnalysisRequest& req) {
  const Joined sys = join(req);
  const std::size_t iu = std::min<std::size_t>(req.options.value("input", 0), sys.inputs.size() - 1);
  const std::size_t iy = std::min<std::size_t>(req.options.value("output", 0), sys.outputs.size() - 1);
  const std::size_t n = sys.A.rows;
  Matrix b = Matrix::zeros(n, 1), c = Matrix::zeros(1, n);
  for (std::size_t i = 0; i < n; ++i) b(i, 0) = sys.B(i, iu), c(0, i) = sys.C(iy, i);
  const double d = sys.D(iy, iu);
  auto [num, den] = transfer(sys.A, b, c, d);
  // A numerator coefficient that is rounding noise is zero.
  double scale = 0;
  for (double v : num) scale = std::max(scale, std::abs(v));
  for (double& v : num)
    if (std::abs(v) < 1e-10 * scale) v = 0;
  const std::vector<Complex> poles = roots(den), zeros = roots(num);
  const bool discrete = sys.discrete;
  const double T = sys.T;
  // stable: every pole inside; marginal: some on the boundary (an integrator), none outside.
  bool stable = true, outside = false;
  for (const Complex& p : poles) {
    const double margin = discrete ? 1 - std::abs(p) : -p.real();
    stable = stable && margin > 1e-9;
    outside = outside || margin < -1e-9;
  }
  const char* stability = stable ? "stable" : outside ? "unstable" : "marginal";

  // Bode: from a decade under the slowest pole or zero to one over the fastest (to Nyquist in z).
  double lo = 1e9, hi = 0;
  for (const auto* set : {&poles, &zeros})
    for (Complex r : *set) {
      const double w = discrete ? std::abs(std::log(r)) / T : std::abs(r);
      if (w > 1e-9) lo = std::min(lo, w), hi = std::max(hi, w);
    }
  if (hi == 0) lo = 1, hi = 1;
  double wmin = lo / 100, wmax = discrete ? pi / T : hi * 100;
  if (discrete) wmin = std::min(wmin, wmax / 1000);
  const int N = 400;
  json w = json::array(), mag = json::array(), phase = json::array();
  std::vector<double> ws, dbs, degs;
  double unwrap = 0, last = 0;
  for (int k = 0; k < N; ++k) {
    const double wk = wmin * std::pow(wmax / wmin, k / double(N - 1));
    const Complex s = discrete ? std::polar(1.0, wk * T) : Complex(0, wk);
    const Complex h = at(num, s) / at(den, s);
    double deg = std::arg(h) * 180 / pi;
    if (k > 0) {
      while (deg + unwrap - last > 180) unwrap -= 360;
      while (deg + unwrap - last < -180) unwrap += 360;
    }
    last = deg + unwrap;
    ws.push_back(wk), dbs.push_back(20 * std::log10(std::abs(h))), degs.push_back(last);
  }
  // Start the phase where it belongs at low frequency: within (−360, 180].
  const double shift = 360 * std::ceil((degs.front() - 180) / 360);
  for (double& v : degs) v -= shift;

  // Margins: where |H| crosses 0 dB (phase margin) and the phase crosses −180° (gain margin).
  json margins = {{"gm_db", nullptr}, {"pm_deg", nullptr}, {"w_gc", nullptr}, {"w_pc", nullptr}};
  const auto cross = [&](std::size_t k, const std::vector<double>& v, double level) {
    const double f = (level - v[k]) / (v[k + 1] - v[k]);
    return std::exp(std::log(ws[k]) + f * (std::log(ws[k + 1]) - std::log(ws[k])));
  };
  const auto lerp = [&](std::size_t k, const std::vector<double>& from, const std::vector<double>& v, double level) {
    const double f = (level - from[k]) / (from[k + 1] - from[k]);
    return v[k] + f * (v[k + 1] - v[k]);
  };
  for (std::size_t k = 0; k + 1 < ws.size(); ++k)
    if (std::isfinite(dbs[k]) && std::isfinite(dbs[k + 1]) && (dbs[k] - 0) * (dbs[k + 1] - 0) <= 0 && dbs[k] != dbs[k + 1]) {
      const double p = lerp(k, dbs, degs, 0);
      margins["w_gc"] = cross(k, dbs, 0);
      margins["pm_deg"] = 180 + p - 360 * std::floor((180 + p + 180) / 360);
      break;
    }
  for (std::size_t k = 0; k + 1 < ws.size(); ++k) {
    const double level = -180 + 360 * std::round((degs[k] + 180) / 360);  // the nearest odd multiple of 180
    if ((degs[k] - level) * (degs[k + 1] - level) <= 0 && degs[k] != degs[k + 1]) {
      margins["w_pc"] = cross(k, degs, level);
      margins["gm_db"] = -lerp(k, degs, dbs, level);
      break;
    }
  }
  for (std::size_t k = 0; k < ws.size(); ++k) w.push_back(ws[k]), mag.push_back(dbs[k]), phase.push_back(degs[k]);

  // Step response: x[k+1] = Ad x + Bd, y = c x + d, over about five of the slowest time constants.
  double slow = 0;
  for (Complex p : poles) {
    const double rate = discrete ? -std::log(std::max(std::abs(p), 1e-300)) / T : -p.real();
    if (rate > 1e-9) slow = slow == 0 ? rate : std::min(slow, rate);
  }
  double t_end = slow > 0 ? std::min(8.0 / slow, 1e5) : 10.0;
  int steps = 500;
  Matrix Ad, Bd;
  double h = t_end / steps;
  if (discrete) {
    h = T, steps = std::clamp(static_cast<int>(std::ceil(t_end / T)), 20, 2000), t_end = steps * T;
    Ad = sys.A, Bd = b;
  } else {
    Matrix Mx = Matrix::zeros(n + 1, n + 1);
    for (std::size_t r = 0; r < n; ++r) {
      for (std::size_t k = 0; k < n; ++k) Mx(r, k) = sys.A(r, k) * h;
      Mx(r, n) = b(r, 0) * h;
    }
    const Matrix E = expm(Mx);
    Ad = Matrix::zeros(n, n), Bd = Matrix::zeros(n, 1);
    for (std::size_t r = 0; r < n; ++r) {
      for (std::size_t k = 0; k < n; ++k) Ad(r, k) = E(r, k);
      Bd(r, 0) = E(r, n);
    }
  }
  json st = json::array(), sy = json::array();
  Matrix x = Matrix::zeros(n, 1);
  std::vector<double> ys;
  for (int k = 0; k <= steps; ++k) {
    const double y = multiply(c, x)(0, 0) + d;
    st.push_back(k * h), sy.push_back(y), ys.push_back(y);
    x = add(multiply(Ad, x), Bd);
  }
  json info = json::object();
  const double dc_den = std::abs(at(den, discrete ? 1.0 : 0.0));
  const std::optional<double> dc = dc_den > 1e-12 ? std::optional((at(num, discrete ? 1.0 : 0.0) / at(den, discrete ? 1.0 : 0.0)).real()) : std::nullopt;
  if (stable && dc && std::abs(*dc) > 1e-12) {
    const double f = *dc;
    const auto first = [&](double frac) {
      for (std::size_t k = 0; k < ys.size(); ++k)
        if (ys[k] / f >= frac) return static_cast<double>(k) * h;
      return std::numeric_limits<double>::quiet_NaN();
    };
    double peak = ys[0] / f;
    for (double v : ys) peak = std::max(peak, v / f);
    std::size_t settled = 0;
    for (std::size_t k = 0; k < ys.size(); ++k)
      if (std::abs(ys[k] / f - 1) > 0.02) settled = k + 1;
    info = {{"rise_time", first(0.9) - first(0.1)}, {"overshoot_pct", std::max(0.0, (peak - 1) * 100)},
            {"settling_time", settled < ys.size() ? static_cast<double>(settled) * h : std::numeric_limits<double>::quiet_NaN()}};
  }

  return {{"domain", discrete ? "z" : "s"}, {"sample_time", T}, {"order", n},
          {"inputs", sys.inputs}, {"outputs", sys.outputs}, {"input", iu}, {"output", iy},
          {"A", matrix_json(sys.A)}, {"B", matrix_json(sys.B)}, {"C", matrix_json(sys.C)}, {"D", matrix_json(sys.D)},
          {"num", num}, {"den", den}, {"poles", complex_list(poles)}, {"zeros", complex_list(zeros)},
          {"stable", stable}, {"stability", stability}, {"dc_gain", dc ? json(*dc) : json(nullptr)},
          {"bode", {{"w", w}, {"mag_db", mag}, {"phase_deg", phase}}}, {"margins", margins},
          {"step", {{"t", st}, {"y", sy}, {"info", info}}}};
}

}  // namespace

void add_linear(Registry& registry) {
  registry.add(AnalysisTemplate{
      .name = "control/linear",
      .def = {.label = "Linear analysis", .description = "Poles, zeros, Bode, gain and phase margins and step response of a subsystem or selection"},
      .run = analyse});
}

}  // namespace paperboat::toolkit::control
