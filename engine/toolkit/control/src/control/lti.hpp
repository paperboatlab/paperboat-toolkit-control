#pragma once

// Private to the toolkit: linear time-invariant SISO blocks from a numerator and a
// denominator, and the polynomial helpers the blocks that design one use.
//
// - Lti runs coefficients: s integrated (controllable canonical form, with a solver), z as
//   direct form II transposed (every step, or every sample_time with a hold between).
// - Designed is what a block registers: a design function, called with the step it will
//   run at, gives the coefficients; `realize` says whether s coefficients are integrated or
//   discretized by Tustin. transfer_fn is a design that returns what was typed; a filter
//   prewarps its edges with warp() and returns its analog prototype.
//
// Polynomials are in descending powers, as MATLAB's tf() writes them.

#include "control/compact.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common.hpp"
#include "control/matrix.hpp"
#include "control/sampler.hpp"
#include "paperboat/toolkit/solve.hpp"

namespace paperboat::toolkit::control {

using Complex = std::complex<double>;

inline Coefficients poly_mul(const Coefficients& a, const Coefficients& b) {
  if (a.empty() || b.empty()) return {};
  Coefficients out(a.size() + b.size() - 1, 0.0);
  for (std::size_t i = 0; i < a.size(); ++i)
    for (std::size_t j = 0; j < b.size(); ++j) out[i + j] += a[i] * b[j];
  return out;
}

inline Coefficients poly_pow(const Coefficients& a, std::size_t n) {
  Coefficients out{1.0};
  while (n--) out = poly_mul(out, a);
  return out;
}

// The monic real polynomial with these roots. Complex roots must come in conjugate pairs.
inline Coefficients poly_from_roots(const std::vector<Complex>& roots, std::string_view param) {
  std::vector<Complex> p{1.0};
  for (const Complex& r : roots) {
    std::vector<Complex> next(p.size() + 1, 0.0);
    for (std::size_t i = 0; i < p.size(); ++i) {
      next[i] += p[i];
      next[i + 1] -= r * p[i];
    }
    p = std::move(next);
  }
  double scale = 1.0;
  for (const Complex& c : p) scale = std::max(scale, std::abs(c));
  Coefficients out;
  for (const Complex& c : p) {
    if (std::abs(c.imag()) > 1e-9 * scale)
      throw std::invalid_argument(std::string(param) + ": complex roots must come in conjugate pairs");
    out.push_back(c.real());
  }
  return out;
}

// Roots as text, "-1 -2+3j -2-3j" (i works for j), as zpk and place take them.
inline Complex parse_root(std::string token, std::string_view param) {
  const auto bad = [&] { return std::invalid_argument(std::string(param) + ": \"" + token + "\" is not a number like -2 or -2+3j"); };
  if (token.back() != 'j' && token.back() != 'i') {
    std::size_t i = 0;
    double re = 0;
    if (!read_number(token, i, re) || i != token.size()) throw bad();
    return re;
  }
  std::string body = token.substr(0, token.size() - 1);
  // The sign that starts the imaginary part: the last + or - not at the start and not in an exponent.
  std::size_t split = std::string::npos;
  for (std::size_t i = body.size(); i-- > 1;)
    if ((body[i] == '+' || body[i] == '-') && body[i - 1] != 'e' && body[i - 1] != 'E') {
      split = i;
      break;
    }
  const std::string re_text = split == std::string::npos ? "" : body.substr(0, split);
  std::string im_text = split == std::string::npos ? body : body.substr(split);
  if (im_text.empty() || im_text == "+" || im_text == "-") im_text += "1";
  double re = 0, im = 0;
  std::size_t i = 0;
  if (!re_text.empty() && (!read_number(re_text, i, re) || i != re_text.size())) throw bad();
  i = 0;
  if (!read_number(im_text, i, im) || i != im_text.size()) throw bad();
  return {re, im};
}

inline std::vector<Complex> parse_roots(const std::string& text, std::string_view param) {
  std::vector<Complex> out;
  std::string token;
  for (char ch : text + " ") {
    if (ch == ' ' || ch == ',' || ch == '[' || ch == ']' || ch == '\t' || ch == '\n') {
      if (!token.empty()) out.push_back(parse_root(token, param));
      token.clear();
    } else {
      token += ch;
    }
  }
  return out;
}

inline Coefficients trim_leading_zeros(const Coefficients& coefficients) {
  std::size_t first = 0;
  while (first + 1 < coefficients.size() && coefficients[first] == 0.0) ++first;
  return Coefficients(coefficients.begin() + static_cast<std::ptrdiff_t>(first), coefficients.end());
}

struct ProperTransfer {
  Coefficients num;  // padded to order + 1, descending powers
  Coefficients den;  // padded to order + 1, descending powers, den[0] = 1
  std::size_t order = 0;
};

inline ProperTransfer proper_transfer(const Coefficients& num, const Coefficients& den) {
  if (den.empty()) throw std::invalid_argument("den: expected at least one coefficient");
  Coefficients denominator = den;
  if (denominator[0] == 0.0) throw std::invalid_argument("den: the leading coefficient must be nonzero");
  Coefficients numerator = trim_leading_zeros(num);
  if (numerator.size() > denominator.size())
    throw std::invalid_argument(
        "num: the numerator's degree must not exceed the denominator's for a proper transfer function");
  const double leading = denominator[0];
  for (double& value : numerator) value /= leading;
  for (double& value : denominator) value /= leading;
  numerator.insert(numerator.begin(), denominator.size() - numerator.size(), 0.0);
  const std::size_t order = denominator.size() - 1;
  return {std::move(numerator), std::move(denominator), order};
}

struct Polynomials {
  Coefficients num, den;
};

// N(s)/D(s) with s = P(x)/Q(x), cleared of fractions: the frequency transforms (lowpass to
// highpass, bandpass, bandstop) and Tustin are each one call. den[0] is made 1.
inline Polynomials substitute(const Coefficients& num, const Coefficients& den, const Coefficients& P,
                              const Coefficients& Q) {
  const ProperTransfer tf = proper_transfer(num, den);
  const std::size_t n = tf.order;
  const std::size_t size = n * (std::max(P.size(), Q.size()) - 1) + 1;
  Coefficients b(size, 0.0), a(size, 0.0);
  for (std::size_t i = 0; i <= n; ++i) {  // tf.num[i] multiplies s^(n - i)
    const Coefficients term = poly_mul(poly_pow(P, n - i), poly_pow(Q, i));
    const std::size_t at = size - term.size();  // descending powers: align the constant terms
    for (std::size_t j = 0; j < term.size(); ++j) {
      b[at + j] += tf.num[i] * term[j];
      a[at + j] += tf.den[i] * term[j];
    }
  }
  a = trim_leading_zeros(a);  // keep b and a the same length: pad b back
  b = trim_leading_zeros(b);
  if (b.size() < a.size()) b.insert(b.begin(), a.size() - b.size(), 0.0);
  const double lead = a[0];
  for (double& v : b) v /= lead;
  for (double& v : a) v /= lead;
  return {std::move(b), std::move(a)};
}

// H(s) → H(z) by Tustin, s = (2/T)(z − 1)/(z + 1).
inline Polynomials bilinear(const Coefficients& num, const Coefficients& den, double T) {
  return substitute(num, den, {2.0 / T, -2.0 / T}, {1.0, 1.0});
}

// The analog frequency (rad/s) that Tustin at step T maps onto w: a design that prewarps its
// critical frequencies is exact there. T = 0 (integrated): w itself.
inline double warp(double w, double T, std::string_view param = "cutoff") {
  if (T <= 0) return w;
  if (w * T / 2.0 >= std::numbers::pi / 2.0)
    throw std::invalid_argument(std::string(param) + ": " + std::to_string(w / (2.0 * std::numbers::pi)) +
                                " Hz is at or above the Nyquist frequency of a " + std::to_string(T) +
                                " s step; use a smaller step or a lower frequency");
  return 2.0 / T * std::tan(w * T / 2.0);
}

struct LtiOptions {
  std::string solver = "auto";
  double sample_time = 0.0;   // z: update every sample_time s, hold between; 0 every step
  bool steady_start = false;  // start at the steady state for the first input, not at rest
};

// One realization of (num, den) in s or z.
class Lti : public Block {
 public:
  Lti(const Coefficients& num, const Coefficients& den, std::string_view domain, LtiOptions options)
      : options_(std::move(options)), integrated_(domain == "s") {
    const ProperTransfer tf = proper_transfer(num, den);
    order_ = tf.order;
    num_ = tf.num;
    den_ = tf.den;
    if (integrated_) {
      // Controllable canonical form: A's first row is -a1 ... -an, B is e1, C is the
      // numerator's remainder after the direct term, D is its leading coefficient.
      A_ = Matrix::zeros(order_, order_);
      B_.assign(order_, 0.0);
      C_.assign(order_, 0.0);
      D_ = num_[0];
      if (order_ > 0) {
        for (std::size_t i = 0; i < order_; ++i) A_(0, i) = -den_[i + 1];
        for (std::size_t i = 1; i < order_; ++i) A_(i, i - 1) = 1.0;
        B_[0] = 1.0;
        for (std::size_t i = 0; i < order_; ++i) C_[i] = num_[i + 1] - D_ * den_[i + 1];
      }
      x_.assign(order_, 0.0);
      solver_ = solve::make(options_.solver);
    } else {
      s_.assign(order_, 0.0);
    }
  }

  // Its (A, B, C, D): the controllable canonical form in s, direct form II transposed in z.
  std::optional<LinearModel> linearize(const Values&) const override {
    LinearModel m{.inputs = {"u"}, .outputs = {"y"}, .states = order_};
    const std::size_t n = order_;
    m.A.assign(n * n, 0.0), m.B.assign(n, 0.0), m.C.assign(n, 0.0), m.D = {num_[0]};
    if (integrated_) {
      m.A = A_.values, m.B = B_, m.C = C_, m.D = {D_};
      return m;
    }
    // y = b0 u + s0;  s_i ← b_{i+1} u + s_{i+1} − a_{i+1} y
    for (std::size_t i = 0; i < n; ++i) {
      m.A[i * n] = -den_[i + 1];
      if (i + 1 < n) m.A[i * n + i + 1] = 1.0;
      m.B[i] = num_[i + 1] - den_[i + 1] * num_[0];
    }
    if (n > 0) m.C[0] = 1.0;
    m.discrete = true;
    m.sample_time = options_.sample_time;
    return m;
  }

  Values step(const Context& ctx, const Values& in) override {
    const double u = get_or(in, "u");
    if (!started_) {
      started_ = true;
      sampler_.period = integrated_ ? 0.0 : options_.sample_time;
      if (options_.steady_start) settle(u);
    }
    if (!sampler_.due(ctx.t)) return {{"y", y_}};  // hold between samples
    y_ = integrated_ ? step_solver(ctx, u) : step_discrete(u);
    return {{"y", y_}};
  }

 private:
  // The steady state for a constant input u (none when the block has a pole at dc).
  void settle(double u) {
    if (integrated_) {
      if (order_ > 0 && den_[order_] != 0.0) x_[order_ - 1] = u / den_[order_];
      return;
    }
    double sb = 0.0, sa = 0.0;
    for (std::size_t i = 0; i <= order_; ++i) sb += num_[i], sa += den_[i];
    if (std::abs(sa) < 1e-12) return;
    const double y = sb / sa * u;
    double acc = 0.0;
    for (std::size_t i = order_; i-- > 0;) {
      acc += num_[i + 1] * u - den_[i + 1] * y;
      s_[i] = acc;
    }
  }

  double step_solver(const Context& ctx, double u) {
    double y = D_ * u;  // from this step's input and the state before it is advanced
    for (std::size_t i = 0; i < order_; ++i) y += C_[i] * x_[i];
    if (order_ > 0) {
      solve::State state = x_;
      solver_->step(state, ctx.t, ctx.dt, [&](double, const solve::State& x) {
        solve::State dx(order_, 0.0);
        for (std::size_t i = 0; i < order_; ++i) {
          dx[i] = B_[i] * u;
          for (std::size_t j = 0; j < order_; ++j) dx[i] += A_(i, j) * x[j];
        }
        return dx;
      });
      x_ = std::move(state);
    }
    return y;
  }

  // Direct form II transposed.
  // ponytail: one section of the whole order; cascade biquads (SOS) if high orders ring.
  double step_discrete(double u) {
    const double y = num_[0] * u + (order_ > 0 ? s_[0] : 0.0);
    for (std::size_t i = 0; i + 1 < order_; ++i) s_[i] = num_[i + 1] * u + s_[i + 1] - den_[i + 1] * y;
    if (order_ > 0) s_[order_ - 1] = num_[order_] * u - den_[order_] * y;
    return y;
  }

  LtiOptions options_;
  bool integrated_;
  std::size_t order_ = 0;
  Coefficients num_, den_;
  Matrix A_;
  Vector B_, C_;
  double D_ = 0.0;
  solve::State x_;
  std::unique_ptr<solve::Solver> solver_;
  solve::State s_;
  bool started_ = false;
  Sampler sampler_;
  double y_ = 0.0;
};

// What a design function returns: coefficients and what they mean.
struct Design {
  Coefficients num, den;
  std::string domain = "s";
};
// Called with the step the block will run at (0 when it is integrated): prewarp with warp().
using DesignFn = std::function<Design(double T)>;

// A block that designs its coefficients for the step it runs at, then runs them as an Lti.
class Designed : public Block {
 public:
  Designed(DesignFn design, bool tustin, LtiOptions options)
      : design_(std::move(design)), tustin_(tustin), options_(std::move(options)) {
    build(0.0);  // check the params now, so a bad one fails when the block is made
    if (tustin_) lti_.reset();
  }

  // The design itself, unwarped: an s design as the analog system, a z design as it is.
  std::optional<LinearModel> linearize(const Values& in) const override {
    const Design d = design_(0.0);
    return Lti(d.num, d.den, d.domain, options_).linearize(in);
  }

  Values step(const Context& ctx, const Values& in) override {
    const double T = options_.sample_time > 0 ? options_.sample_time : ctx.dt;
    if (!lti_ || (tustin_ && T != T_)) build(T);  // ponytail: a new dt restarts the state
    return lti_->step(ctx, in);
  }

 private:
  void build(double T) {
    T_ = T;
    const bool discretize = tustin_ && T > 0;
    Design d = design_(discretize ? T : 0.0);
    if (discretize && d.domain == "s") {
      Polynomials z = bilinear(d.num, d.den, T);
      d = {std::move(z.num), std::move(z.den), "z"};
    }
    lti_ = std::make_unique<Lti>(d.num, d.den, d.domain, options_);
  }

  DesignFn design_;
  bool tustin_;
  LtiOptions options_;
  std::unique_ptr<Lti> lti_;
  double T_ = -1.0;
};

// The params every designed block shares; `realize` defaults to what suits the block, and a
// block that only ever designs z coefficients passes "" and gets no realize or solver.
inline std::vector<ParamDef> lti_params(std::string realize_default) {
  std::vector<ParamDef> out;
  if (!realize_default.empty())
    out.push_back({.id = "realize", .type = "enum", .default_text = realize_default, .options = {"solver", "tustin"}});
  out.push_back({.id = "sample_time", .default_value = 0.0, .unit = "s"});
  out.push_back({.id = "start", .type = "enum", .default_text = "zero", .options = {"zero", "steady"}});
  if (!realize_default.empty()) out.push_back(solve::param());
  return out;
}

inline std::unique_ptr<Block> make_designed(const Values& p, const Config& c, std::string realize_default,
                                            DesignFn design) {
  LtiOptions o;
  o.solver = param_text(p, c, "solver", "auto");
  o.sample_time = get_or(p, "sample_time", 0.0);
  o.steady_start = enum_text(p, c, "start", {"zero", "steady"}, "zero") == "steady";
  if (o.sample_time < 0) throw std::invalid_argument("sample_time: expected 0 or more");
  const bool tustin = enum_text(p, c, "realize", {"solver", "tustin"}, realize_default) == "tustin";
  return std::make_unique<Designed>(std::move(design), tustin, std::move(o));
}

// Registers a designed block: `params` then the shared LTI params. A vector signal gets one
// filter per element. Feedthrough: Tustin gives every design a direct term, so these always
// have it; a loop needs a lag, an integrator or a delay in it.
inline void add_designed(Registry& registry, std::string type, std::string label, std::vector<ParamDef> params,
                         std::string realize_default, std::function<DesignFn(const Values&, const Config&)> design,
                         std::string description) {
  for (auto& param : lti_params(realize_default)) params.push_back(std::move(param));
  auto t = make_template(
      "control", std::move(type), std::move(label), {"u"}, {"y"}, std::move(params),
      [design, realize_default](const Values& p, const Config& c) {
        return make_designed(p, c, realize_default, design(p, c));
      },
      std::move(description));
  t.def.flows = {"signal"};
  t.def.feedthrough = true;
  t.def.elementwise = true;
  apply_compact_view(t);
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
