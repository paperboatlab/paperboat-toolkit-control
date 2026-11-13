// control/transfer_fn: a transfer function H(s) or H(z) from a numerator and a denominator.
//
// The coefficients are in descending powers, as MATLAB's tf() writes them:
//
//   H(s) = (b0 s^m + ... + bm) / (a0 s^n + ... + an)
//   H(z) = (b0 z^m + ... + bm) / (a0 z^n + ... + an)
//
// It is the shared Lti block (control/lti.hpp) with its coefficients typed in.

#include <memory>
#include <utility>

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_transfer_fn(Registry& registry) {
  add_designed(
      registry, "transfer_fn", "Transfer function",
      {{.id = "num", .type = "text", .default_text = "1"},
       {.id = "den", .type = "text", .default_text = "1 1"},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}}},
      "solver",
      [](const Values& p, const Config& c) -> DesignFn {
        Design d{parse_coefficients(param_text(p, c, "num", "1"), "num"),
                 parse_coefficients(param_text(p, c, "den", "1 1"), "den"), enum_text(p, c, "domain", {"s", "z"}, "s")};
        return [d](double) { return d; };
      },
      "A transfer function H(s) or H(z) from its numerator and denominator coefficients (descending powers). "
      "s: integrated (controllable canonical form) or discretized by Tustin; z: direct form II transposed.");
}

}  // namespace paperboat::toolkit::control
