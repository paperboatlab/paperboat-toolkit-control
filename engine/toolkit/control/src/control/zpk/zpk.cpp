// control/zpk: a transfer function from its zeros, poles and gain, k Π(s − z) / Π(s − p).
// Roots are written "-1 -2+3j -2-3j" (i works for j); complex ones come in conjugate pairs.

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {

void add_zpk(Registry& registry) {
  add_designed(
      registry, "zpk", "Zero-pole-gain",
      {{.id = "zeros", .type = "text", .default_text = ""},
       {.id = "poles", .type = "text", .default_text = "-1"},
       {.id = "gain", .default_value = 1.0},
       {.id = "domain", .type = "enum", .default_text = "s", .options = {"s", "z"}}},
      "solver",
      [](const Values& p, const Config& c) -> DesignFn {
        Coefficients num = poly_from_roots(parse_roots(param_text(p, c, "zeros", ""), "zeros"), "zeros");
        for (double& v : num) v *= get_or(p, "gain", 1.0);
        Design d{num, poly_from_roots(parse_roots(param_text(p, c, "poles", "-1"), "poles"), "poles"),
                 enum_text(p, c, "domain", {"s", "z"}, "s")};
        return [d](double) { return d; };
      },
      "A transfer function from its zeros, poles and gain: k Π(s − z) / Π(s − p). Roots like -1 or -2+3j.");
}

}  // namespace paperboat::toolkit::control
