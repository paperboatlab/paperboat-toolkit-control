// control/zpk: a transfer function from its zeros, poles and gain, k Π(s − z) / Π(s − p).
// Roots are written "-1 -2+3j -2-3j" (i works for j); complex ones come in conjugate pairs.

#include <string>
#include <string_view>
#include <vector>

#include "common.hpp"
#include "control/lti.hpp"

namespace paperboat::toolkit::control {
namespace {

Complex parse_root(std::string token, std::string_view param) {
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

std::vector<Complex> parse_roots(const std::string& text, std::string_view param) {
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

}  // namespace

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
