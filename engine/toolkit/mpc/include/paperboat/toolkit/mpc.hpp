#pragma once

// The control package: model predictive control blocks that solve a QP, LP or NLP every clock
// step. It links paperboat::toolkit_optimization; the optimization toolkit owns the solver and
// workspace API, this package owns the control loop.
//
//   tk::Registry r = tk::builtins();
//   tk::optimization::register_optimization(r);
//   tk::mpc::register_mpc(r);

#include "paperboat/toolkit/registry.hpp"

namespace paperboat::toolkit::mpc {

void register_mpc(Registry& registry);

}  // namespace paperboat::toolkit::mpc
