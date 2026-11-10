// The control package: each block is a folder, control/<block>/<block>.cpp, defining
// add_<block>. This registers them.

#include "paperboat/toolkit/control.hpp"

namespace paperboat::toolkit::control {

void add_pid(Registry&);
void add_integrator(Registry&);
void add_lag(Registry&);
void add_delay(Registry&);
void add_mpc(Registry&);
void add_lp_step(Registry&);
void add_nmpc(Registry&);

void register_control(Registry& registry) {
  add_pid(registry);
  add_integrator(registry);
  add_lag(registry);
  add_delay(registry);
  registry.default_flows("control", {"signal", "step"});
  add_mpc(registry);
  add_lp_step(registry);
  add_nmpc(registry);
}

}  // namespace paperboat::toolkit::control
