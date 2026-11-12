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
void add_transfer_fn(Registry&);
void add_state_space(Registry&);
void add_bode(Registry&);
void add_controllability(Registry&);
void add_observability(Registry&);

void register_control(Registry& registry) {
  add_pid(registry);
  add_integrator(registry);
  add_lag(registry);
  add_delay(registry);
  registry.default_flows("control", {"signal", "step"});
  add_transfer_fn(registry);
  add_state_space(registry);
  add_bode(registry);
  add_controllability(registry);
  add_observability(registry);
  add_mpc(registry);
  add_lp_step(registry);
  add_nmpc(registry);
}

}  // namespace paperboat::toolkit::control
