#pragma once

// Private to the toolkit: when a block with a sample_time takes its next sample, and how the
// small Signal blocks register.

#include <utility>

#include "common.hpp"

namespace paperboat::toolkit::control {

// due(t) is true at the first call and then once every `period` s of run time (every call
// when period <= 0); between, the block holds what it last gave.
struct Sampler {
  double period = 0.0;
  bool started = false;
  double next = 0.0;

  bool due(double t) {
    if (period <= 0) return true;
    if (!started) started = true, next = t;
    if (t + 1e-9 * period < next) return false;
    while (next <= t + 1e-9 * period) next += period;
    return true;
  }
};

// A Signal-flow block with feedthrough; a vector input gets one block per element.
inline void add_signal(Registry& registry, Template t, bool elementwise = true) {
  t.def.flows = {"signal"};
  t.def.elementwise = elementwise;
  registry.add(std::move(t));
}

}  // namespace paperboat::toolkit::control
