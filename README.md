# paperboat-toolkit-control

The optional control toolkit for [Paperboat](https://github.com/paperboatlab/paperboat).

It carries the clock dynamics that used to be built into the engine:

| Block | What it does |
| --- | --- |
| `control/pid` | `e` → `u`; gains `kp`, `ki`, `kd`; `n` filters the derivative (0: none); `u_min` < `u_max` limits `u` (equal: none) with `anti_windup` = `none`, `clamp` or `back_calc` (rate `kt`) |
| `control/integrator` | `u` → `y`; `initial`, optional `solver` |
| `control/lag` | first-order lag `u` → `y`; `tau`, optional `solver` |
| `control/delay` | its input `steps` clock steps ago; `initial` |

`integrator`, `lag` and `delay` have no direct feedthrough, so a feedback loop may pass through
them. A vector signal gets one state per element.

It also adds frequency-domain and state-space blocks. Polynomial coefficients and matrices are
text params in MATLAB style, with polynomial coefficients in descending powers:

| Block | What it does |
| --- | --- |
| `control/transfer_fn` | `u` → `y`; `num`, `den`, `domain` (`s` or `z`). `s`: controllable canonical realization, integrated (`realize = solver`) or discretized by Tustin at the step (`realize = tustin`); `z`: direct form II transposed |
| `control/state_space` | `u` → `y`, `x`; `A`, `B`, `C`, `D` text matrices, `domain` (`s` or `z`), optional `solver` |
| `control/bode` | `w` → `magnitude` (dB), `phase` (degrees); `num`, `den`, `domain`, `sample_time` for `z` |
| `control/controllability` | `A`, `B` text matrices → `rank`, `order`, `controllable` |
| `control/observability` | `A`, `C` text matrices → `rank`, `order`, `observable` |

Linear and filter blocks, built on the same core:

| Block | What it does |
| --- | --- |
| `control/second_order` | k wn² / (s² + 2 zeta wn s + wn²); `wn`, `zeta`, `gain` |
| `control/zpk` | k Π(s − z) / Π(s − p); `zeros`, `poles` (`"-1 -2+3j -2-3j"`), `gain`, `domain` |
| `control/lead_lag` | k (s/zero + 1) / (s/pole + 1) |
| `control/derivative` | filtered d/dt, s / (tau s + 1) |
| `control/washout` | first-order highpass, tau s / (tau s + 1) |
| `control/notch` | cuts `depth` dB at `freq` Hz; `width` |
| `control/filter` | `family` (`butterworth`, `chebyshev1`, `chebyshev2`, `bessel`), `kind` (`lowpass`, `highpass`, `bandpass`, `bandstop`), `order` 1–10, `cutoff`/`cutoff2` Hz, `ripple`, `attenuation`. Matches SciPy's `butter`/`cheby1`/`cheby2`/`bessel(norm="mag")` |
| `control/biquad` | one Audio EQ Cookbook section: `lowpass`, `highpass`, `bandpass`, `notch`, `allpass`, `peak`, `lowshelf`, `highshelf` |
| `control/moving_average` | mean of the last `window` samples |
| `control/ema` | y += alpha (u − y) |
| `control/median` | median of the last `window` (odd) samples: removes spikes |

Filters default to `realize = tustin` with their edges prewarped, so a cutoff is exact at any
step; plants (`transfer_fn`, `zpk`, `second_order`) default to `solver`. Every one is
elementwise and has feedthrough (Tustin gives a direct term): a loop needs a `lag`,
`integrator` or `delay` in it.

Nonlinear and discrete blocks:

| Block | What it does |
| --- | --- |
| `control/rate_limiter` | follows `u` no faster than `rise` up, `fall` down (per s) |
| `control/backlash` | gear play of `width` |
| `control/quantizer` | `u` in steps of `interval`, `round` or `floor` |
| `control/relay` | hysteresis switch: `on_value` from `on_at`, `off_value` from `off_at` |
| `control/friction` | Coulomb + viscous + Stribeck force at velocity `u` |
| `control/discrete_integrator` | `forward`, `backward` or `trapezoid`; `lower`/`upper`; input `reset` |
| `control/zoh`, `control/foh` | zero- and first-order hold at `sample_time` |
| `control/discrete_derivative` | (u[n] − u[n−1]) / T |
| `control/tapped_delay` | last `taps` samples as a numbers list, newest first |

Linear blocks share `realize`, `sample_time` (z and Tustin: update every `sample_time` s and
hold between; 0 every step), `start` (`zero`, or `steady`: begin at the steady state for the
first input, no start-up transient) and `solver`.

A coefficient list is `"1 2 1"` or `"[1, 2, 1]"`; a matrix is `"1 0; 0 1"` or
`"[[1, 0], [0, 1]]"`. `transfer_fn` takes a signal `u` and gives a signal `y`; `state_space`
takes a numbers list `u` and gives numbers lists `y` and `x`. `controllability` and
`observability` have no signal inputs: their matrices are params, and they report the rank of
`[B AB …]` and `[C; CA; …]`.

It also adds per-step Signal blocks that use the optimization toolkit's `Workspace`:

| Block | Problem |
| --- | --- |
| `control/mpc` | a condensed QP from `A`, `B`, `Q`, `R`, `N` and `u` bounds |
| `control/lp_step` | an LP every clock step |
| `control/nmpc` | an NLP every clock step (needs Ipopt) |

They are `feedthrough = false`: `outputs()` returns the last `u`; `step()` reads `y`/`ref` and
solves for the next one. Outputs are `u`, `status`, `iterations` and `solve_time`. A `max_time`
param turns a solve that misses the step into `status = "late"` while keeping the last `u`.

The model is a JSON file on the block:

```json
{ "A": [[1]], "B": [[1]], "Q": [[1]], "R": [[0.1]], "N": 20, "u_min": [-1], "u_max": [1] }
```

`lp_step` takes the optimization LP form (`c`, `A`, `l`, `u`); `nmpc` takes the NLP form
(`variables`, `objective`, `constraints`). The control toolkit links
`paperboat::toolkit_optimization`; install both toolkits.

## Install

```sh
paperboat toolkit install optimization --repo https://github.com/paperboatlab/paperboat-toolkit-optimization.git
paperboat toolkit install control --repo https://github.com/paperboatlab/paperboat-toolkit-control.git
```

Then in the backend's `main`:

```cpp
tk::Registry r = tk::builtins();
tk::optimization::register_optimization(r);
tk::control::register_control(r);
```

and import `"optimization"` and `"control"` in `descriptor.toml`.
