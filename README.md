# paperboat-toolkit-control

The optional control toolkit for [Paperboat](https://github.com/paperboatlab/paperboat).

It adds per-step Signal blocks that use the optimization toolkit's `Workspace`:

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
