# paperboat-toolkit-control

The optional control toolkit for [Paperboat](https://github.com/paperboatlab/paperboat): PID and the
clock dynamics, transfer functions and state space, filters, nonlinear and discrete blocks, LQR,
observers and Kalman filters, and per-step MPC on the optimization toolkit.

**Docs:** <https://paperboatlab.github.io/toolkits/control/>, also in the Help window (F1) of
Paperboat. Their sources live with the code: `engine/toolkit/control/docs/` for the toolkit's
pages, and a block's notes in its folder (`src/control/<block>/doc.rst`):
a block's ports and params come from the catalog, so write only what the catalog can't say.

## Install

```sh
paperboat toolkit install optimization --repo https://github.com/paperboatlab/paperboat-toolkit-optimization.git
paperboat toolkit install control --repo https://github.com/paperboatlab/paperboat-toolkit-control.git
```

Then in the station's `main`:

```cpp
tk::Registry r = tk::builtins();
tk::optimization::register_optimization(r);
tk::control::register_control(r);
```

and import `"optimization"` and `"control"` in `descriptor.toml`.

## Licence

GNU General Public License version 3 (see [LICENSE](LICENSE)). Installed by the user and never
bundled into a Paperboat release artifact.
