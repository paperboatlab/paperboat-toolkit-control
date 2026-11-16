control
=======

The optional control toolkit for Paperboat: the clock dynamics (``pid``, ``integrator``, ``lag``,
``delay``), transfer functions and state space, filters, nonlinear and discrete blocks,
state-space design and estimation, and per-step MPC on the optimization toolkit.

.. toctree::
   :maxdepth: 1

   install
   blocks/index

Dynamics and loops
------------------

``integrator``, ``lag``, ``delay``, ``observer`` and ``rhs`` have no direct feedthrough, so a feedback
loop may pass through them. Every other block has feedthrough (Tustin gives a direct term), so a loop
needs one of those in it. A vector signal gets one state per element.

Linear blocks
-------------

Polynomial coefficients and matrices are text params in MATLAB style, coefficients in descending
powers: a coefficient list is ``"1 2 1"`` or ``"[1, 2, 1]"``, a matrix ``"1 0; 0 1"`` or
``"[[1, 0], [0, 1]]"``. A number for ``Q`` or ``R`` is that times the identity.

The linear blocks share ``realize`` (``solver``: integrated; ``tustin``: discretized at the step),
``sample_time`` (z and Tustin: update every ``sample_time`` s and hold between; 0 every step),
``start`` (``zero``, or ``steady``: begin at the steady state for the first input, with no start-up
transient) and ``solver``. Filters default to ``realize = tustin`` with their edges prewarped, so a
cutoff is exact at any step; plants (``transfer_fn``, ``zpk``, ``second_order``) default to
``solver``.

``rhs`` integrates an ODE you type, x' = f(t, x, u): ``f`` is one expression per state separated by
``;`` (``x1``…``xn``, ``u1``…``um``, ``t``), ``x0`` the start, and the run's solver integrates it.
``state_space`` also takes an ``x0`` (zero when empty).

Design blocks (``lqr``, ``place``, ``kalman_gain``, ``c2d``) have no inputs and give matrices as
text (``"1 2; 3 4"``), so their output wires into ``state_feedback``'s ``K`` input or binds like a
variable.

Linear analysis
---------------

Pick blocks or a subsystem and press **Analyse**: the Analysis drawer shows its order and
stability, gain and phase margins, dc gain, Bode, pole-zero map and step response, for one
input → output pair. Each block gives its linear model (the LTI blocks, PID with ``n > 0``,
integrator, lag, state_space); stateless blocks (``gain``, ``sum``…) are linearized by differences.
Loops inside the part are solved; a block that has no linear model (``median``, ``relay``…) is
named. The panel is this toolkit's ``web/`` part, so the entry points appear only when it is
installed.

Predictive control
------------------

``mpc``, ``lp_step`` and ``nmpc`` solve a problem every clock step with the optimization toolkit's
``Workspace``. They have no feedthrough: they output the last ``u``, then read ``y`` and ``ref`` and
solve for the next one. Outputs are ``u``, ``status``, ``iterations`` and ``solve_time``; a
``max_time`` param turns a solve that misses the step into ``status = "late"`` while keeping the
last ``u``.
