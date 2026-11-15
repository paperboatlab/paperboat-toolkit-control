``n`` filters the derivative (0: none). With ``u_min`` < ``u_max`` the output is limited (equal: no
limit), and ``anti_windup`` keeps the integral from running away while it is: ``none``, ``clamp``
(stop integrating) or ``back_calc`` (bleed the excess back at rate ``kt``). PID is linear for the
Analysis drawer when ``n > 0``.
